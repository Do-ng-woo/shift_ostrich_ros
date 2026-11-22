#!/usr/bin/env python3
"""Prepare the Fusion export for ROS 2 / Gazebo Sim without changing CAD frames."""
import json
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import ConvexHull
from model_geometry import transform, stl_vertices

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'urdf/Shift_ostrich_walking.xacro'


def repair_rounded_inertias(robot):
    """Project invalid rounded CAD principal moments to a physical tensor.

    Minimal Euclidean correction of principal moments (Frobenius tensor norm),
    with a 1e-12 kg m^2 positive/triangle margin. This is a numerical repair,
    not a measurement of the actual inertia. CAD source remains unchanged.
    """
    repairs = {}
    for link in robot.findall('link'):
        inertia = link.find('inertial/inertia')
        a = {key: float(value) for key, value in inertia.attrib.items()}
        tensor = np.array([[a['ixx'], a['ixy'], a['ixz']],
                           [a['ixy'], a['iyy'], a['iyz']],
                           [a['ixz'], a['iyz'], a['izz']]])
        values, axes = np.linalg.eigh(tensor)
        if values[0] > 0 and values[2] <= values[0] + values[1]:
            continue
        if values[0] < 0:
            raise ValueError(f'{link.get("name")}: negative principal inertia; inspect CAD')
        original = dict(inertia.attrib)
        corrected = np.maximum(values, 1e-12)
        deficit = corrected[2] - corrected[0] - corrected[1] + 1e-12
        if deficit > 0:
            corrected += np.array([1., 1., -1.]) * deficit / 3
        fixed = axes @ np.diag(corrected) @ axes.T
        for key, i, j in [('ixx',0,0),('iyy',1,1),('izz',2,2),
                          ('ixy',0,1),('ixz',0,2),('iyz',1,2)]:
            inertia.set(key, format(fixed[i,j], '.17g'))
        repairs[link.get('name')] = {'original': original, 'generated': dict(inertia.attrib),
                                    'method': 'minimum principal-moment correction with 1e-12 margin'}
    return repairs


def collision_hull(source, destination):
    vertices = np.unique(stl_vertices(source), axis=0)
    hull = ConvexHull(vertices)
    with destination.open('wb') as stream:
        stream.write(b'Shift Ostrich convex collision hull'.ljust(80, b'\0'))
        stream.write(struct.pack('<I', len(hull.simplices)))
        for face, equation in zip(hull.simplices, hull.equations):
            triangle = vertices[face].copy()
            if np.dot(np.cross(triangle[1]-triangle[0], triangle[2]-triangle[0]), equation[:3]) < 0:
                triangle[[1, 2]] = triangle[[2, 1]]
            stream.write(struct.pack('<12fH', *equation[:3], *triangle.ravel(), 0))
    return {'source_vertices': len(vertices), 'hull_triangles': len(hull.simplices),
            'hull_volume_mesh_units': float(hull.volume)}



def main():
    robot = ET.parse(SOURCE).getroot()
    robot.set('name', 'shift_ostrich')
    # The .gazebo/.trans includes target Gazebo Classic / ROS 1. Use only CAD data.
    for child in list(robot):
        if child.tag.startswith('{'):
            robot.remove(child)
    materials = ET.parse(SOURCE.parent / 'materials.xacro').getroot()
    robot.extend(list(materials))
    mesh_dir = ROOT / 'meshes'
    mesh_dir.mkdir(exist_ok=True)
    collision_dir = mesh_dir / 'collision'
    collision_dir.mkdir(exist_ok=True)
    hulls = {}
    for mesh in robot.findall('.//mesh'):
        name = mesh.get('filename').rsplit('/', 1)[-1]
        source = mesh_dir / name
        if not source.is_file():
            raise FileNotFoundError(source)
        mesh.set('filename', f'package://shift_ostrich/meshes/{name}')
    for mesh in robot.findall('link/collision/geometry/mesh'):
        name = mesh.get('filename').rsplit('/', 1)[-1]
        if name not in hulls:
            hulls[name] = collision_hull(mesh_dir / name, collision_dir / name)
        mesh.set('filename', f'package://shift_ostrich/meshes/collision/{name}')
    joints = robot.findall('joint')
    for joint in joints:
        if joint.get('type') != 'fixed':
            # Simulation experiment: unbounded angular travel. CAD source and
            # debug/reference/original_joint_angles.json retain original limits.
            joint.set('type', 'continuous')
            limit = joint.find('limit')
            if limit is not None:
                limit.attrib.pop('lower', None)
                limit.attrib.pop('upper', None)
            # Small passive damping for the optional free physics preview.
            dynamics = joint.find('dynamics')
            if dynamics is None:
                ET.SubElement(joint, 'dynamics', damping='0.05', friction='0.0')
    # Keep exporter axes, masses and mesh scales/origins; repair invalid rounded tensors only.
    inertia_repairs = repair_rounded_inertias(robot)
    # Effort / velocity values are CAD placeholders, not calibrated motor specs.
    gazebo = ET.SubElement(robot, 'gazebo')
    plugin = ET.SubElement(gazebo, 'plugin', filename='gz-sim-joint-state-publisher-system',
                           name='gz::sim::systems::JointStatePublisher')
    ET.SubElement(plugin, 'topic').text = '/model/shift_ostrich/joint_state'

    links = {link.get('name'): link for link in robot.findall('link')}
    children = {joint.find('child').get('link') for joint in joints}
    roots = set(links) - children
    if len(roots) != 1:
        raise ValueError(f'Expected one root link, got {roots}')
    root = roots.pop()
    poses = {root: np.eye(4)}
    pending = list(joints)
    while pending:
        ready = [joint for joint in pending if joint.find('parent').get('link') in poses]
        if not ready:
            raise ValueError('Disconnected or cyclic joint tree')
        for joint in ready:
            poses[joint.find('child').get('link')] = poses[joint.find('parent').get('link')] @ transform(joint.find('origin'))
            pending.remove(joint)
    bounds = {}
    for name, link in links.items():
        points = []
        for collision in link.findall('collision'):
            mesh = collision.find('geometry/mesh')
            vertices = stl_vertices(ROOT / mesh.get('filename').removeprefix('package://shift_ostrich/'))
            vertices = vertices * np.array(list(map(float, mesh.get('scale', '1 1 1').split())))
            pose = poses[name] @ transform(collision.find('origin'))
            points.append(vertices @ pose[:3, :3].T + pose[:3, 3])
        vertices = np.concatenate(points)
        bounds[name] = {'min': vertices.min(axis=0).tolist(), 'max': vertices.max(axis=0).tolist()}
    lower = np.min([b['min'] for b in bounds.values()], axis=0)
    upper = np.max([b['max'] for b in bounds.values()], axis=0)
    metadata = {'root_link': root, 'zero_pose_bounds_m': {'min': lower.tolist(), 'max': upper.tolist()},
                'ground_clearance_m': 0.005, 'spawn_z_m': float(0.005 - lower[2]), 'links': bounds,
                'source': str(SOURCE.relative_to(ROOT)), 'collision_method': 'convex hull per mesh',
                'collision_meshes': hulls,
                'inertia_repairs': inertia_repairs,
                'total_mass_kg': sum(float(m.get('value')) for m in robot.findall('link/inertial/mass'))}
    output = ROOT / 'urdf'
    output.mkdir(exist_ok=True)
    ET.indent(robot, space='  ')
    ET.ElementTree(robot).write(output / 'shift_ostrich.urdf', encoding='utf-8', xml_declaration=True)
    # Separate controlled model; ordinary gazebo.launch remains a passive preview.
    robot.remove(gazebo)
    control = ET.SubElement(robot, 'ros2_control', name='GazeboSystem', type='system')
    hardware = ET.SubElement(control, 'hardware')
    ET.SubElement(hardware, 'plugin').text = 'gz_ros2_control/GazeboSimSystem'
    for joint in joints:
        if joint.get('type') == 'fixed':
            continue
        entry = ET.SubElement(control, 'joint', name=joint.get('name'))
        ET.SubElement(entry, 'command_interface', name='position')
        state = ET.SubElement(entry, 'state_interface', name='position')
        ET.SubElement(state, 'param', name='initial_value').text = '0.0'
        ET.SubElement(entry, 'state_interface', name='velocity')
    gazebo = ET.SubElement(robot, 'gazebo')
    plugin = ET.SubElement(gazebo, 'plugin', filename='gz_ros2_control-system',
                           name='gz_ros2_control::GazeboSimROS2ControlPlugin')
    ET.SubElement(plugin, 'parameters').text = '$(find shift_ostrich)/config/controllers.yaml'
    ET.indent(robot, space='  ')
    ET.ElementTree(robot).write(output / 'shift_ostrich_controlled.urdf', encoding='utf-8', xml_declaration=True)
    (output / 'spawn_geometry.json').write_text(json.dumps(metadata, indent=2) + '\n')
    print(f'{len(links)} links, {len(joints)} joints; root={root}')
    print(f'Zero-pose bounds (m): {lower} .. {upper}')
    print(f'Spawn z: {metadata["spawn_z_m"]:.6f} m (5 mm clearance)')


if __name__ == '__main__':
    main()
