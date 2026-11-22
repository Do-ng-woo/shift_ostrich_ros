"""Original STL surface collision queries; adjacent assembly contacts are reported."""
import ctypes
from itertools import combinations
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from .model_geometry import joint_poses, stl_vertices, transform


class MeshInterference:
    def __init__(self, root, library):
        self.root = Path(root)
        self.robot = ET.parse(self.root/'urdf/shift_ostrich.urdf').getroot()
        self.lib = ctypes.CDLL(str(library))
        array = np.ctypeslib.ndpointer(dtype=np.float64, flags='C_CONTIGUOUS')
        self.lib.shift_mesh_create.argtypes = [array, ctypes.c_int]
        self.lib.shift_mesh_create.restype = ctypes.c_void_p
        self.lib.shift_mesh_destroy.argtypes = [ctypes.c_void_p]
        self.lib.shift_mesh_contact.argtypes = [ctypes.c_void_p, array, ctypes.c_void_p, array, array]
        self.lib.shift_mesh_contact.restype = ctypes.c_int
        self.meshes = {}
        self.corners = {}
        for link in self.robot.findall('link'):
            # Original visual STL, not the filled convex collision hull.
            triangles = []
            for visual in link.findall('visual'):
                mesh = visual.find('geometry/mesh')
                path = self.root/mesh.get('filename').removeprefix('package://shift_ostrich/')
                v = stl_vertices(path) * np.fromstring(mesh.get('scale', '1 1 1'), sep=' ')
                t = transform(visual.find('origin'))
                triangles.append(v@t[:3,:3].T+t[:3,3])
            v = np.ascontiguousarray(np.concatenate(triangles), dtype=np.float64)
            name = link.get('name')
            self.meshes[name] = self.lib.shift_mesh_create(v, len(v))
            lower, upper = v.min(axis=0), v.max(axis=0)
            self.corners[name] = np.array([[x,y,z] for x in (lower[0],upper[0])
                for y in (lower[1],upper[1]) for z in (lower[2],upper[2])])
        self.adjacent = {frozenset((j.find('parent').get('link'),j.find('child').get('link')))
                         for j in self.robot.findall('joint')}

    def contacts(self, positions, pairs=None):
        poses = joint_poses(self.robot, positions)
        boxes = {}
        for name, corners in self.corners.items():
            t = poses[name]
            v = corners@t[:3,:3].T+t[:3,3]
            boxes[name] = (v.min(axis=0),v.max(axis=0))
        contacts = []
        for a,b in (combinations(self.meshes,2) if pairs is None else pairs):
            if np.any(boxes[a][1] < boxes[b][0]) or np.any(boxes[b][1] < boxes[a][0]):
                continue
            point = np.zeros(3)
            kind = self.lib.shift_mesh_contact(self.meshes[a],np.ascontiguousarray(poses[a]),
                                              self.meshes[b],np.ascontiguousarray(poses[b]),point)
            if kind:
                contacts.append({'links':[a,b], 'adjacent':frozenset((a,b)) in self.adjacent,
                                 'transverse_surface_crossing':kind == 2,
                                 'point_base_m':point.tolist()})
        return contacts

    def close(self):
        for mesh in self.meshes.values():
            self.lib.shift_mesh_destroy(mesh)
        self.meshes.clear()


def first_sequence_contact(checker, joint_names, stages, max_step_deg=2.):
    """Reject sampled surface contact; passing is not continuous certification."""
    previous = None
    for name, values in stages:
        target = np.asarray(values, dtype=float)
        count = 1 if previous is None else max(1, int(np.ceil(np.max(np.abs(target-previous))/np.radians(max_step_deg))))
        for index in range(1,count+1):
            q = target if previous is None else previous+(target-previous)*(index/count)
            contacts = checker.contacts(dict(zip(joint_names,q)))
            if contacts:
                return {'target_stage':name,'segment_fraction':index/count,'positions':q.tolist(),'contacts':contacts}
        previous = target
    return None


