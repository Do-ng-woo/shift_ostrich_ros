"""Observe native Gazebo poses and contacts; no physical-state changes."""
import json
import math
from pathlib import Path
import threading
import xml.etree.ElementTree as ET

import numpy as np
from gz.transport13 import Node
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.contacts_pb2 import Contacts


class SimulationMetrics:
    def __init__(self):
        self.lock = threading.Lock()
        self.poses = []
        self.contacts = []
        self.com = []
        self.ground_support = []
        from ament_index_python.packages import get_package_share_directory
        robot = ET.parse(Path(get_package_share_directory('shift_ostrich')) / 'urdf/shift_ostrich.urdf').getroot()
        self.inertials = {}
        for link in robot.findall('link'):
            inertial = link.find('inertial')
            self.inertials[link.get('name')] = (float(inertial.find('mass').get('value')),
                np.fromstring(inertial.find('origin').get('xyz'), sep=' '))
        self.total_mass = sum(m for m,c in self.inertials.values())
        self.stage = 'startup'
        self.node = Node()
        self.node.subscribe(Pose_V, '/model/shift_ostrich/pose', self.on_pose)
        self.node.subscribe(Contacts, '/model/shift_ostrich/contacts', self.on_contacts)

    def on_pose(self, message):
        t = message.header.stamp.sec + message.header.stamp.nsec * 1e-9
        for pose in message.pose:
            if pose.name != 'shift_ostrich':
                continue
            if not t:
                t = pose.header.stamp.sec + pose.header.stamp.nsec * 1e-9
            q = pose.orientation
            pitch = math.asin(max(-1., min(1., 2*(q.w*q.y-q.z*q.x))))
            roll = math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y))
            with self.lock:
                self.poses.append([t, pose.position.x, pose.position.y, pose.position.z, roll, pitch, self.stage])
                root_rotation = rotation(pose.orientation)
                root_position = np.array([pose.position.x,pose.position.y,pose.position.z])
                weighted = np.zeros(3)
                total_mass = 0.
                for link_pose in message.pose:
                    link_name = link_pose.name.rsplit('::',1)[-1]
                    if link_name not in self.inertials:
                        continue
                    mass, center = self.inertials[link_name]
                    local = rotation(link_pose.orientation) @ center + np.array([
                        link_pose.position.x,link_pose.position.y,link_pose.position.z])
                    weighted += mass*(root_rotation @ local + root_position)
                    total_mass += mass
                if abs(total_mass-self.total_mass)<1e-8:
                    self.com.append([t,*(weighted/total_mass).tolist(),self.stage])
            break

    def on_contacts(self, message):
        for contact in message.contact:
            a, b = contact.collision1.name, contact.collision2.name
            t = contact.header.stamp.sec + contact.header.stamp.nsec * 1e-9
            if not t:
                t = message.header.stamp.sec + message.header.stamp.nsec * 1e-9
            ground = 'ground_plane' in a or 'ground_plane' in b
            depth = max(contact.depth, default=0.)
            force = max((math.sqrt(w.body_1_wrench.force.x**2+w.body_1_wrench.force.y**2+w.body_1_wrench.force.z**2)
                         for w in contact.wrench), default=0.)
            with self.lock:
                self.contacts.append([t, a, b, depth, force, ground, self.stage])
                if ground and force>0.1 and contact.position:
                    self.ground_support.append([t,min(p.x for p in contact.position),
                        max(p.x for p in contact.position),a if 'ground_plane' not in a else b,self.stage])

    def save(self, path):
        with self.lock:
            poses = list(self.poses)
            com = list(self.com)
            ground_support = list(self.ground_support)
            # Two sensors can report the same pair in the same simulation step.
            contacts = list({(c[0], *sorted(c[1:3])): c for c in self.contacts}.values())
        report = {'poses': poses, 'contacts': contacts, 'columns': {
            'poses': ['time_s','x_m','y_m','z_m','roll_rad','pitch_rad','stage'],
            'contacts': ['time_s','collision1','collision2','depth_m','force_N','ground','stage']}}
        report['com'] = com
        report['ground_support'] = ground_support
        report['com_sample_count'] = len(com)
        report['total_mass_kg'] = self.total_mass
        if len(poses)>2:
            data = np.array([p[:6] for p in poses], dtype=float)
            # Pose timestamps may repeat when the observer reconnects.
            data = data[np.r_[True, np.diff(data[:,0]) > 1e-6]]
            dt = np.diff(data[:,0])
            report['peak_pitch_rate_rad_s'] = float(np.max(np.abs(np.diff(np.unwrap(data[:,5]))/dt)))
            report['peak_downward_speed_m_s'] = float(max(0., np.max(-np.diff(data[:,3])/dt)))
            report['pitch_range_deg'] = np.degrees([data[:,5].min(), data[:,5].max()]).tolist()
            report['final_pose'] = data[-1].tolist()
        internal = [c for c in contacts if not c[5]]
        report['internal_collision_pairs'] = sorted({tuple(sorted(c[1:3])) for c in internal})
        report['internal_contact_count'] = len(internal)
        report['peak_internal_depth_m'] = max((c[3] for c in internal), default=0.)
        report['peak_ground_force_N'] = max((c[4] for c in contacts if c[5]), default=0.)
        report['pose_sample_count'] = len(poses)
        elbows = [c for c in contacts if c[5] and c[4]>1.0 and any('::'+name+'::' in c[1]+c[2]
                   for name in ('ARM_link2_1','ARM_link3_1'))]
        report['first_elbow_support_time_s'] = min((c[0] for c in elbows),default=None)
        report['contact_sample_count'] = len(contacts)
        Path(path).write_text(json.dumps(report, indent=2)+'\n')
        return {k:v for k,v in report.items() if k not in ('poses','contacts','columns','com','ground_support')}


def rotation(q):
    x,y,z,w=q.x,q.y,q.z,q.w
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
