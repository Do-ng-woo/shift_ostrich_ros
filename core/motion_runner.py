"""Shared FollowJointTrajectory client for the two terminal commands."""
import json
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint

from .trajectory import load_motion
from .hardware_motion import bridge_parameters, run_hardware_motion


def main(motion, args=None):
    rclpy.init(args=args)
    node = Node('shift_ostrich_'+motion)
    node.declare_parameter('backend', 'auto')
    node.declare_parameter('result_file', '')
    node.declare_parameter('metrics_file', '')
    node.declare_parameter('trajectory_file', '')
    observer = None
    client = None
    try:
        share = Path(get_package_share_directory('shift_ostrich'))
        if not node.get_parameter('trajectory_file').value:
            manifest = json.loads((share/'config/motions/verified.json').read_text())
            if manifest.get('verification_status') not in (None, 'passed'):
                raise RuntimeError('Saved motions require a Gazebo replay for the updated CAD model')
        data = load_motion(share, motion, node.get_parameter('trajectory_file').value)
        backend = node.get_parameter('backend').value
        if backend not in ('auto', 'hardware', 'gazebo'):
            raise ValueError('backend must be auto, hardware, or gazebo')
        if backend == 'auto' and node.get_parameter('use_sim_time').value:
            backend = 'gazebo'
        hardware = None if backend == 'gazebo' else bridge_parameters(node)
        if hardware is not None:
            node.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=False)])
            report = run_hardware_motion(node, share, motion, data)
            output = node.get_parameter('result_file').value
            if output:
                Path(output).write_text(json.dumps(report, indent=2)+'\n')
            return
        if backend == 'hardware':
            raise RuntimeError('Hardware bridge is unavailable; start hardware_display.launch.py first')
        node.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        client = ActionClient(node, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')
        node.get_logger().info('Waiting for Gazebo trajectory controller...')
        if not client.wait_for_server(timeout_sec=60.):
            raise RuntimeError('Gazebo trajectory controller is unavailable; start gazebo.launch.py first')
        manager = node.create_client(ListControllers, '/controller_manager/list_controllers')
        if not manager.wait_for_service(timeout_sec=60.):
            raise RuntimeError('Controller manager is unavailable')
        deadline = time.monotonic()+60.
        while time.monotonic() < deadline:
            future = manager.call_async(ListControllers.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=3.)
            if future.done() and future.result() and any(c.name == 'joint_trajectory_controller' and c.state == 'active'
                                                        for c in future.result().controller):
                break
            rclpy.spin_once(node, timeout_sec=.1)
        else:
            raise RuntimeError('Trajectory controller did not activate')
        node.destroy_client(manager)
        if node.get_parameter('metrics_file').value:
            from core.simulation_metrics import SimulationMetrics
            observer = SimulationMetrics()
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = data['joint_names']
        for stage in data['stages']:
            point = JointTrajectoryPoint()
            point.positions = stage['positions']
            point.velocities = [0.]*len(data['joint_names'])
            point.time_from_start.sec, point.time_from_start.nanosec = divmod(round(stage['time_sec']*1e9),1_000_000_000)
            goal.trajectory.points.append(point)
        seen = set(); last_error = []
        def feedback(message):
            nonlocal last_error
            value = message.feedback
            elapsed = value.desired.time_from_start.sec+value.desired.time_from_start.nanosec*1e-9
            index = next((i for i,s in enumerate(data['stages']) if elapsed <= s['time_sec']),len(data['stages'])-1)
            if index not in seen:
                seen.add(index)
                node.get_logger().info(f'Stage {index+1}/{len(data["stages"])}: {data["stages"][index]["name"]}')
            if observer:
                observer.stage = data['stages'][index]['name']
            last_error = list(value.error.positions)
        sent = client.send_goal_async(goal, feedback_callback=feedback)
        rclpy.spin_until_future_complete(node, sent, timeout_sec=15.)
        if not sent.done() or not sent.result().accepted:
            raise RuntimeError('Trajectory was not accepted')
        start = time.monotonic()
        finished = sent.result().get_result_async()
        rclpy.spin_until_future_complete(node, finished, timeout_sec=max(180.,data['stages'][-1]['time_sec']*5+30))
        if not finished.done():
            sent.result().cancel_goal_async()
            raise RuntimeError('Trajectory timed out; check Gazebo is playing')
        response = finished.result()
        if observer:
            end = node.get_clock().now().nanoseconds+2_000_000_000
            deadline = time.monotonic()+15.
            observer.stage = 'final_hold'
            while node.get_clock().now().nanoseconds < end and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.05)
            observer.save(node.get_parameter('metrics_file').value)
        report = {'motion':motion,'action_status':response.status,'error_code':response.result.error_code,
                  'error_string':response.result.error_string,'wall_duration_sec':time.monotonic()-start,
                  'stages':[s['name'] for s in data['stages']],'joint_names':data['joint_names'],
                  'last_feedback_error_rad':last_error}
        output = node.get_parameter('result_file').value
        if output:
            Path(output).write_text(json.dumps(report,indent=2)+'\n')
        node.get_logger().info(f'{motion} finished: status={response.status}, code={response.result.error_code}')
        if response.status != 4 or response.result.error_code != 0:
            raise RuntimeError(response.result.error_string or 'Trajectory failed')
    finally:
        if client:
            client.destroy()
        node.destroy_node()
        rclpy.shutdown()
