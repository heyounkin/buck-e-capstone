#!/usr/bin/env python3
"""Plan 20 simulated myCobot 280 random-start -> Buck-E READY paths.

No trajectory is executed or sent to hardware. Uses the running MoveIt
/move_group services. Output: ~/bucke_sim_ws/ready_trials.csv.
"""

import csv
import math
import random
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy
from rcl_interfaces.srv import GetParameters
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetMotionPlan, GetStateValidity

GROUP = 'arm'
POSE_NAME = 'bucke_ready'
TRIALS = 20
MAX_RANDOM_DRAWS = 500
OUT = Path.home() / 'bucke_sim_ws' / 'ready_trials.csv'


def await_response(node, client, request, timeout=12.0):
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done():
        raise TimeoutError(f'Timed out calling {client.srv_name}')
    result = future.result()
    if result is None:
        raise RuntimeError(f'No response from {client.srv_name}: {future.exception()}')
    return result


def client_for(node, srv_type, name):
    client = node.create_client(srv_type, name)
    if not client.wait_for_service(timeout_sec=15.0):
        raise RuntimeError(f'{name} is unavailable. Keep move_group.launch.py running.')
    return client


def make_robot_state(names, values):
    state = RobotState()
    state.is_diff = True  # Keep the non-arm joints from the current scene.
    state.joint_state.name = list(names)
    state.joint_state.position = [float(v) for v in values]
    return state


def main():
    rclpy.init()
    node = rclpy.create_node('bucke_ready_trials')
    try:
        param_client = client_for(node, GetParameters, '/move_group/get_parameters')
        response = await_response(
            node, param_client,
            GetParameters.Request(names=['robot_description', 'robot_description_semantic'])
        )
        if len(response.values) != 2 or any(not v.string_value for v in response.values):
            raise RuntimeError('Could not read the running MoveIt URDF/SRDF parameters')

        urdf = ET.fromstring(response.values[0].string_value)
        srdf = ET.fromstring(response.values[1].string_value)
        pose = next((s for s in srdf.findall('group_state')
                     if s.get('group') == GROUP and s.get('name') == POSE_NAME), None)
        if pose is None:
            raise RuntimeError('bucke_ready missing from MoveIt SRDF; restart MoveIt')
        joints = pose.findall('joint')
        names = [j.attrib['name'] for j in joints]
        ready = [float(j.attrib['value']) for j in joints]
        if len(names) != 6:
            raise RuntimeError(f'Expected 6 arm joints in bucke_ready; got {names}')

        urdf_joints = {j.attrib['name']: j for j in urdf.findall('joint')}
        limits = []
        for name in names:
            joint = urdf_joints.get(name)
            if joint is None:
                raise RuntimeError(f'Joint {name} missing from URDF')
            limit = joint.find('limit')
            if joint.get('type') == 'continuous':
                lo, hi = -math.pi, math.pi
            elif limit is not None:
                lo = float(limit.attrib['lower'])
                hi = float(limit.attrib['upper'])
            else:
                raise RuntimeError(f'Joint {name} has no position limits')
            margin = math.radians(2)
            limits.append((lo + margin, hi - margin))
            if limits[-1][0] >= limits[-1][1]:
                raise RuntimeError(f'Invalid joint range for {name}')

        validity_client = client_for(node, GetStateValidity, '/check_state_validity')
        planning_client = client_for(node, GetMotionPlan, '/plan_kinematic_path')

        def valid(angles):
            request = GetStateValidity.Request()
            request.group_name = GROUP
            request.robot_state = make_robot_state(names, angles)
            return await_response(node, validity_client, request).valid

        if not valid(ready):
            raise RuntimeError('Buck-E READY is invalid in MoveIt. Stop and inspect collisions/limits.')

        goal = Constraints()
        goal.name = POSE_NAME
        for name, angle in zip(names, ready):
            constraint = JointConstraint()
            constraint.joint_name = name
            constraint.position = angle
            constraint.tolerance_above = 0.01
            constraint.tolerance_below = 0.01
            constraint.weight = 1.0
            goal.joint_constraints.append(constraint)

        OUT.parent.mkdir(parents=True, exist_ok=True)
        rng = random.Random(280)  # Reproducible test positions.
        print('Target READY (degrees):', [round(math.degrees(a), 2) for a in ready])
        print('Planning only: NO physical or simulated execution commands are sent.')
        successes = 0
        trials = 0
        draws = 0
        with OUT.open('w', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(['trial', 'start_degrees', 'result', 'moveit_error_code',
                             'planning_time_seconds', 'trajectory_waypoints',
                             'waypoints_validated', 'max_goal_error_degrees'])
            while trials < TRIALS and draws < MAX_RANDOM_DRAWS:
                draws += 1
                start = [rng.uniform(lo, hi) for lo, hi in limits]
                if not valid(start):
                    continue  # Not a valid random starting configuration.
                trials += 1

                request = GetMotionPlan.Request()
                plan = request.motion_plan_request
                plan.group_name = GROUP
                plan.start_state = make_robot_state(names, start)
                plan.goal_constraints = [goal]
                plan.allowed_planning_time = 4.0
                plan.num_planning_attempts = 3
                plan.max_velocity_scaling_factor = 0.2
                plan.max_acceleration_scaling_factor = 0.2
                response = await_response(node, planning_client, request, timeout=25)
                outcome = response.motion_plan_response
                error = int(outcome.error_code.val)
                trajectory = outcome.trajectory.joint_trajectory
                points = trajectory.points

                result = 'planning_failed'
                validated = 0
                max_error_deg = ''
                if error == MoveItErrorCodes.SUCCESS and points:
                    point_names = trajectory.joint_names
                    final_map = dict(zip(point_names, points[-1].positions))
                    if not all(n in final_map for n in names):
                        result = 'missing_goal_joints'
                    else:
                        max_error_deg = math.degrees(max(abs(final_map[n] - a)
                                                       for n, a in zip(names, ready)))
                        if max_error_deg > 1.0:
                            result = 'goal_mismatch'
                        else:
                            result = 'pass'
                            for point in points:
                                mapping = dict(zip(point_names, point.positions))
                                if not all(n in mapping for n in names) or not valid(
                                        [mapping[n] for n in names]):
                                    result = 'invalid_waypoint'
                                    break
                                validated += 1
                elif error == MoveItErrorCodes.SUCCESS:
                    result = 'empty_trajectory'

                successes += result == 'pass'
                writer.writerow([
                    trials, ' '.join(f'{math.degrees(a):.2f}' for a in start),
                    result, error, round(outcome.planning_time, 3),
                    len(points), validated,
                    '' if max_error_deg == '' else round(max_error_deg, 3)
                ])
                stream.flush()
                print(f'Trial {trials:02}/{TRIALS}: {result} '
                      f'(points={len(points)}, checked={validated})', flush=True)

        print(f'\nRESULT: {successes}/{trials} valid-start trials passed; '
              f'{draws} random candidates sampled.')
        print(f'CSV report: {OUT}')
        if trials < TRIALS:
            print('Note: Not enough valid starting positions found in the draw limit.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        sys.exit(1)
