import rclpy
import argparse
from rclpy.node import Node
from rclpy.action.client import ActionClient
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from builtin_interfaces.msg import Duration

def main():
    # Parser to read arguments from the terminal
    parser = argparse.ArgumentParser(
        prog='gripper',
        description='Node that executes FollowJointTrajectory action to move the finger_joint'
    )
    parser.add_argument('pos', default='-0.08')
    parser.print_help
    args = parser.parse_args()
    # ROS node code
    rclpy.init()
    node = Node("gripper_controller")
    logger = node.get_logger()
    move_to_position = float(args.pos)

    ac = ActionClient(node, FollowJointTrajectory, "/gripper_controller/follow_joint_trajectory")
    ac.wait_for_server()

    logger.info("connected to server")
    traj = JointTrajectory()
    traj.joint_names = ["finger_joint"]
    traj.points = [JointTrajectoryPoint(positions=[move_to_position])]
    
    goal = FollowJointTrajectory.Goal()
    goal.trajectory = traj
    goal.goal_time_tolerance = Duration(sec=1)
    
    tol = [JointTolerance(name=n, position=0.001, velocity=0.001) for n in traj.joint_names]
    
    goal.path_tolerance = tol
    goal.goal_tolerance = tol
    
    res = ac.send_goal_async(goal)
    logger.info("waiting for goal complete")
    rclpy.spin_until_future_complete(node, res)