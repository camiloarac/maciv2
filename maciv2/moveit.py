import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from threading import Thread
from pymoveit2 import MoveIt2

home = [0., 0., 0., 0., 0., 0.]
pos_grab = [0., -1.12, 1.57, 0.92, 0., -1.57]
gripper_open = [0.0]
gripper_closed = [-0.08]

def main():
    rclpy.init()
    node = rclpy.create_node(node_name="maci_moveit_controller")
    logger = node.get_logger()
    node_finger = rclpy.create_node(node_name="gripper_moveit_controller")
    logger_finger = node.get_logger()
    
    callback_group = ReentrantCallbackGroup()
    callback_group_finger = ReentrantCallbackGroup() 

    moveit2 = MoveIt2(node=node, 
                      joint_names=['ur5_shoulder_pan_joint', 'ur5_shoulder_lift_joint',
                                   'ur5_elbow_joint','ur5_wrist_1_joint',
                                   'ur5_wrist_2_joint','ur5_wrist_3_joint'],
                      base_link_name='ur5_base_link',
                      end_effector_name='gripper',
                      group_name='arm',
                      callback_group=callback_group)
    moveit2_finger = MoveIt2(node=node_finger, 
                      joint_names=['finger_joint'],
                      base_link_name='ur5_tool0',
                      end_effector_name='gripper',
                      group_name='gripper',
                      callback_group=callback_group_finger)

    # Spin the node in background thread(s) and wait a bit for initialization
    executor = rclpy.executors.MultiThreadedExecutor(3)
    executor.add_node(node)
    executor.add_node(node_finger)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    node.create_rate(1.0).sleep()


    logger.info("Move to grabbing position")
    moveit2.move_to_configuration(pos_grab)
    moveit2.wait_until_executed()

    logger_finger.info("Close gripper")
    moveit2_finger.move_to_configuration(gripper_closed)
    moveit2_finger.wait_until_executed()

    logger.info("Move To Home Position")
    moveit2.move_to_configuration(home)
    moveit2.wait_until_executed()

    logger_finger.info("Open gripper")
    moveit2_finger.move_to_configuration(gripper_open)
    moveit2_finger.wait_until_executed()

    rclpy.shutdown()
    executor_thread.join()