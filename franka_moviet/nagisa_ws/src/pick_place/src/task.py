#!/usr/bin/env python3  
  
import rclpy  
from rclpy.node import Node  
from moveit_msgs.action import MoveGroup  
from moveit_msgs.msg import (  
    MotionPlanRequest, Constraints, JointConstraint,  
    PositionConstraint, OrientationConstraint  
)  
from geometry_msgs.msg import PoseStamped  
from franka_msgs.action import Grasp, Move  
from rclpy.action import ActionClient  
from shape_msgs.msg import SolidPrimitive  
import time  
  
class PickAndPlaceTask(Node):  
    def __init__(self):  
        super().__init__('pick_and_place_task')  
          
        # 使用正确的命名空间  
        self.move_group_client = ActionClient(self, MoveGroup, '/NS_1/move_action')  
        self.gripper_grasp_client = ActionClient(self, Grasp, '/NS_1/franka_gripper/grasp')  
        self.gripper_move_client = ActionClient(self, Move, '/NS_1/franka_gripper/move')  
        self.get_logger().info("Pick and place task node initialized")  
          
        # 使用 Franka 默认 home 位置的正确姿态（末端执行器朝前）  
        self.home_orientation = [-0.7071, 0.7071, 0.0, 0.0]  # 正确的 home 姿态  
          
        # 等待服务器，但设置超时  
        if not self.move_group_client.wait_for_server(timeout_sec=10.0):  
            self.get_logger().error("MoveGroup action server not available")  
            return  
              
        if not self.gripper_grasp_client.wait_for_server(timeout_sec=10.0):  
            self.get_logger().error("Gripper grasp action server not available")  
            return  
              
        if not self.gripper_move_client.wait_for_server(timeout_sec=10.0):  
            self.get_logger().error("Gripper move action server not available")  
            return  
  
    def create_pose_constraint(self, pose_stamped):  
        """Create proper pose constraint for MoveIt2"""  
        constraints = Constraints()  
          
        # Position constraint  
        position_constraint = PositionConstraint()  
        position_constraint.header = pose_stamped.header  
        position_constraint.link_name = "fr3_hand_tcp"  
        position_constraint.target_point_offset.x = 0.0  
        position_constraint.target_point_offset.y = 0.0  
        position_constraint.target_point_offset.z = 0.0  
          
        # 设置约束区域  
        constraint_region = SolidPrimitive()  
        constraint_region.type = SolidPrimitive.SPHERE  
        constraint_region.dimensions = [0.01]  # 1cm tolerance  
        position_constraint.constraint_region.primitives.append(constraint_region)  
        position_constraint.constraint_region.primitive_poses.append(pose_stamped.pose)  
        position_constraint.weight = 1.0  
          
        # Orientation constraint  
        orientation_constraint = OrientationConstraint()  
        orientation_constraint.header = pose_stamped.header  
        orientation_constraint.link_name = "fr3_hand_tcp"  
        orientation_constraint.orientation = pose_stamped.pose.orientation  
        orientation_constraint.absolute_x_axis_tolerance = 0.1  
        orientation_constraint.absolute_y_axis_tolerance = 0.1  
        orientation_constraint.absolute_z_axis_tolerance = 0.1  
        orientation_constraint.weight = 1.0  
          
        constraints.position_constraints.append(position_constraint)  
        constraints.orientation_constraints.append(orientation_constraint)  
          
        return constraints  
  
    def move_to_pose_smooth(self, pose_list):  
        """Move robot with smoother trajectory parameters"""  
        goal_msg = MoveGroup.Goal()  
        goal_msg.request.group_name = "fr3_arm"  
        goal_msg.request.num_planning_attempts = 15  
        goal_msg.request.allowed_planning_time = 8.0  
        goal_msg.request.max_velocity_scaling_factor = 0.3  # 降低速度  
        goal_msg.request.max_acceleration_scaling_factor = 0.2  # 降低加速度  
        goal_msg.request.planner_id = "RRTstarkConfigDefault"  # 使用 RRT* 获得更优路径  
          
        # 设置目标姿态  
        pose_goal = PoseStamped()  
        pose_goal.header.frame_id = "fr3_link0"  
        pose_goal.header.stamp = self.get_clock().now().to_msg()  
        pose_goal.pose.position.x = pose_list[0]  
        pose_goal.pose.position.y = pose_list[1]  
        pose_goal.pose.position.z = pose_list[2]  
        pose_goal.pose.orientation.x = pose_list[3]  
        pose_goal.pose.orientation.y = pose_list[4]  
        pose_goal.pose.orientation.z = pose_list[5]  
        pose_goal.pose.orientation.w = pose_list[6]  
          
        goal_msg.request.goal_constraints.append(self.create_pose_constraint(pose_goal))  
          
        # 发送目标并等待  
        future = self.move_group_client.send_goal_async(goal_msg)  
        rclpy.spin_until_future_complete(self, future)  
          
        goal_handle = future.result()  
        if not goal_handle.accepted:  
            raise Exception("Move goal rejected")  
          
        result_future = goal_handle.get_result_async()  
        rclpy.spin_until_future_complete(self, result_future)  
          
        result = result_future.result()  
        if result.result.error_code.val != 1:  
            raise Exception(f"Move failed with error code: {result.result.error_code.val}")  
  
    def move_to_pose(self, pose_list):  
        """Move robot to specified pose using MoveIt2"""  
        goal_msg = MoveGroup.Goal()  
        goal_msg.request.group_name = "fr3_arm"  
        goal_msg.request.num_planning_attempts = 10  
        goal_msg.request.allowed_planning_time = 5.0  
          
        # Set target pose  
        pose_goal = PoseStamped()  
        pose_goal.header.frame_id = "fr3_link0"  
        pose_goal.header.stamp = self.get_clock().now().to_msg()  
        pose_goal.pose.position.x = pose_list[0]  
        pose_goal.pose.position.y = pose_list[1]  
        pose_goal.pose.position.z = pose_list[2]  
        pose_goal.pose.orientation.x = pose_list[3]  
        pose_goal.pose.orientation.y = pose_list[4]  
        pose_goal.pose.orientation.z = pose_list[5]  
        pose_goal.pose.orientation.w = pose_list[6]  
          
        goal_msg.request.goal_constraints.append(self.create_pose_constraint(pose_goal))  
          
        # Send goal and wait  
        future = self.move_group_client.send_goal_async(goal_msg)  
        rclpy.spin_until_future_complete(self, future)  
          
        goal_handle = future.result()  
        if not goal_handle.accepted:  
            raise Exception("Move goal rejected")  
          
        result_future = goal_handle.get_result_async()  
        rclpy.spin_until_future_complete(self, result_future)  
          
        result = result_future.result()  
        if result.result.error_code.val != 1:  # SUCCESS = 1  
            raise Exception(f"Move failed with error code: {result.result.error_code.val}")  
  
    def control_gripper_grasp(self, width, force):  
        """Control gripper with grasp action - returns success status instead of raising exception"""  
        try:  
            goal_msg = Grasp.Goal()  
            goal_msg.width = width  
            goal_msg.force = force  
            goal_msg.speed = 0.03  # 更慢的速度  
            goal_msg.epsilon.inner = 0.005  
            goal_msg.epsilon.outer = 0.010  # 增加外部容差  
              
            future = self.gripper_grasp_client.send_goal_async(goal_msg)  
            rclpy.spin_until_future_complete(self, future)  
              
            goal_handle = future.result()  
            if not goal_handle.accepted:  
                self.get_logger().warn("Gripper grasp goal rejected")  
                return False  
                  
            result_future = goal_handle.get_result_async()  
            rclpy.spin_until_future_complete(self, result_future)  
              
            result = result_future.result()  
            if result.result.success:  
                self.get_logger().info("Gripper grasp succeeded")  
                return True  
            else:  
                self.get_logger().warn("Gripper grasp failed - no object detected or grasp unsuccessful")  
                return False  
                  
        except Exception as e:  
            self.get_logger().error(f"Gripper grasp error: {str(e)}")  
            return False  
  
    def control_gripper_move(self, width):  
        """Control gripper with move action - returns success status instead of raising exception"""  
        try:  
            goal_msg = Move.Goal()  
            goal_msg.width = width  
            goal_msg.speed = 0.05  # 更慢的速度  
              
            future = self.gripper_move_client.send_goal_async(goal_msg)  
            rclpy.spin_until_future_complete(self, future)  
              
            goal_handle = future.result()  
            if not goal_handle.accepted:  
                self.get_logger().warn("Gripper move goal rejected")  
                return False  
                  
            result_future = goal_handle.get_result_async()  
            rclpy.spin_until_future_complete(self, result_future)  
              
            result = result_future.result()  
            if result.result.success:  
                self.get_logger().info(f"Gripper moved to {width}m successfully")  
                return True  
            else:  
                self.get_logger().warn(f"Gripper move to {width}m failed")  
                return False  
                  
        except Exception as e:  
            self.get_logger().error(f"Gripper move error: {str(e)}")  
            return False  
  
    def move_to_home(self):  
        """Move robot to home position"""  
        goal_msg = MoveGroup.Goal()  
        goal_msg.request.group_name = "fr3_arm"  
        goal_msg.request.num_planning_attempts = 10  
        goal_msg.request.allowed_planning_time = 5.0  
          
        # Home joint positions  
        joint_names = ['fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4',  
                      'fr3_joint5', 'fr3_joint6', 'fr3_joint7']  
        joint_positions = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]  
          
        constraints = Constraints()  
        for name, position in zip(joint_names, joint_positions):  
            joint_constraint = JointConstraint()  
            joint_constraint.joint_name = name  
            joint_constraint.position = position  
            joint_constraint.tolerance_above = 0.01  
            joint_constraint.tolerance_below = 0.01  
            joint_constraint.weight = 1.0  
            constraints.joint_constraints.append(joint_constraint)  
              
        goal_msg.request.goal_constraints.append(constraints)  
          
        # Send goal and wait  
        future = self.move_group_client.send_goal_async(goal_msg)  
        rclpy.spin_until_future_complete(self, future)  
          
        goal_handle = future.result()  
        if not goal_handle.accepted:  
            raise Exception("Home move goal rejected")  
              
        result_future = goal_handle.get_result_async()  
        rclpy.spin_until_future_complete(self, result_future)  
          
        result = result_future.result()  
        if result.result.error_code.val != 1:  
            raise Exception(f"Home move failed with error code: {result.result.error_code.val}")  
  
    def execute_task(self):  
        """Execute a smooth pick and place sequence with consistent end-effector orientation"""  
        try:  
            self.get_logger().info("Starting smooth pick and place task...")  
              
            # 1. Move to home position first  
            self.get_logger().info("Moving to home position...")  
            self.move_to_home()  
            time.sleep(2.0)  # 增加等待时间  
              
            # 2. Open gripper  
            self.get_logger().info("Opening gripper...")  
            gripper_opened = self.control_gripper_move(0.08)  
            if not gripper_opened:  
                self.get_logger().warn("Failed to open gripper, but continuing...")  
            time.sleep(2.0)  
              
            # 定义统一的操作高度和姿态  
            PICK_PLACE_HEIGHT = 0.2  
            TRANSPORT_HEIGHT = 0.3  # 提高运输高度  
            INTERMEDIATE_HEIGHT = 0.35  # 中间过渡高度  
              
            # 使用与 home 位置一致的姿态（只平移，不旋转）  
            orientation = self.home_orientation  
              
            # 3. 先移动到中间高度位置  
            self.get_logger().info("Moving to intermediate position above pick...")  
            self.move_to_pose_smooth([0.3, 0.0, INTERMEDIATE_HEIGHT] + orientation)  
            time.sleep(1.5)  

            # 4. 缓慢下降到抓取位置  
            self.get_logger().info("Descending to grasp position...")  
            self.move_to_pose_smooth([0.3, 0.0, PICK_PLACE_HEIGHT] + orientation)  
            time.sleep(1.5)  
              
            # 5. 执行抓取  
            self.get_logger().info("Attempting to grasp object...")  
            grasp_successful = self.control_gripper_grasp(0.03, 30.0)  
            if grasp_successful:  
                self.get_logger().info("Object grasped successfully!")  
            else:  
                self.get_logger().info("No object detected - continuing anyway")  
            time.sleep(1.5)  
              
            # 6. 缓慢抬起到运输高度  
            self.get_logger().info("Lifting to transport height...")  
            self.move_to_pose_smooth([0.3, 0.0, TRANSPORT_HEIGHT] + orientation)  
            time.sleep(1.5)  
              
            # 7. 移动到放置区域上方（保持相同姿态）  
            self.get_logger().info("Moving to above place position...")  
            self.move_to_pose_smooth([0.4, 0.4, TRANSPORT_HEIGHT] + orientation)  
            time.sleep(1.5)  
              
            # 8. 缓慢下降到放置位置（与抓取相同高度）  
            self.get_logger().info("Descending to place position...")  
            self.move_to_pose_smooth([0.4, 0.4, PICK_PLACE_HEIGHT] + orientation)  
            time.sleep(1.5)  
              
            # 9. 释放物体  
            self.get_logger().info("Releasing object...")  
            gripper_released = self.control_gripper_move(0.08)  
            if not gripper_released:  
                self.get_logger().warn("Failed to open gripper for release")  
            time.sleep(1.5)  
              
            # 10. 抬起到中间高度  
            self.get_logger().info("Lifting from place position...")  
            self.move_to_pose_smooth([0.4, 0.4, INTERMEDIATE_HEIGHT] + orientation)  
            time.sleep(1.5)  
              
            # 11. 返回 home 位置  
            self.get_logger().info("Returning to home position...")  
            self.move_to_home()  
              
            if grasp_successful:  
                self.get_logger().info("Smooth pick and place completed successfully with consistent orientation!")  
            else:  
                self.get_logger().info("Smooth motion sequence completed (no object grasped)")  
                  
        except Exception as e:  
            self.get_logger().error(f"Task failed: {str(e)}")  
  
def main(args=None):  
    rclpy.init(args=args)  
      
    task_node = PickAndPlaceTask()  
      
    # Wait a bit for everything to initialize  
    time.sleep(3.0)  
      
    # Execute the task  
    task_node.execute_task()  
      
    rclpy.shutdown()  
  
if __name__ == '__main__':  
    main()