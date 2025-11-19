#!/usr/bin/env python3
"""
全向轮路径跟踪节点

功能：
- 订阅path、odom_world和kfs_data，使用PID控制跟踪路径
- 当距离target 0.7m时，检查target位置的kfs_data
- 如果kfs_data不为0，立即停止（cmd_vel=0）
- 持续检查直到kfs_data转为0，然后继续移动
- 全向轮可以直接控制x、y方向的线速度
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
import math
import numpy as np
import json


class OmnidirectionalTrackerNode(Node):
    """全向轮路径跟踪节点"""
    
    def __init__(self):
        super().__init__('omnidirectional_tracker')
        
        # 参数（直接赋值）
        self.kp_linear = 5.0
        self.ki_linear = 0.0
        self.kd_linear = 0.1
        self.kp_angular = 10.0
        self.ki_angular = 0.0
        self.kd_angular = 0.1
        self.max_linear_vel = 0.5
        self.max_angular_vel = 5.0
        self.control_frequency = 50.0
        self.target_distance_threshold = 0.1
        self.kfs_check_distance = 0.7  # 检查KFS的距离阈值
        
        # 网格配置（与astar_planner一致）
        self.grid_rows = 4
        self.grid_cols = 3
        self.map2_origin = [3.2, 1.2, 0.0]  # [x, y, theta]
        self.grid_resolution = 1.2  # 米/单元格
        
        # 状态
        self.have_path = False
        self.have_odom = False
        self.have_kfs_data = False
        self.current_path = None
        self.current_path_index = 0
        
        # KFS网格数据
        self.kfs_grid = np.zeros((self.grid_rows, self.grid_cols), dtype=int)
        
        # 等待KFS标志
        self.is_waiting_for_kfs = False
        self.waiting_target = None  # 等待的目标点
        
        # 当前位姿
        self.current_pos = np.array([0.0, 0.0, 0.0])  # [x, y, z]
        self.current_yaw = 0.0
        
        # PID控制状态
        self.integral_error_x = 0.0
        self.integral_error_y = 0.0
        self.integral_error_yaw = 0.0
        self.last_error_x = 0.0
        self.last_error_y = 0.0
        self.last_error_yaw = 0.0
        
        # 当前速度（用于平滑减速）
        self.current_vel_x = 0.0
        self.current_vel_y = 0.0
        self.current_vel_yaw = 0.0
        
        # 创建订阅者（使用TRANSIENT_LOCAL QoS）
        path_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.path_sub = self.create_subscription(
            Path,
            '/planning/path',
            self.path_callback,
            path_qos
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom_world',
            self.odom_callback,
            10
        )
        
        # 订阅KFS数据
        kfs_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.kfs_data_sub = self.create_subscription(
            String,
            '/kfs_grid_data',
            self.kfs_data_callback,
            kfs_qos
        )
        
        # 创建发布者
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # 创建控制定时器
        control_period = 1.0 / self.control_frequency
        self.control_timer = self.create_timer(control_period, self.control_callback)
        
        self.get_logger().info('Omnidirectional Tracker Node started')
        self.get_logger().info(f'Max linear vel: {self.max_linear_vel}m/s')
        self.get_logger().info(f'Max angular vel: {self.max_angular_vel}rad/s')
        self.get_logger().info(f'KFS check distance: {self.kfs_check_distance}m')
    
    def path_callback(self, msg):
        """处理路径消息"""
        if len(msg.poses) == 0:
            self.get_logger().warn('Received empty path')
            return
        
        self.current_path = msg
        self.have_path = True
        
        # 重置等待状态
        self.is_waiting_for_kfs = False
        self.waiting_target = None
        
        # 检查是否已经在第一个点的grid上
        if self.have_odom and len(msg.poses) > 0:
            first_pose = msg.poses[0].pose
            first_point = np.array([first_pose.position.x, first_pose.position.y])
            
            # 将第一个点和当前位置转换为grid坐标
            first_row, first_col = self.map_to_grid_coords(first_point[0], first_point[1])
            current_row, current_col = self.map_to_grid_coords(self.current_pos[0], self.current_pos[1])
            
            # 如果已经在第一个点的grid上，跳过第一个点（如果路径有多个点）
            if current_row == first_row and current_col == first_col and len(msg.poses) > 1:
                self.current_path_index = 1
                self.get_logger().info(f'Already at first path point grid [{current_row}, {current_col}], starting from index 1')
            else:
                self.current_path_index = 0
                if current_row == first_row and current_col == first_col:
                    self.get_logger().info(f'Already at first path point grid [{current_row}, {current_col}], but path has only 1 point')
                else:
                    self.get_logger().info(f'Starting from first path point, current grid [{current_row}, {current_col}], first point grid [{first_row}, {first_col}]')
        else:
            self.current_path_index = 0
        
        self.get_logger().info(f'Received new path with {len(msg.poses)} points, starting from index {self.current_path_index}')
    
    def odom_callback(self, msg):
        """处理里程计消息"""
        self.current_pos[0] = msg.pose.pose.position.x
        self.current_pos[1] = msg.pose.pose.position.y
        self.current_pos[2] = msg.pose.pose.position.z
        
        # 从四元数提取yaw角
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        self.current_yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        
        self.have_odom = True
    
    def kfs_data_callback(self, msg):
        """处理kfs_grid_data消息"""
        try:
            grid_data = json.loads(msg.data)
            grid = np.array(grid_data['grid'])
            
            if grid.shape != (self.grid_rows, self.grid_cols):
                self.get_logger().warn(f'Invalid grid dimensions: {grid.shape}')
                return
            
            self.kfs_grid = grid
            self.have_kfs_data = True
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse JSON: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing kfs_data: {e}')
    
    def map_to_grid_coords(self, x, y):
        """
        将map坐标转换为grid索引
        
        Args:
            x, y: map坐标
            
        Returns:
            tuple: (row, col) grid索引
        """
        row = int((x - self.map2_origin[0]) / self.grid_resolution)
        col = int((y - self.map2_origin[1]) / self.grid_resolution)
        return row, col
    
    def get_target_kfs_value(self, target):
        """
        获取目标点位置的KFS值
        
        Args:
            target: [x, y, z] 目标点坐标
            
        Returns:
            int: KFS值，如果坐标无效返回-1
        """
        if not self.have_kfs_data:
            return -1
        
        row, col = self.map_to_grid_coords(target[0], target[1])
        
        # 检查grid坐标是否有效
        if row < 0 or row >= self.grid_rows or col < 0 or col >= self.grid_cols:
            return -1
        
        return self.kfs_grid[row, col]
    
    def get_next_target(self):
        """获取下一个目标点"""
        if not self.have_path or self.current_path is None:
            return None
        
        if self.current_path_index >= len(self.current_path.poses):
            return None
        
        pose = self.current_path.poses[self.current_path_index].pose
        target = np.array([pose.position.x, pose.position.y, pose.position.z])
        
        # 检查是否到达当前目标点
        dist = np.linalg.norm(target[:2] - self.current_pos[:2])
        if dist < self.target_distance_threshold:
            # 到达目标点，移动到下一个点
            self.current_path_index += 1
            if self.current_path_index >= len(self.current_path.poses):
                # 路径完成
                return None
            pose = self.current_path.poses[self.current_path_index].pose
            target = np.array([pose.position.x, pose.position.y, pose.position.z])
        
        return target
    
    def compute_pid_control(self, target):
        """计算PID控制指令"""
        if target is None:
            return 0.0, 0.0, 0.0
        
        # 计算位置误差（世界坐标系）
        error_x = target[0] - self.current_pos[0]
        error_y = target[1] - self.current_pos[1]
        
        # 计算期望航向角（朝向目标点）
        desired_yaw = math.atan2(error_y, error_x)
        error_yaw = desired_yaw - self.current_yaw
        
        # 归一化角度误差到[-pi, pi]
        while error_yaw > math.pi:
            error_yaw -= 2.0 * math.pi
        while error_yaw < -math.pi:
            error_yaw += 2.0 * math.pi
        
        # PID控制
        dt = 1.0 / self.control_frequency
        
        # X方向PID
        self.integral_error_x += error_x * dt
        derivative_error_x = (error_x - self.last_error_x) / dt
        vel_x = self.kp_linear * error_x + self.ki_linear * self.integral_error_x + self.kd_linear * derivative_error_x
        
        # Y方向PID
        self.integral_error_y += error_y * dt
        derivative_error_y = (error_y - self.last_error_y) / dt
        vel_y = self.kp_linear * error_y + self.ki_linear * self.integral_error_y + self.kd_linear * derivative_error_y
        
        # 航向角PID
        self.integral_error_yaw += error_yaw * dt
        derivative_error_yaw = (error_yaw - self.last_error_yaw) / dt
        vel_yaw = self.kp_angular * error_yaw + self.ki_angular * self.integral_error_yaw + self.kd_angular * derivative_error_yaw
        
        # 更新上次误差
        self.last_error_x = error_x
        self.last_error_y = error_y
        self.last_error_yaw = error_yaw
        
        # 限制积分饱和
        self.integral_error_x = max(-1.0, min(1.0, self.integral_error_x))
        self.integral_error_y = max(-1.0, min(1.0, self.integral_error_y))
        self.integral_error_yaw = max(-1.0, min(1.0, self.integral_error_yaw))
        
        return vel_x, vel_y, vel_yaw
    
    def control_callback(self):
        """控制回调函数"""
        if not self.have_path or not self.have_odom:
            return
        
        if self.current_path is None or len(self.current_path.poses) == 0:
            return
        
        # 获取目标点
        target = self.get_next_target()
        
        if target is None:
            # 路径完成，停止
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)
            self.have_path = False
            self.is_waiting_for_kfs = False
            self.waiting_target = None
            self.current_vel_x = 0.0
            self.current_vel_y = 0.0
            self.current_vel_yaw = 0.0
            self.get_logger().info('Path completed, stopping')
            return
        
        # 计算到目标点的距离
        dist_to_target = np.linalg.norm(target[:2] - self.current_pos[:2])
        
        # 如果正在等待KFS变为0
        if self.is_waiting_for_kfs:
            # 检查当前目标点的KFS值
            kfs_value = self.get_target_kfs_value(self.waiting_target)
            
            if kfs_value == 0:
                # KFS已变为0，可以继续移动
                self.is_waiting_for_kfs = False
                self.waiting_target = None
                self.get_logger().info('Target KFS cleared, resuming movement')
            else:
                # 仍然等待，保持停止
                cmd = Twist()
                cmd.linear.x = 0.0
                cmd.linear.y = 0.0
                cmd.angular.z = 0.0
                self.cmd_vel_pub.publish(cmd)
                return
        
        # 如果距离目标点小于0.7m，检查KFS
        if dist_to_target < self.kfs_check_distance:
            kfs_value = self.get_target_kfs_value(target)
            
            if kfs_value > 0:
                # KFS不为0，立即停止并等待
                self.is_waiting_for_kfs = True
                self.waiting_target = target.copy()
                cmd = Twist()
                cmd.linear.x = 0.0
                cmd.linear.y = 0.0
                cmd.angular.z = 0.0
                self.cmd_vel_pub.publish(cmd)
                self.get_logger().info(f'Target KFS={kfs_value} detected at distance {dist_to_target:.2f}m, stopping and waiting')
                return
        
        # 正常控制：计算PID控制指令
        vel_x, vel_y, vel_yaw = self.compute_pid_control(target)
        
        # 限幅
        vel_x = max(-self.max_linear_vel, min(self.max_linear_vel, vel_x))
        vel_y = max(-self.max_linear_vel, min(self.max_linear_vel, vel_y))
        vel_yaw = max(-self.max_angular_vel, min(self.max_angular_vel, vel_yaw))
        
        # 更新当前速度
        self.current_vel_x = vel_x
        self.current_vel_y = vel_y
        self.current_vel_yaw = vel_yaw
        
        # 发布cmd_vel
        cmd = Twist()
        cmd.linear.x = float(self.current_vel_x)
        cmd.linear.y = float(self.current_vel_y)
        cmd.angular.z = float(self.current_vel_yaw)
        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = OmnidirectionalTrackerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

