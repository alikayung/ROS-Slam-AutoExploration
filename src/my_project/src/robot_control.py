#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
import tf
import math
import numpy as np
import random
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult
from actionlib_msgs.msg import GoalStatus
from visualization_msgs.msg import Marker
from nav_msgs.srv import GetPlan

# 机器人自主探索类：提取前沿未知点探索未知区域
class AutoExplorer:
    def __init__(self):
        rospy.init_node('auto_explorer_robot', anonymous=False)

        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.latest_map = None # 存储最新栅格地图
        
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("正在连接 move_base...")
        self.client.wait_for_server()
        rospy.loginfo("move_base 已连接！")

        # 初始化 make_plan 服务，用于在导航前预估路径可行性
        rospy.loginfo("正在等待 make_plan 服务...")
        rospy.wait_for_service('/move_base/make_plan')
        self.make_plan = rospy.ServiceProxy('/move_base/make_plan', GetPlan)
        rospy.loginfo("make_plan 服务已就绪！")

        self.tf_listener = tf.TransformListener()
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.marker_pub = rospy.Publisher('/explorer_goal_marker', Marker, queue_size=10)
        
        self.failed_points = [] # 存储导航失败的点，避免重复尝试

    # 更新最新地图
    def map_callback(self, msg):
        self.latest_map = msg
   
    # 获取机器人在map坐标系下的XY坐标和位姿
    def get_robot_pose(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform('/map', 'base_footprint', rospy.Time(0))
            return trans[0], trans[1]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None, None
            
    # 发布目标点标记（Rviz上的绿色球体）
    def publish_goal_marker(self, x, y):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "current_goal"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.2
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.3; marker.scale.y = 0.3; marker.scale.z = 0.3
        marker.color.a = 1.0; marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0
        self.marker_pub.publish(marker)

    # 获取修正后的可行目标点（核心防卡死逻辑）
    def get_feasible_plan_target(self, x, y):
        start_pose = PoseStamped()
        start_pose.header.frame_id = "map"
        start_pose.header.stamp = rospy.Time.now()
        rx, ry = self.get_robot_pose()
        if rx is None: return None
        start_pose.pose.position.x = rx
        start_pose.pose.position.y = ry
        start_pose.pose.orientation.w = 1.0

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = rospy.Time.now()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation.w = 1.0

        try:
            # 容差为 0.2，请求全局规划器计算路径
            plan = self.make_plan(start_pose, goal_pose, 0.2)
            
            if len(plan.plan.poses) == 0:
                return None   # 无规划路径则返回None
            
            # 获取路径终点（如果原目标在墙内，这里会返回路径上离目标最近的有效点）
            valid_pose = plan.plan.poses[-1]
            return (valid_pose.pose.position.x, valid_pose.pose.position.y)

        except rospy.ServiceException as e:
            rospy.logwarn("make_plan 调用失败: %s" % e)
            return None 
             
    # 检查目标点是否在失败黑名单中
    def is_point_blacklisted(self, x, y):
        for fx, fy in self.failed_points:
            dist = math.sqrt((x - fx)**2 + (y - fy)**2)
            if dist < 0.7: 
                return True
        return False
        
    # 原地旋转：更新地图，防止卡死
    def recovery_rotate(self):
        rospy.loginfo("正在原地旋转以更新地图/解除卡死...")
        twist = Twist()
        twist.angular.z = 0.8
        for _ in range(15):
            self.cmd_vel_pub.publish(twist)
            rospy.sleep(0.1)
            
    # 筛选探索前沿点
    def find_best_frontier(self):
        if self.latest_map is None: return None

        width = self.latest_map.info.width
        height = self.latest_map.info.height
        resolution = self.latest_map.info.resolution
        origin_x = self.latest_map.info.origin.position.x
        origin_y = self.latest_map.info.origin.position.y
        
        map_data = np.array(self.latest_map.data).reshape((height, width))
        rx, ry = self.get_robot_pose()
        if rx is None: return None

        potential_goals = []
        step = 5
        
        # 遍历栅格地图寻找前沿点 (未知 -1 且邻域有 已知 0)
        for y in range(step, height - step, step):
            for x in range(step, width - step, step):
                if map_data[y, x] == -1: 
                    neighbors = map_data[y-2:y+3, x-2:x+3]
                    # 过滤噪点，要求周围至少有5个已知点
                    if np.count_nonzero(neighbors == 0) > 5:
                        real_x = origin_x + x * resolution
                        real_y = origin_y + y * resolution
                        dist = math.sqrt((real_x - rx)**2 + (real_y - ry)**2)
                        
                        if dist < 0.5: continue
                        if self.is_point_blacklisted(real_x, real_y): continue
                        potential_goals.append((dist, real_x, real_y))

        if not potential_goals: return None

        potential_goals.sort(key=lambda item: item[0])
        
        # 检查最近的 5 个候选点，防止因为最近点不可达而导致卡死
        check_limit = min(len(potential_goals), 5)
        rospy.loginfo(f"检查 Top {check_limit} 候选点的可达性...")

        for i in range(check_limit):
            candidate = potential_goals[i]
            cx, cy = candidate[1], candidate[2]
            
            # 使用修正后的目标点 (如果点在墙里，会自动修正到墙边)
            valid_target = self.get_feasible_plan_target(cx, cy)
            
            if valid_target:
                vx, vy = valid_target
                rospy.loginfo(f"原始目标:({cx:.2f}, {cy:.2f}) -> 修正为可达目标:({vx:.2f}, {vy:.2f})")
                return vx, vy 
            
        rospy.logwarn("附近的候选点均不可达，尝试旋转刷新...")
        return None
        
    # 发送导航目标并监控执行状态
    def move_to_goal(self, x, y):
        self.publish_goal_marker(x, y)

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1.0

        rospy.loginfo(f"前往目标: ({x:.2f}, {y:.2f})")
        self.client.send_goal(goal)
        
        start_time = rospy.Time.now()
        last_moved_time = rospy.Time.now()
        start_pos = self.get_robot_pose()
        if start_pos is None: start_pos = (0,0)
        last_pos = start_pos

        # 监控移动过程，处理超时或机器人长时间未移动的情况
        while (rospy.Time.now() - start_time).to_sec() < 60.0:
            state = self.client.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("到达！")
                return True
            
            if state in [GoalStatus.ABORTED, GoalStatus.REJECTED]:
                self.failed_points.append((x, y))
                return False

            curr_pos = self.get_robot_pose()
            if curr_pos:
                dist_moved = math.sqrt((curr_pos[0] - last_pos[0])**2 + (curr_pos[1] - last_pos[1])**2)
                if dist_moved > 0.1:
                    last_moved_time = rospy.Time.now()
                    last_pos = curr_pos
            
            # 卡死检测：如果 20秒 内位移不足 0.1m，判定为卡死
            if (rospy.Time.now() - last_moved_time).to_sec() > 20.0:
                rospy.logwarn("卡死超时 (20s)，放弃。")
                self.client.cancel_goal()
                self.failed_points.append((x, y))
                return False

            rospy.sleep(0.2)

        self.client.cancel_goal()
        self.failed_points.append((x, y))
        return False
        
    # 主循环：持续探索未知区域
    def run(self):
        rate = rospy.Rate(3.0)
        self.recovery_rotate() # 初始旋转以构建局部地图

        rospy.loginfo("开始探索...")

        while not rospy.is_shutdown():
            if self.latest_map is None:
                rate.sleep()
                continue

            target = self.find_best_frontier()

            if target:
                tx, ty = target
                success = self.move_to_goal(tx, ty)
                if not success:
                    self.recovery_rotate()
            else:
                rospy.loginfo("当前无可行目标,可能探索已完成。")
                if len(self.failed_points) > 0:
                    rospy.logwarn("所有潜在点都在黑名单中，尝试最后一次旋转更新地图...")
                    
                    self.recovery_rotate()
                rospy.sleep(2.0)
            
            rate.sleep()

if __name__ == "__main__":
    try:
        explorer = AutoExplorer()
        explorer.run()
    except rospy.ROSInterruptException:
        pass
