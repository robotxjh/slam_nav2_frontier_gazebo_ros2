#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import numpy as np

class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')
        
        # 订阅地图
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        
        # Nav2 Action客户端
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )
        
        self.map_data = None
        self.exploring = False
        
        # 定时检测frontier
        self.timer = self.create_timer(3.0, self.explore)
        
        self.get_logger().info('Frontier Explorer 启动!')

    def map_callback(self, msg):
        self.map_data = msg

    def find_frontiers(self):
        if self.map_data is None:
            return []

        width = self.map_data.info.width
        height = self.map_data.info.height
        data = np.array(self.map_data.data).reshape(height, width)
        
        frontiers = []
        
        for y in range(1, height-1):
            for x in range(1, width-1):
                # 找到已知空闲区域(-1是未知,0是空闲,100是障碍)
                if data[y][x] == 0:
                    # 检查邻居有没有未知区域
                    neighbors = [
                        data[y-1][x], data[y+1][x],
                        data[y][x-1], data[y][x+1]
                    ]
                    if -1 in neighbors:
                        # 这是一个frontier点
                        wx = self.map_data.info.origin.position.x + \
                             (x + 0.5) * self.map_data.info.resolution
                        wy = self.map_data.info.origin.position.y + \
                             (y + 0.5) * self.map_data.info.resolution
                        frontiers.append((wx, wy))
        
        return frontiers

    def get_nearest_frontier(self, frontiers):
        if not frontiers:
            return None
        
        # 找最近的frontier(简单用第一个,可以改成距离最近)
        return frontiers[len(frontiers)//2]

    def explore(self):
        if self.exploring:
            return
            
        frontiers = self.find_frontiers()
        
        if not frontiers:
            self.get_logger().info('没有找到frontier,探索完成!')
            return
        
        self.get_logger().info(f'找到 {len(frontiers)} 个frontier点')
        
        target = self.get_nearest_frontier(frontiers)
        if target:
            self.navigate_to(target[0], target[1])

    def navigate_to(self, x, y):
        if not self.nav_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().warn('Nav2 Action服务不可用')
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0

        self.get_logger().info(f'导航到frontier: ({x:.2f}, {y:.2f})')
        self.exploring = True
        
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('导航目标被拒绝')
            self.exploring = False
            return
            
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        self.exploring = False
        self.get_logger().info('到达frontier,继续探索...')

def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()