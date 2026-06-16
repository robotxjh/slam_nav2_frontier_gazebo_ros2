import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from tf2_ros import Buffer, TransformListener
import numpy as np
import math
from rclpy.time import Time
from rclpy.duration import Duration

class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')
        
        self.map_sub = self.create_subscription(
            OccupancyGrid, 
            '/map',
              self.map_callback, 10
        )
        
        self.nav_client = ActionClient(
            self, 
            NavigateToPose, 
            'navigate_to_pose'
        )
        
        # 用TF获取机器人当前位置
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.map_data = None
        self.exploring = False
        self.failed_frontiers = []  # 记录失败的frontier,避免重复尝试
        
        self.timer = self.create_timer(3.0, self.explore)
        self.get_logger().info('Frontier Explorer 启动!')

    def map_callback(self, msg):
        self.map_data = msg

    def get_robot_position(self):
        """获取机器人在地图坐标系中的当前位置"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_footprint',
                Time(),
                timeout=Duration(seconds=1.0)
            )
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            return x, y
        except Exception as e:
            self.get_logger().warn(f'获取机器人位置失败: {e}')
            return None,None

    def find_frontiers(self):
        if self.map_data is None:
            return []

        width = self.map_data.info.width
        height = self.map_data.info.height
        data = np.array(self.map_data.data).reshape(height, width)
        
        frontiers = []
        
        for y in range(1, height-1):
            for x in range(1, width-1):
                if data[y][x] == 0:
                    neighbors = [
                        data[y-1][x], data[y+1][x],
                        data[y][x-1], data[y][x+1]
                    ]
                    if -1 in neighbors:
                        wx = self.map_data.info.origin.position.x + \
                             (x + 0.5) * self.map_data.info.resolution
                        wy = self.map_data.info.origin.position.y + \
                             (y + 0.5) * self.map_data.info.resolution
                        frontiers.append((wx, wy))
        
        return frontiers

    def get_nearest_frontier(self, frontiers):
        """选择离机器人最近的frontier"""
        if not frontiers:
            return None
        
        robot_x, robot_y = self.get_robot_position()
        if robot_x is None:
            # 获取不到位置时,用地图中心
            return frontiers[len(frontiers)//2]
        
        # 过滤掉之前失败的frontier(距离太近的也过滤)
        valid_frontiers = []
        for fx, fy in frontiers:
            too_close_to_failed = False
            for fx2, fy2 in self.failed_frontiers:
                if math.sqrt((fx-fx2)**2 + (fy-fy2)**2) < 0.5:
                    too_close_to_failed = True
                    break
            if not too_close_to_failed:
                valid_frontiers.append((fx, fy))
        
        if not valid_frontiers:
            self.get_logger().warn('所有frontier都曾失败过,清空失败记录重试')
            self.failed_frontiers = []
            valid_frontiers = frontiers
        
        # 计算到每个frontier的距离,选最近的
        nearest = min(
            valid_frontiers,
            key=lambda f: math.sqrt(
                (f[0] - robot_x)**2 + (f[1] - robot_y)**2
            )
        )
        
        self.get_logger().info(
            f'机器人位置:({robot_x:.2f},{robot_y:.2f}) '
            f'选择frontier:({nearest[0]:.2f},{nearest[1]:.2f}) '
            f'距离:{math.sqrt((nearest[0]-robot_x)**2+(nearest[1]-robot_y)**2):.2f}m'
        )
        
        return nearest

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
        self.current_target = (x, y)
        
        future = self.nav_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )
        future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        """监控导航进度"""
        feedback = feedback_msg.feedback
        distance = feedback.distance_remaining
        if distance < 0.3:
            self.get_logger().info(f'接近目标,剩余距离:{distance:.2f}m')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('导航目标被拒绝,记录失败frontier')
            if hasattr(self, 'current_target'):
                self.failed_frontiers.append(self.current_target)
            self.exploring = False
            return
            
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result()
        if result.status == 4:  # SUCCEEDED
            self.get_logger().info('成功到达frontier,继续探索...')
        else:
            self.get_logger().warn(f'导航失败(status:{result.status}),记录失败frontier')
            if hasattr(self, 'current_target'):
                self.failed_frontiers.append(self.current_target)
        self.exploring = False

def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()