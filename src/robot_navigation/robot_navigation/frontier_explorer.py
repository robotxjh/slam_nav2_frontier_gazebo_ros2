import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from tf2_ros import Buffer, TransformListener
import numpy as np
import math
from rclpy.time import Time
from rclpy.duration import Duration
from collections import deque

class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')
        
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback, 10
        )
        
        self.nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose'
        )
        
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.map_data = None
        self.map_array = None
        self.exploring = False
        self.failed_frontiers = deque(maxlen=50)  # 只定义一次
        
        # 探索间隔改为5秒，减少CPU压力
        self.timer = self.create_timer(8.0, self.explore)
        self.get_logger().info('Frontier Explorer 启动!')

    def map_callback(self, msg):
        self.map_data = msg
        # 缓存地图数组，避免重复转换
        self.map_array = np.array(
            msg.data, dtype=np.int8
        ).reshape(msg.info.height, msg.info.width)

    def get_robot_position(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_footprint',
                Time(),
                timeout=Duration(seconds=0.5)
            )
            return (transform.transform.translation.x,
                    transform.transform.translation.y)
        except Exception as e:
            self.get_logger().warn(f'获取机器人位置失败: {e}')
            return None, None

    def find_frontiers(self):
        if self.map_data is None or self.map_array is None:
            return []

        # 用numpy向量化运算替代for循环，速度提升10-100倍
        data = self.map_array
        height, width = data.shape

        # 找出所有空闲格子(值为0)
        free = (data == 0)

        # 找出上下左右有未知格子(-1)的空闲格子
        unknown = (data == -1)
        
        # 用numpy滚动检测邻居
        has_unknown_neighbor = (
            np.roll(unknown, 1, axis=0) |   # 上方
            np.roll(unknown, -1, axis=0) |  # 下方
            np.roll(unknown, 1, axis=1) |   # 左方
            np.roll(unknown, -1, axis=1)    # 右方
        )

        # frontier = 空闲 且 有未知邻居
        frontier_mask = free & has_unknown_neighbor

        # 获取frontier的像素坐标
        ys, xs = np.where(frontier_mask)

        if len(xs) == 0:
            return []

        # 转换为世界坐标
        origin_x = self.map_data.info.origin.position.x
        origin_y = self.map_data.info.origin.position.y
        resolution = self.map_data.info.resolution

        # 降采样：每隔5个点取一个，减少frontier数量
        step = 5
        xs = xs[::step]
        ys = ys[::step]

        frontiers = [
            (origin_x + (x + 0.5) * resolution,
             origin_y + (y + 0.5) * resolution)
            for x, y in zip(xs, ys)
        ]

        return frontiers

    def get_nearest_frontier(self, frontiers):
        if not frontiers:
            return None

        robot_x, robot_y = self.get_robot_position()
        if robot_x is None:
            return frontiers[len(frontiers)//2]

        # 过滤失败的frontier
        valid_frontiers = []
        for fx, fy in frontiers:
            too_close = any(
                math.sqrt((fx-fx2)**2 + (fy-fy2)**2) < 0.5
                for fx2, fy2 in self.failed_frontiers
            )
            if not too_close:
                valid_frontiers.append((fx, fy))

        if not valid_frontiers:
            self.get_logger().warn('所有frontier失败过,清空重试')
            self.failed_frontiers.clear()
            valid_frontiers = frontiers

        nearest = min(
            valid_frontiers,
            key=lambda f: math.sqrt(
                (f[0]-robot_x)**2 + (f[1]-robot_y)**2
            )
        )

        dist = math.sqrt(
            (nearest[0]-robot_x)**2 + (nearest[1]-robot_y)**2
        )
        self.get_logger().info(
            f'机器人:({robot_x:.2f},{robot_y:.2f}) '
            f'目标:({nearest[0]:.2f},{nearest[1]:.2f}) '
            f'距离:{dist:.2f}m'
        )

        return nearest

    def explore(self):
        if self.exploring:
            return

        frontiers = self.find_frontiers()

        if not frontiers:
            self.get_logger().info('没有frontier,探索完成!')
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

        self.get_logger().info(f'导航到: ({x:.2f}, {y:.2f})')
        self.exploring = True
        self.current_target = (x, y)

        future = self.nav_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )
        future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        distance = feedback_msg.feedback.distance_remaining
        if distance < 0.3:
            self.get_logger().info(f'接近目标:{distance:.2f}m')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('目标被拒绝')
            if hasattr(self, 'current_target'):
                self.failed_frontiers.append(self.current_target)
            self.exploring = False
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result()
        if result.status == 4:
            self.get_logger().info('到达frontier,继续探索...')
        else:
            self.get_logger().warn(f'导航失败(status:{result.status})')
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