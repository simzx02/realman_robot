import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from rm_ros_interfaces.msg import Movej, Movec
from geometry_msgs.msg import Point, Quaternion

class MoveArcDemo(Node):
    """
    一个ROS 2节点，用于演示一个单一的MoveC（圆弧）运动。
    该脚本首先使用MoveJ移动到一个预备位置，然后执行一个预定义的圆弧。
    """
    def __init__(self):
        super().__init__('move_arc_demo_node')

        # --- 状态机管理变量 ---
        # 管理运动的顺序: MoveJ -> MoveC
        self.state = 'INITIAL_MOVE'
        self.move_completed = True

        # --- 发布器 ---
        self.movej_pub = self.create_publisher(Movej, '/rm_driver/movej_cmd', 10)
        self.movec_pub = self.create_publisher(Movec, '/rm_driver/movec_cmd', 10)

        # --- 订阅器 ---
        self.movej_sub = self.create_subscription(Bool, '/rm_driver/movej_result', self.movej_result_callback, 10)
        self.movec_sub = self.create_subscription(Bool, '/rm_driver/movec_result', self.movec_result_callback, 10)

        # --- 定时器，用于驱动状态机 ---
        self.timer = self.create_timer(0.2, self.state_machine_callback)
        self.get_logger().info('节点已启动。圆弧运动序列即将开始。')

    def state_machine_callback(self):
        """主逻辑循环，根据当前状态执行。"""
        if not self.move_completed:
            return  # 等待上一个指令完成

        # --- 状态 1: 移动到安全的预备位置 ---
        if self.state == 'INITIAL_MOVE':
            self.get_logger().info('状态 1: 正在移动到安全的预备关节位置...')
            # 使用您C++脚本中为7轴臂定义的已知安全姿态
            movej_msg = Movej()
            movej_msg.joint = [0.176278, 0.0, 0.3543, 0.53, 0.00873, 0.3595, 0.3595]
            movej_msg.speed = 20  # 使用一个适中的速度
            movej_msg.dof = 7
            movej_msg.block = True
            
            self.movej_pub.publish(movej_msg)
            self.move_completed = False
            self.state = 'WAITING_FOR_INITIAL_MOVE'

        # --- 状态 2: 绘制单个半圆形圆弧 ---
        elif self.state == 'ARC_MOVE':
            self.get_logger().info("状态 2: 发送MoveC（圆弧）指令...")
            
            self.send_movec_command()
            self.move_completed = False
            self.state = 'WAITING_FOR_ARC'

        # --- 状态 3: 完成 ---
        elif self.state == 'DONE':
            self.get_logger().info('圆弧运动序列完成！')
            self.timer.cancel()
            self.create_timer(1.0, self.shutdown_node) # 延迟1秒后关闭节点
            self.state = 'FINISHED' # 防止重复进入

    def movej_result_callback(self, msg: Bool):
        """处理初始MoveJ指令的结果。"""
        if self.state != 'WAITING_FOR_INITIAL_MOVE': return
        
        if msg.data:
            self.get_logger().info('初始移动成功，准备开始圆弧运动。')
            self.move_completed = True
            self.state = 'ARC_MOVE'
        else:
            self.get_logger().error('初始移动失败！终止序列。')
            self.timer.cancel()

    def movec_result_callback(self, msg: Bool):
        """处理MoveC指令的结果。"""
        if self.state != 'WAITING_FOR_ARC': return

        if msg.data:
            self.get_logger().info('圆弧运动成功。')
            self.move_completed = True
            self.state = 'DONE'
        else:
            self.get_logger().error('MoveC 失败！终止序列。')
            self.timer.cancel()
            
    def send_movec_command(self):
        """辅助函数，用于创建和发布MoveC消息。"""
        movec_msg = Movec()
        
        # 圆弧中间点
        movec_msg.pose_mid.position = Point(x=-0.307239, y=0.150903, z=0.222814)
        movec_msg.pose_mid.orientation = Quaternion(x=0.995179, y=-0.094604, z=-0.025721, w=0.002349)
        
        # 圆弧结束点
        movec_msg.pose_end.position = Point(x=-0.357239, y=0.000903, z=0.222814)
        movec_msg.pose_end.orientation = Quaternion(x=0.995179, y=-0.094604, z=-0.025721, w=0.002349)
        
        movec_msg.speed = 15 # 为圆弧运动使用一个较慢的速度以确保稳定
        movec_msg.loop = 0   # 值为0表示不循环，只执行一次
        movec_msg.block = True
        
        self.movec_pub.publish(movec_msg)

    def shutdown_node(self):
        """干净地关闭节点。"""
        self.get_logger().info('正在关闭节点。')
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    move_arc_node = MoveArcDemo()
    try:
        rclpy.spin(move_arc_node)
    except KeyboardInterrupt:
        # 允许通过 Ctrl+C 手动关闭
        pass
    finally:
        if rclpy.ok():
            move_arc_node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main() 