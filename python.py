#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

class CmdStampToUnstamped(Node):
    def __init__(self):
        super().__init__('cmd_stamp_to_unstamped_spawner')
        
        # Nome dos tópicos
        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/diff_base_controller/cmd_vel')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        # Assinatura e publicação
        self.subscription = self.create_subscription(
            Twist, input_topic, self.cmd_callback, 10)

        # QoS: RELIABLE + TRANSIENT_LOCAL (keep last, depth=10)
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        # Publisher usa o perfil solicitado (TRANSIENT_LOCAL e RELIABLE)
        self.publisher = self.create_publisher(
            TwistStamped, output_topic, qos_profile)

        self.get_logger().info(
            f"✅ Nó iniciado: convertendo {input_topic} → {output_topic} (QoS: TRANSIENT_LOCAL, RELIABLE)")

    def cmd_callback(self, msg: Twist):
        """Converte Twist → TwistStamped e publica"""
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = 'frame'
        stamped.twist = msg
        self.publisher.publish(stamped)

def main(args=None):
    rclpy.init(args=args)
    node = CmdStampToUnstamped()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
