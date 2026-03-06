#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range

class RoomExplorerNode(Node):
    def __init__(self):
        super().__init__('room_explorer')
        
        # Параметры
        self.declare_parameter('base_speed', 60)
        self.declare_parameter('loop_hz', 10)
        self.declare_parameter('wheel_base', 0.09)  # ДОБАВИТЬ ЭТО
        
        self.speed = self.get_parameter('base_speed').value
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.sensors = {'left': 1.0, 'right': 1.0, 'front': 1.0}
        self.create_subscription(Range, '/sensors/distance', self.sensor_callback, 10)
        
        loop_period = 1.0 / self.get_parameter('loop_hz').value
        self.create_timer(loop_period, self.control_loop)
        
        self.loop_count = 0
        self.get_logger().info('Room Explorer Node запущен')

    def sensor_callback(self, msg):
        frame = msg.header.frame_id
        if '_sensor' in frame:
            position = frame.replace('_sensor', '')
            if position in self.sensors:
                if 0.02 < msg.range < 2.0:
                    self.sensors[position] = msg.range
                else:
                    self.sensors[position] = 1.0

    def get_speeds(self, nearest_distance):
        if nearest_distance >= 1.0:
            return self.speed, self.speed, 100
        elif nearest_distance >= 0.5:
            return self.speed, int(self.speed * 0.8), 100
        elif nearest_distance >= 0.25:
            return self.speed, int(self.speed * 0.6), 100
        elif nearest_distance >= 0.15:
            return -int(self.speed * 0.4), -self.speed, 100
        else:
            return -self.speed, -self.speed, 150

    def control_loop(self):
        left = self.sensors['left']
        right = self.sensors['right']
        front = self.sensors['front']
        
        nearest_side = min(left, right)
        if front < 0.20:
            nearest_side = 0.10
            
        nearest_speed, furthest_speed, delay = self.get_speeds(nearest_side)
        
        if left < right:
            left_cmd = nearest_speed / 100.0
            right_cmd = furthest_speed / 100.0
        else:
            left_cmd = furthest_speed / 100.0
            right_cmd = nearest_speed / 100.0
            
        cmd = Twist()
        cmd.linear.x = (left_cmd + right_cmd) / 2.0
        
        # ИСПРАВЛЕНА ФОРМУЛА ПОВОРОТА (под упрощенный hardware_node)
        cmd.angular.z = (right_cmd - left_cmd) * 0.5 
        
        self.cmd_pub.publish(cmd)
        
        self.loop_count += 1
        if self.loop_count % 10 == 0:
            self.get_logger().info(f'L:{left:.2f} R:{right:.2f} F:{front:.2f} | Cmd: {cmd.linear.x:.2f}, {cmd.angular.z:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = RoomExplorerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
