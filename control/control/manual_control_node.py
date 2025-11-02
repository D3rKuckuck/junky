import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Twist

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        
        # Переменные для хранения состояния
        self.key_states = {
            'w': 0, 'a': 0, 's': 0, 'd': 0
        }
        self.speed_level = 0  # Текущий уровень скорости (0-255)
        self.joystick_left_y = 0
        self.joystick_right_x = 0
        self.control_mode = -1  # -1: нет управления, 0: ПК, 1: телефон
        
        # Подписчики
        self.key_sub = self.create_subscription(
            String, '/junky/keyboard_control', self.key_callback, 10)
        self.speed_sub = self.create_subscription(
            Int32, '/junky/speed_level', self.speed_callback, 10)
        self.joy_sub = self.create_subscription(
            Twist, '/junky/joystick_control', self.joy_callback, 10)
        self.mode_sub = self.create_subscription(
            Int32, '/junky/control_mode', self.mode_callback, 10)
        
        self.get_logger().info('Control node started')

    def key_callback(self, msg):
        try:
            key, state = msg.data.split(':')
            if key in self.key_states:
                self.key_states[key] = int(state)
                self.get_logger().info(f'Key {key}: {state}')
                self.process_control()
        except ValueError:
            pass

    def speed_callback(self, msg):
        self.speed_level = msg.data
        self.get_logger().info(f'Speed level: {self.speed_level}')
        self.process_control()

    def joy_callback(self, msg):
        self.joystick_left_y = int(msg.linear.x * 100)  # Денормализация
        self.joystick_right_x = int(msg.angular.z * 100)  # Денормализация
        self.get_logger().info(f'Joystick: LY={self.joystick_left_y}, RX={self.joystick_right_x}')
        self.process_control()

    def mode_callback(self, msg):
        self.control_mode = msg.data
        self.get_logger().info(f'Control mode: {self.control_mode}')

    def process_control(self):
        # Здесь будет логика преобразования команд в управление моторами
        if self.control_mode == 0:  # Управление с ПК
            # Обработка клавиатурного управления
            # speed_level уже содержит установленную скорость (0-255)
            pass
        elif self.control_mode == 1:  # Управление с телефона
            # Обработка джойстиков
            pass
        
        # Логирование текущего состояния
        self.get_logger().info(
            f'Mode: {self.control_mode}, '
            f'Keys: {self.key_states}, '
            f'Speed: {self.speed_level}, '
            f'Joystick: LY={self.joystick_left_y}, RX={self.joystick_right_x}'
        )

def main():
    rclpy.init()
    node = ControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()