import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
import RPi.GPIO as GPIO
import atexit
import time

from Raspi_MotorHAT import Raspi_MotorHAT
from my_robot.hardware.encoder_counter import EncoderCounter
from my_robot.hardware.servos import Servos

class HardwareNode(Node):
    def __init__(self):
        super().__init__('hardware_node')
        
        # Параметры
        self.declare_parameter('motorhat_addr', 0x6F)
        self.declare_parameter('wheel_base', 0.09)  # 9 см между колесами
        
        # Инициализация железа (как в вашем robot.py)
        self.init_robot()

        # Таймер проверки энкодеров
        self.encoder_test_timer = self.create_timer(0.5, self.test_encoders) 
        
        # Подписка на команды скорости
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_callback, 10)
        
        # Публикация данных с датчиков (10 Hz)
        self.dist_pub = self.create_publisher(Range, '/sensors/distance', 10)

        # Таймер для публикации датчиков 
        self.sensor_timer = self.create_timer(0.1, self.publish_sensors)

        # Таймер heartbeat
        self.heartbeat = self.create_timer(1.0, self.heartbeat_callback)
        
        # Регистрация остановки
        atexit.register(self.stop_all)
        
        self.get_logger().info('Hardware Node с моторами и датчиками запущен!')
    
    def init_robot(self):

        """Инициализация всего железа"""
        addr = self.get_parameter('motorhat_addr').value
        
        # Motor HAT
        self._mh = Raspi_MotorHAT(addr=addr)
        self.left_motor = self._mh.getMotor(1)
        self.right_motor = self._mh.getMotor(2)
        
        GPIO.setwarnings(False)      # 1. Отключить предупреждения
        GPIO.cleanup()               # 2. Сбросить старые настройки 
        GPIO.setmode(GPIO.BCM)       # 3. Задать режим ДО настройки пинов
        
        # Пины датчиков
        self.trig_left = 27
        self.echo_left = 17
        self.trig_right = 6
        self.echo_right = 5
        self.trig_front = 23
        self.echo_front = 24
        
        # 4. Сначала настрой TRIG как OUTPUT
        for pin in [self.trig_left, self.trig_right, self.trig_front]:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)
        
        # 5. Потом настрой ECHO как INPUT
        for pin in [self.echo_left, self.echo_right, self.echo_front]:
            GPIO.setup(pin, GPIO.IN)
        
        time.sleep(0.1) # Стабилизация
        # =======================================
        
        # === Энкодеры ===
        
        try:
            GPIO.remove_event_detect(12)
        except:
            pass
        try:
            GPIO.remove_event_detect(26)
        except:
            pass
        
        time.sleep(0.2)  # Пауза для освобождения пинов


        self.left_encoder = EncoderCounter(12)
        self.right_encoder = EncoderCounter(26)

        self.left_ticks = 0
        self.right_ticks = 0

        # self.servos = Servos(addr=addr)
        
        self.current_left_speed = 0
        self.current_right_speed = 0
    
    def get_distance(self, trig_pin, echo_pin):
        """Измерение дистанции методом опроса (без прерываний)"""
        try:
            GPIO.output(trig_pin, True)
            time.sleep(0.00001)
            GPIO.output(trig_pin, False)
            
            pulse_start = time.time()
            timeout = pulse_start + 0.1
            
            while GPIO.input(echo_pin) == 0:
                if time.time() > timeout: return float('inf')
                pulse_start = time.time()
                
            pulse_end = time.time()
            while GPIO.input(echo_pin) == 1:
                if time.time() > timeout: return float('inf')
                pulse_end = time.time()
                
            duration = pulse_end - pulse_start
            distance = duration * 17150 # см
            
            if distance > 200 or distance < 2: return float('inf')
            return distance / 100.0 # метры
        except:
            return float('inf')

    def convert_speed(self, speed_percent):
        """Из вашего robot.py"""
        mode = Raspi_MotorHAT.RELEASE
        if speed_percent > 0:
            mode = Raspi_MotorHAT.FORWARD
        elif speed_percent < 0:
            mode = Raspi_MotorHAT.BACKWARD
            speed_percent = -speed_percent
        
        output_speed = int((speed_percent * 255) // 100)
        output_speed = min(255, max(0, output_speed))
        return mode, output_speed
    
    def cmd_callback(self, msg):
        self.get_logger().info(f'Получена команда: linear.x={msg.linear.x}, angular.z={msg.angular.z}')
        """Обработка команд скорости"""
        wheel_base = self.get_parameter('wheel_base').value
        
        # Дифференциальная модель для узкой колесной базы
        left = msg.linear.x - msg.angular.z 
        right = msg.linear.x + msg.angular.z
        
        # Конвертация в проценты (0.6 скорость = 60%)
        left_percent = left * 100
        right_percent = right * 100
        
        # Применяем к моторам (как в robot.py)
        mode_l, speed_l = self.convert_speed(left_percent)
        mode_r, speed_r = self.convert_speed(right_percent)
        
        self.left_motor.setSpeed(speed_l)
        self.left_motor.run(mode_l)
        self.right_motor.setSpeed(speed_r)
        self.right_motor.run(mode_r)
        
        self.current_left_speed = left_percent
        self.current_right_speed = right_percent
        
        self.get_logger().debug(f'L:{left_percent:.0f}% R:{right_percent:.0f}%')
    
    def publish_sensors(self):
        try:
            left = self.get_distance(self.trig_left, self.echo_left)
            time.sleep(0.05)
            right = self.get_distance(self.trig_right, self.echo_right)
            time.sleep(0.05)
            front = self.get_distance(self.trig_front, self.echo_front)
            
            self.publish_sensor('left', left)
            self.publish_sensor('right', right)
            self.publish_sensor('front', front)
        except Exception as e:
            self.get_logger().error(f'Ошибка датчиков: {e}')

    def publish_sensor(self, position, distance):
        """Вспомогательный метод для публикации одного датчика"""
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f'{position}_sensor'
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 0.26  # ~15 градусов для HC-SR04
        msg.min_range = 0.02
        msg.max_range = 2.0
        msg.range = distance if distance != float('inf') else 2.0
        
        self.dist_pub.publish(msg)
    
    def heartbeat_callback(self):
        self.get_logger().info('Hardware Node работает', throttle_duration_sec=5)
    
    def test_encoders(self):
        """Простой вывод счёта энкодеров в консоль"""
        if hasattr(self, 'left_encoder') and hasattr(self, 'right_encoder'):
            left = self.left_encoder.get_count()
            right = self.right_encoder.get_count()
            if left != self.left_ticks or right != self.right_ticks:
                self.get_logger().info(f'Encoders: L={left}, R={right}')
                self.left_ticks = left
                self.right_ticks = right

    def stop_all(self):
        """Остановка всех устройств"""
        self.left_motor.run(Raspi_MotorHAT.RELEASE)
        self.right_motor.run(Raspi_MotorHAT.RELEASE)
        
        # Остановка энкодеров (освобождение пинов)
        if hasattr(self, 'left_encoder'):
            self.left_encoder.stop()
        if hasattr(self, 'right_encoder'):
            self.right_encoder.stop()
            
        GPIO.cleanup()
        self.get_logger().info('Все моторы остановлены')


def main(args=None):
    rclpy.init(args=args)
    node = HardwareNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Получен сигнал остановки')
    finally:
        node.stop_all()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
