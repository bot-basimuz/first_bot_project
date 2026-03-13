#!/usr/bin/env python3
"""
Hardware Node для ROS2 робота
Одометрия + TF + Датчики + Моторы

Функции:
- Управление моторами через /cmd_vel
- Публикация одометрии в /odom (30 Hz)
- TF трансформация odom → base_footprint
- Ультразвуковые датчики (3 шт, 10 Hz)
- Энкодеры на polling-опросе (без прерываний)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry as NavOdometry
import RPi.GPIO as GPIO
import atexit
import time
import math
import threading
from Raspi_MotorHAT import Raspi_MotorHAT
from my_robot.hardware.encoder_counter import EncoderCounter
from tf2_ros import TransformBroadcaster


class HardwareNode(Node):
    def __init__(self):
        super().__init__('hardware_node')
        
        # ===========================================
        # ПАРАМЕТРЫ (можно менять без пересборки)
        # ===========================================
        self.declare_parameter('motorhat_addr', 0x6F)
        self.declare_parameter('wheel_base', 0.12)          # Расстояние между колёсами (м)
        self.declare_parameter('wheel_diameter', 0.065)     # Диаметр колеса (м) - 65мм для TT motor
        self.declare_parameter('ticks_per_rev', 20)       # Тиков на оборот (зависит от энкодера)
        
        # ===========================================
        # ИНИЦИАЛИЗАЦИЯ ЖЕЛЕЗА
        # ===========================================
        self.init_robot()
        
        # ===========================================
        # ОДОМЕТРИЯ (переменные состояния)
        # ===========================================
        self.x = 0.0                    # Позиция X (м)
        self.y = 0.0                    # Позиция Y (м)
        self.theta = 0.0                # Ориентация (рад)
        self.last_left_ticks = 0        # Последние тики левого энкодера
        self.last_right_ticks = 0       # Последние тики правого энкодера
        self.last_time = self.get_clock().now()  # Время последнего обновления
        
        # Блокировка для потокобезопасного чтения энкодеров
        self.encoder_lock = threading.Lock()
        
        # ===========================================
        # ПУБЛИКАТОРЫ И ТАЙМЕРЫ
        # ===========================================
        # Одометрия (30 Hz = 0.033 сек)
        self.odom_pub = self.create_publisher(NavOdometry, '/odom', 10)
        self.update_timer = self.create_timer(0.033, self.update_odometry)
        
        # Датчики (10 Hz = 0.1 сек)
        self.dist_pub = self.create_publisher(Range, '/sensors/distance', 10)
        self.sensor_timer = self.create_timer(0.1, self.publish_sensors)
        
        # TF Broadcast (трансформация координат)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Heartbeat (1 Hz)
        self.heartbeat = self.create_timer(1.0, self.heartbeat_callback)
        
        cmd_vel_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Не ждать подтверждений
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        # Подписка на команды скорости
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_callback, cmd_vel_qos)
        
        # Watchdog для cmd_vel
        self.cmd_timeout = 5.0  # сек
        self.last_cmd_time = self.get_clock().now()
        self.cmd_watchdog = self.create_timer(0.1, self.cmd_watchdog_callback)

        # Регистрация остановки при выходе
        atexit.register(self.stop_all)
        
        self.get_logger().info('Hardware Node запущен! (Одометрия + TF + Датчики)')

    def init_robot(self):
        """Инициализация всего железа (моторы, GPIO, энкодеры)"""
        addr = self.get_parameter('motorhat_addr').value
        
        # Motor HAT
        self._mh = Raspi_MotorHAT(addr=addr)
        self.left_motor = self._mh.getMotor(1)
        self.right_motor = self._mh.getMotor(2)
        
        # GPIO Настройки
        GPIO.setwarnings(False)
        GPIO.cleanup()
        GPIO.setmode(GPIO.BCM)
        
        # Пины датчиков (HC-SR04)
        self.trig_left = 27
        self.echo_left = 17
        self.trig_right = 6
        self.echo_right = 5
        self.trig_front = 23
        self.echo_front = 24
        
        # Настройка пинов датчиков
        for pin in [self.trig_left, self.trig_right, self.trig_front]:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)
        
        for pin in [self.echo_left, self.echo_right, self.echo_front]:
            GPIO.setup(pin, GPIO.IN)
        
        time.sleep(0.1)  # Стабилизация
        
        # ===========================================
        # ЭНКОДЕРЫ (Polling-опрос, без прерываний)
        # ===========================================
        # Левый энкодер: GPIO 12 (или 4, проверь по схеме)
        # Правый энкодер: GPIO 26
        self.left_encoder = EncoderCounter(12)
        self.right_encoder = EncoderCounter(26)
        
        # Устанавливаем константы для конвертации тиков в расстояние
        wheel_diameter = self.get_parameter('wheel_diameter').value
        ticks_per_rev = self.get_parameter('ticks_per_rev').value
        EncoderCounter.set_constants(wheel_diameter * 1000, ticks_per_rev)  # мм
        
        self.left_ticks = 0
        self.right_ticks = 0
        
        # Переменные для отслеживания скорости
        self.current_left_speed = 0
        self.current_right_speed = 0
        
        self.get_logger().info(f'Энкодеры инициализированы: wheel_diameter={wheel_diameter}м, ticks_per_rev={ticks_per_rev}')

    def get_distance(self, trig_pin, echo_pin):
        """Измерение дистанции ультразвуковым датчиком (метод опроса)"""
        try:
            GPIO.output(trig_pin, True)
            time.sleep(0.00001)  # 10 мкс
            GPIO.output(trig_pin, False)
            
            pulse_start = time.time()
            timeout = pulse_start + 0.1  # Таймаут 100 мс
            
            # Ждём нарастающий фронт
            while GPIO.input(echo_pin) == 0:
                if time.time() > timeout:
                    return float('inf')
                pulse_start = time.time()
            
            # Ждём спадающий фронт
            pulse_end = time.time()
            while GPIO.input(echo_pin) == 1:
                if time.time() > timeout:
                    return float('inf')
                pulse_end = time.time()
            
            duration = pulse_end - pulse_start
            distance = duration * 17150  # см (скорость звука 343 м/с)
            
            # Фильтр недостоверных значений
            if distance > 200 or distance < 2:
                return float('inf')
            
            return distance / 100.0  # метры
        except Exception as e:
            self.get_logger().error(f'Ошибка датчика: {e}')
            return float('inf')

    def convert_speed(self, speed_percent):
        """Конвертация скорости из процентов в режим и ШИМ (0-255)"""
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
        """Обработка команд скорости из /cmd_vel"""
        self.last_cmd_time = self.get_clock().now()
        self.get_logger().debug(f'Команда: linear.x={msg.linear.x:.2f}, angular.z={msg.angular.z:.2f}')
        
        wheel_base = self.get_parameter('wheel_base').value
        
        # === КОЭФФИЦИЕНТ УСИЛЕНИЯ ПОВОРОТА ===
        linear_scale = 1.5   # Усиление линейной скорости
        turn_scale = 2.0  # Усиление поворота
            
        # Дифференциальная кинематика с усилением
        left = (msg.linear.x * linear_scale) - (msg.angular.z * turn_scale) * (wheel_base / 2.0)
        right = (msg.linear.x * linear_scale) + (msg.angular.z * turn_scale) * (wheel_base / 2.0)       
        # Конвертация в проценты
        left_percent = left * 100
        right_percent = right * 100

        # === МИНИМАЛЬНАЯ СКОРОСТЬ (критично для 4.8V!) ===
        min_speed = 50  # Проценты 

        if abs(left_percent) > 0 and abs(left_percent) < min_speed:
            left_percent = min_speed if left_percent > 0 else -min_speed
        if abs(right_percent) > 0 and abs(right_percent) < min_speed:
            right_percent = min_speed if right_percent > 0 else -min_speed

        # Применяем к моторам
        mode_l, speed_l = self.convert_speed(left_percent)
        mode_r, speed_r = self.convert_speed(right_percent)
        
        self.left_motor.setSpeed(speed_l)
        self.left_motor.run(mode_l)
        self.right_motor.setSpeed(speed_r)
        self.right_motor.run(mode_r)
        
        self.current_left_speed = left_percent
        self.current_right_speed = right_percent


    def cmd_watchdog_callback(self):
        """Остановить моторы если не было команд более cmd_timeout"""
        now = self.get_clock().now()
        dt = (now - self.last_cmd_time).nanoseconds / 1e9
        if dt > self.cmd_timeout:
            self.left_motor.run(Raspi_MotorHAT.RELEASE)
            self.right_motor.run(Raspi_MotorHAT.RELEASE)

    def update_odometry(self):
        """
        Расчёт и публикация одометрии (30 Hz)
        Использует дифференциальную кинематику
        """
        # Проверяем наличие энкодеров
        if not hasattr(self, 'left_encoder') or not hasattr(self, 'right_encoder'):
            return
        
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9  # дельта времени (сек)
        
        # Защита от деления на ноль и слишком малого dt
        if dt < 0.001:
            return
        
        # === ЧТЕНИЕ ЭНКОДЕРОВ (потокобезопасно) ===
        with self.encoder_lock:
            left_ticks = self.left_encoder.get_count()
            right_ticks = self.right_encoder.get_count()
        
        # Разница тиков с прошлого цикла
        diff_left = left_ticks - self.last_left_ticks
        diff_right = right_ticks - self.last_right_ticks

        # Обновляем последние значения
        self.last_left_ticks = left_ticks
        self.last_right_ticks = right_ticks
        self.last_time = current_time
        
        # === КОНВЕРТАЦИЯ ТИКОВ В МЕТРЫ ===
        wheel_diameter = self.get_parameter('wheel_diameter').value
        ticks_per_rev = self.get_parameter('ticks_per_rev').value
        ticks_to_m = (math.pi * wheel_diameter) / ticks_per_rev
        
        dist_left = diff_left * ticks_to_m
        dist_right = diff_right * ticks_to_m
        
        # === ДИФФЕРЕНЦИАЛЬНАЯ КИНЕМАТИКА ===
        wheel_base = self.get_parameter('wheel_base').value
        dist_center = (dist_left + dist_right) / 2.0  # Пройденное расстояние центра
        delta_theta = (dist_right - dist_left) / wheel_base  # Изменение угла
        
        # Обновляем позицию
        self.theta += delta_theta
        self.x += dist_center * math.cos(self.theta)
        self.y += dist_center * math.sin(self.theta)
        
       
        # === ПУБЛИКАЦИЯ ОДОМЕТРИИ ===
        odom_msg = NavOdometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'

        # Позиция
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # Ориентация
        odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        # ИСПРАВЛЕННАЯ КОВАРИАЦИЯ (все значения явно float)
        odom_msg.pose.covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1
        ]

        # Скорость
        odom_msg.twist.twist.linear.x = dist_center / dt if dt > 0 else 0.0
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = delta_theta / dt if dt > 0 else 0.0

        # КОВАРИАЦИЯ СКОРОСТИ
        odom_msg.twist.covariance = [
            0.001, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.001, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.001, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1
        ]

        self.odom_pub.publish(odom_msg)
        
        # === TF TRANSFORM (odom → base_footprint) ===
        transform = TransformStamped()
        transform.header.stamp = current_time.to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_footprint'
        
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        
        transform.transform.rotation.z = math.sin(self.theta / 2.0)
        transform.transform.rotation.w = math.cos(self.theta / 2.0)
        
        self.tf_broadcaster.sendTransform(transform)
        
        # Логирование (раз в секунду, чтобы не спамить)
        self.get_logger().debug(
            f'Odom: x={self.x:.3f}м, y={self.y:.3f}м, theta={math.degrees(self.theta):.1f}°',
            throttle_duration_sec=1.0
        )

        # === DEBUG: Лог энкодеров (удалить после тестов) ===
        self.get_logger().info(
            f'DEBUG: left_ticks={left_ticks}, right_ticks={right_ticks}, '
            f'diff_l={diff_left}, diff_r={diff_right}, '
            f'dist_l={dist_left:.4f}м, dist_r={dist_right:.4f}м',
            throttle_duration_sec=0.5
        )

    def publish_sensors(self):
        """Публикация данных с ультразвуковых датчиков (10 Hz)"""
        try:
            left = self.get_distance(self.trig_left, self.echo_left)
            time.sleep(0.05)  # Пауза между датчиками
            right = self.get_distance(self.trig_right, self.echo_right)
            time.sleep(0.05)
            front = self.get_distance(self.trig_front, self.echo_front)
            
            self.publish_sensor('left', left)
            self.publish_sensor('right', right)
            self.publish_sensor('front', front)
        except Exception as e:
            self.get_logger().error(f'Ошибка датчиков: {e}')

    def publish_sensor(self, position, distance):
        """Публикация одного датчика в топик /sensors/distance"""
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
        """Heartbeat лог (раз в секунду)"""
        self.get_logger().info(
            f'Hardware Node работает | Odom: x={self.x:.2f}, y={self.y:.2f}',
            throttle_duration_sec=5
        )

    def stop_all(self):
        """Остановка всех устройств и очистка GPIO"""
        self.get_logger().info('Остановка робота...')
        
        # Остановка моторов
        self.left_motor.run(Raspi_MotorHAT.RELEASE)
        self.right_motor.run(Raspi_MotorHAT.RELEASE)
        
        # Остановка энкодеров
        if hasattr(self, 'left_encoder'):
            self.left_encoder.stop()
        if hasattr(self, 'right_encoder'):
            self.right_encoder.stop()
        
        # Очистка GPIO
        GPIO.cleanup()
        
        self.get_logger().info('Все устройства остановлены')


def main(args=None):
    """Точка входа узла"""
    rclpy.init(args=args)
    node = HardwareNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Получен сигнал остановки (Ctrl+C)')
    finally:
        node.stop_all()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
