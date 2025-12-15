from Raspi_MotorHAT.Raspi_PWM_Servo_Driver import PWM

class Servos: 
    """PCA 9865 Servo motors"""

    def __init__(self, addr=0x6f, deflect_90_in_ms=0.8):
        """addr: Адрес i2c микросхемы ШИМ.
        deflect_90_in_ms: устанавливаем значение, полученное при калибровке сервопривода.
        это значение соответствует повороту на 90°
        (длительность соответствующего импульса в мс)"""
        self._pwm = PWM(addr)
        # Устанавливаем временной базис
        pwm_frequency = 100 
        self._pwm.setPWMFreq(pwm_frequency)
        servo_mid_point_ms = 1.3
        # Частота = 1/период, но, так как длина импульса измеряется в мс, мы берем 1000
        period_in_ms = 1000 / pwm_frequency
        # Микросхема отводит 4096 тактов на каждый период
        pulse_steps = 4096
        # Кол-во тактов на каждую мс 
        steps_per_ms = pulse_steps / period_in_ms
        # Такты на каждый градус
        self.steps_per_degree = (deflect_90_in_ms * steps_per_ms) / 90
        # Импульс для перехода  в среднее положение в тактах. 
        self.servo_mid_point_steps = servo_mid_point_ms * steps_per_ms

          # Map for channels
        self.channels = [0, 1, 14, 15]

    def stop_all(self):
        # 0 in start is nothing
        off_bit = 4096  # bit 12 is the OFF bit.
        self._pwm.setPWM(self.channels[0], 0, off_bit)
        self._pwm.setPWM(self.channels[1], 0, off_bit)
        self._pwm.setPWM(self.channels[2], 0, off_bit)
        self._pwm.setPWM(self.channels[3], 0, off_bit)

    def _convert_degrees_to_steps(self, position):
        return int(self.servo_mid_point_steps + (position * self.steps_per_degree))
        
    def set_servo_angle(self, channel, angle):
        """ position: The position in degrees from the center. -90 to 90 """
        # Проверка
        if angle > 90 or angle < -90:
            raise ValueError("Angle outside of range")
        # Задаем положение
        off_step = self._convert_degrees_to_steps(angle)
        self._pwm.setPWM(self.channels[channel], 0, off_step)
