#!/usr/bin/env python3
import RPi.GPIO as GPIO
import math
import threading
import time

class EncoderCounter:
    """Счётчик энкодера на базе опроса (polling)"""
    
    ticks_to_mm_const = None
    
    def __init__(self, pin_number, callback=None):
        """
        pin_number: номер GPIO пина
        callback: функция для вызова при каждом импульсе (опционально)
        """
        self.pin_number = pin_number
        self.pulse_count = 0
        self.direction = 1
        self.callback = callback
        self._running = True
        self._last_state = None
        
        # Блокировка для потокобезопасного доступа
        self.lock = threading.Lock()

        # Настройка пина
        GPIO.setup(self.pin_number, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        # Запуск потока опроса
        self._thread = threading.Thread(target=self._polling_loop)
        self._thread.daemon = True
        self._thread.start()
    
    def _polling_loop(self):
        """Бесконечный цикл опроса пина"""
        while self._running:
            current_state = GPIO.input(self.pin_number)
            
            # Детектируем спад (переход из 1 в 0)
            if self._last_state == 1 and current_state == 0:
                with self.lock:  # Защита записи
                    self.pulse_count += self.direction
                if self.callback:
                    self.callback()
            
            self._last_state = current_state
            time.sleep(0.0001)  # 500 мкс - оптимально для энкодеров
    
    def get_count(self):
        """Вернуть текущее значение счётчика"""
        with self.lock: # Защита чтения
            return self.pulse_count
    
    def reset(self):
        """Сбросить счётчик"""
        with self.lock:
            self.pulse_count = 0
    
    def set_direction(self, direction):
        """Установить направление (1 или -1)"""
        if abs(direction) == 1:
            self.direction = direction
    
    def stop(self):
        """Остановить поток опроса"""
        self._running = False
        if hasattr(self, '_thread') and self._thread.is_alive():
            self._thread.join(timeout=1.0)
    
    def distance_in_mm(self):
        """Конвертация в миллиметры"""
        if EncoderCounter.ticks_to_mm_const is None:
            raise ValueError("ticks_to_mm_const not set")
        return int(self.pulse_count * EncoderCounter.ticks_to_mm_const)
    
    @staticmethod
    def set_constants(wheel_diameter_mm, ticks_per_revolution):
        EncoderCounter.ticks_to_mm_const = (math.pi * wheel_diameter_mm) / ticks_per_revolution
