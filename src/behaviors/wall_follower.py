from robot import Robot
from time import sleep, time

class WallFollower:
    def __init__(self, robot):
        self.robot = robot
        self.target_distance = 0.25  # 25 см от стены
        self.speed = 40
        self.head_scan_interval = 5.0  # сканировать каждые 5 секунд
        self.last_head_scan = 0

        # Настройки для улучшения:
        self.min_safe_distance = 0.15 # Минимальное безопасное растояние
        self.emergency_back_speed = 70  # СИЛА обратного хода при опасности (было 60)
        self.turn_power = 0.6  # СИЛА поворота при корректировке (было 0.8)
        self.reaction_threshold = 0.08  # ПОРОГ реакции на изменение расстояния
        
    def get_all_distances(self):
        """Получаем данные со всех датчиков"""
        left = self.robot.left_distance_sensor.distance
        right = self.robot.right_distance_sensor.distance
        front = self.robot.front_distance_sensor.distance

        print(f"[SENSORS] Left: {left:.2f}m,Right: {right:.2f}m, Front: {front:.2f}m") 
        
        # Определяем, с какой стороны стена
        if left < right:
            wall_side = 'left'
            wall_distance = left
            other_side = right
        else:
            wall_side = 'right'
            wall_distance = right
            other_side = left
            
        return {
            'wall_side': wall_side,
            'wall_distance': wall_distance,
            'other_side': other_side,
            'front': front,
            'left': left,
            'right': right
        }
    
    def adjust_for_wall(self, distances):
        """Корректируем движение в зависимости от расстояния до стены"""
        wall_distance = distances['wall_distance']
        wall_side = distances['wall_side']
        left_raw = distances['left']
        right_raw = distances['right']

        error = wall_distance - self.target_distance

        # ЭКСТРЕННАЯ СИТУАЦИЯ: слишком близко к любой стене
        if left_raw < self.min_safe_distance or right_raw < self.min_safe_distance:
            print(f"⚠️ ЭКСТРЕННО: слишком близко к стене! L:{left_raw:.2f}м, R:{right_raw:.2f}м")
            
            # Определяем, с какой стороны опасность
            if left_raw < right_raw:
                # Опасность слева - резко поворачиваем направо
                print("Резкий поворот направо")
                return -int(self.speed * 0.7), self.speed  # назад левым, вперед правым
            else:
                # Опасность справа - резко поворачиваем налево
                print("Резкий поворот налево")
                return self.speed, -int(self.speed * 0.7)  # вперед левым, назад правым
        
        # НОРМАЛЬНАЯ КОРРЕКЦИЯ
        if abs(error) < self.reaction_threshold:  # в пределах допустимого
            left_speed = self.speed
            right_speed = self.speed
            print(f"✓ Идеально: {wall_distance:.2f}м от стены ({wall_side})")
            
        elif error > 0:  # слишком далеко от стены
            print(f"↷ Далеко от стены: {wall_distance:.2f}м ({wall_side})")
            if wall_side == 'left':
                # Стена слева - поворачиваем налево
                left_speed = int(self.speed * self.turn_power)  # МЕНЬШЕ
                right_speed = self.speed
            else:
                # Стена справа - поворачиваем направо
                left_speed = self.speed
                right_speed = int(self.speed * self.turn_power)  # МЕНЬШЕ
                
        else:  # слишком близко к стене (error < 0)
            print(f"↶ Близко к стене: {wall_distance:.2f}м ({wall_side})")
            if wall_side == 'left':
                # Стена слева - поворачиваем направо
                left_speed = self.speed
                right_speed = int(self.speed * self.turn_power)  # МЕНЬШЕ
            else:
                # Стена справа - поворачиваем налево
                left_speed = int(self.speed * self.turn_power)  # МЕНЬШЕ
                right_speed = self.speed
        
        return left_speed, right_speed
    
    def handle_front_obstacle(self, front_distance):
        """Обработка фронтальных препятствий - УСИЛЕННАЯ"""
        if front_distance < 0.25:  # УМЕНЬШИЛ порог с 0.3 до 0.25
            print(f"🚫 Фронтальное препятствие: {front_distance:.2f}м")
            
            # СИЛЬНЫЙ обратный ход
            self.robot.stop_all()
            sleep(0.3)
            
            print("Отъезжаю назад СИЛЬНО")
            self.robot.set_left(-self.emergency_back_speed)  # УСИЛИЛ
            self.robot.set_right(-self.emergency_back_speed) # УСИЛИЛ
            sleep(0.8)  # УВЕЛИЧИЛ время
            
            self.robot.stop_all()
            sleep(0.3)
            
            # Определяем, куда лучше повернуть
            left_space = self.scan_direction(-25)  # смотрим налево
            right_space = self.scan_direction(25)  # смотрим направо
            
            # Возвращаем голову в центр
            self.robot.set_pan(0)
            sleep(0.3)
            
            # Выбираем направление
            if left_space > right_space and left_space > 0.4:
                print(f"↶ Поворачиваю налево (свободно: {left_space:.2f}м)")
                self.robot.set_left(-self.speed)
                self.robot.set_right(self.speed)
                sleep(0.9)  # УВЕЛИЧИЛ время поворота
            elif right_space > 0.4:
                print(f"↷ Поворачиваю направо (свободно: {right_space:.2f}м)")
                self.robot.set_left(self.speed)
                self.robot.set_right(-self.speed)
                sleep(0.9)
            else:
                print("↔️ Оба направления заняты, еще назад")
                self.robot.set_left(-self.emergency_back_speed)
                self.robot.set_right(-self.emergency_back_speed)
                sleep(1.2)  # ДОЛЬШЕ назад
            
            self.robot.stop_all()
            sleep(0.5)
            return True
        
        return False
    
    def scan_direction(self, angle):
        """Сканируем расстояние в указанном направлении"""
        self.robot.set_pan(angle)
        sleep(0.4)  # УМЕНЬШИЛ задержку для скорости
        distance = self.robot.front_distance_sensor.distance
        return distance
    
    def check_camera_safety(self):
        """Проверяем безопасность для камеры - УПРОЩЕННАЯ"""
        camera_distance = self.robot.front_distance_sensor.distance
        
        if camera_distance < 0.3:  # УМЕНЬШИЛ порог с 0.35 до 0.3
            print(f"⚠️ Камера в опасности! {camera_distance:.2f}м")
            
            # Просто отъезжаем назад
            self.robot.stop_all()
            sleep(0.3)
            
            self.robot.set_left(-self.emergency_back_speed)
            self.robot.set_right(-self.emergency_back_speed)
            sleep(0.7)
            
            self.robot.stop_all()
            sleep(0.3)
            return True
        
        return False
    
    def periodic_head_scan(self):
        """Периодическое сканирование - УПРОЩЕННОЕ"""
        current_time = time()
        if current_time - self.last_head_scan > self.head_scan_interval:
            print("🔍 Сканирую пространство...")
            
            # Быстрое сканирование только вперед
            self.robot.set_pan(0)
            sleep(0.3)
            front_dist = self.robot.front_distance_sensor.distance
            
            print(f"  Впереди: {front_dist:.2f}м")
            
            self.last_head_scan = current_time
            return front_dist
        
        return None
    
    def follow_wall(self, duration=60):
        """Основной цикл следования вдоль стены - УПРОЩЕННЫЙ"""
        print("=" * 50)
        print("ЗАПУСК СЛЕДОВАНИЯ ВДОЛЬ СТЕНЫ")
        print(f"Целевое расстояние: {self.target_distance}м")
        print(f"Минимальное безопасное: {self.min_safe_distance}м")
        print(f"Скорость: {self.speed}%")
        print("=" * 50)
        
        # Устанавливаем голову
        self.robot.set_pan(0)
        self.robot.set_tilt(0)
        sleep(0.6)
        
        start_time = time()
        stuck_counter = 0  # Счетчик застреваний
        last_position_time = time()
        
        try:
            while time() - start_time < duration:
                # 1. Получаем данные
                distances = self.get_all_distances()
                
                # 2. Проверяем, не застряли ли мы (простая проверка)
                current_time = time()
                if current_time - last_position_time > 3.0:  # 3 секунды на одном месте
                    stuck_counter += 1
                    print(f"⚠️ Возможно застревание #{stuck_counter}")
                    
                    if stuck_counter >= 2:
                        print("СИЛЬНЫЙ обратный ход от застревания")
                        self.robot.set_left(-self.emergency_back_speed)
                        self.robot.set_right(-self.emergency_back_speed)
                        sleep(1.0)
                        self.robot.stop_all()
                        sleep(0.5)
                        stuck_counter = 0
                
                last_position_time = current_time
                
                # 3. Проверяем камеру
                if self.check_camera_safety():
                    continue
                
                # 4. Проверяем фронтальные препятствия
                if self.handle_front_obstacle(distances['front']):
                    continue
                
                # 5. Корректируем движение
                left_speed, right_speed = self.adjust_for_wall(
                    distances['wall_distance'],
                    distances['wall_side'],
                    distances['left_raw'],
                    distances['right_raw']
                )
                
                # 6. Периодическое сканирование
                self.periodic_head_scan()
                
                # 7. Двигаемся
                self.robot.set_left(left_speed)
                self.robot.set_right(right_speed)
                
                # 8. Задержка
                sleep(0.15)  # УМЕНЬШИЛ задержку для более частой реакции
                
        except KeyboardInterrupt:
            print("\nПрервано пользователем")
        except Exception as e:
            print(f"\nОшибка: {e}")
        finally:
            print("\nОстанавливаю робота...")
            self.robot.stop_all()
            self.robot.set_pan(0)
            self.robot.set_tilt(0)
            print("Робот остановлен")


# ТЕСТИРОВАНИЕ С РАЗНЫМИ НАСТРОЙКАМИ
if __name__ == "__main__":
    print("Инициализация робота...")
    bot = Robot()
    
    follower = WallFollower(bot)
    
    # ЭКСПЕРИМЕНТИРУЙ С ЭТИМИ НАСТРОЙКАМИ:
    
    # Вариант 1: Агрессивный режим (больше мощности)
    follower.speed = 60
    follower.emergency_back_speed = 80
    follower.min_safe_distance = 0.12
    
    # Вариант 2: Осторожный режим (медленнее, но безопаснее)
    #follower.speed = 35
    #follower.emergency_back_speed = 60
    #follower.min_safe_distance = 0.18
    #follower.target_distance = 0.30  # дальше от стены
    
    # Вариант 3: Режим отладки (медленно, с подробным выводом)
    # follower.speed = 30
    # follower.emergency_back_speed = 50
    # follower.head_scan_interval = 2.0
    
    # Запускаем на 90 секунд для теста
    follower.follow_wall(duration=90)