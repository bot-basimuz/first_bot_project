from robot import Robot
from time import sleep, time

class WallFollower:
    def __init__(self, robot):
        self.robot = robot
        self.target_distance = 0.25  # 25 см от стены
        self.speed = 40
        self.head_scan_interval = 5.0  # сканировать каждые 5 секунд
        self.last_head_scan = 0
        
    def get_all_distances(self):
        """Получаем данные со всех датчиков"""
        left = self.robot.left_distance_sensor.distance  # исправлено: sensor, не sencor
        right = self.robot.right_distance_sensor.distance
        front = self.robot.front_distance_sensor.distance
        
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
            'front': front
        }
    
    def adjust_for_wall(self, wall_distance, wall_side):
        """Корректируем движение в зависимости от расстояния до стены"""
        error = wall_distance - self.target_distance
        
        # Если расстояние в пределах допустимого отклонения (±5 см)
        if abs(error) < 0.05:
            # Идеально - едем прямо
            left_speed = self.speed
            right_speed = self.speed
            print("Движение прямо")
            
        elif error > 0:  # слишком далеко от стены
            print(f"Слишком далеко от стены ({wall_side}): {wall_distance:.2f}м")
            if wall_side == 'left':
                # Стена слева - плавно поворачиваем налево
                left_speed = int(self.speed * 0.8)
                right_speed = self.speed
            else:
                # Стена справа - плавно поворачиваем направо
                left_speed = self.speed
                right_speed = int(self.speed * 0.8)
                
        else:  # слишком близко к стене (error < 0)
            print(f"Слишком близко к стене ({wall_side}): {wall_distance:.2f}м")
            if wall_side == 'left':
                # Стена слева - плавно поворачиваем направо
                left_speed = self.speed
                right_speed = int(self.speed * 0.8)
            else:
                # Стена справа - плавно поворачиваем налево
                left_speed = int(self.speed * 0.8)
                right_speed = self.speed
        
        return left_speed, right_speed
    
    def handle_front_obstacle(self, front_distance):
        """Обработка фронтальных препятствий"""
        if front_distance < 0.3:  # препятствие ближе 30 см
            print(f"Фронтальное препятствие: {front_distance:.2f}м")
            
            # Останавливаемся
            self.robot.stop_all()
            sleep(0.5)
            
            # Определяем, куда лучше повернуть
            # Сканируем по сторонам головой
            left_space = self.scan_direction(-30)  # смотрим налево
            right_space = self.scan_direction(30)  # смотрим направо
            
            # Возвращаем голову в центр
            self.robot.set_pan(0)
            sleep(0.3)
            
            # Выбираем направление с большим пространством
            if left_space > right_space and left_space > 0.5:
                print(f"Поворачиваю налево (свободно: {left_space:.2f}м)")
                # Поворот на месте налево
                self.robot.set_left(-self.speed)
                self.robot.set_right(self.speed)
                sleep(0.8)
            elif right_space > 0.5:
                print(f"Поворачиваю направо (свободно: {right_space:.2f}м)")
                # Поворот на месте направо
                self.robot.set_left(self.speed)
                self.robot.set_right(-self.speed)
                sleep(0.8)
            else:
                print("Оба направления заняты, отъезжаю назад")
                # Отъезжаем назад
                self.robot.set_left(-int(self.speed * 0.6))
                self.robot.set_right(-int(self.speed * 0.6))
                sleep(1.0)
            
            return True  # препятствие было обработано
        
        return False  # препятствий нет
    
    def scan_direction(self, angle):
        """Сканируем расстояние в указанном направлении"""
        self.robot.set_pan(angle)
        sleep(0.5)  # даем время сервоприводу установиться
        distance = self.robot.front_distance_sensor.distance
        return distance
    
    def check_camera_safety(self):
        """Проверяем безопасность для камеры"""
        # Смотрим прямо вперед на уровне камеры
        self.robot.set_pan(0)
        sleep(0.1)
        camera_distance = self.robot.front_distance_sensor.distance
        
        if camera_distance < 0.35:  # опасно для камеры
            print(f"⚠️ Опасность для камеры! Расстояние: {camera_distance:.2f}м")
            # Останавливаемся
            self.robot.stop_all()
            sleep(0.5)
            
            # Наклоняем голову вниз, чтобы посмотреть, что там
            current_tilt = 0  # предполагаем, что сейчас 0
            self.robot.set_tilt(20)  # смотрим вниз
            sleep(0.5)
            
            # Если и внизу препятствие - это низкое препятствие
            down_distance = self.robot.front_distance_sensor.distance
            self.robot.set_tilt(0)  # возвращаем в исходное
            
            if down_distance < 0.3:
                print("Низкое препятствие, отъезжаю назад")
                self.robot.set_left(-int(self.speed * 0.6))
                self.robot.set_right(-int(self.speed * 0.6))
                sleep(1.0)
                self.robot.stop_all()
                return True
        
        return False
    
    def periodic_head_scan(self):
        """Периодическое сканирование пространства головой"""
        current_time = time()
        if current_time - self.last_head_scan > self.head_scan_interval:
            print("Периодическое сканирование пространства...")
            
            # Сканируем три позиции: лево, центр, право
            distances = []
            for angle in [-25, 0, 25]:
                dist = self.scan_direction(angle)
                distances.append((angle, dist))
                print(f"  Угол {angle}°: {dist:.2f}м")
            
            # Возвращаем в центр
            self.robot.set_pan(0)
            sleep(0.3)
            
            self.last_head_scan = current_time
            
            # Если впереди много пространства, можно увеличить скорость
            front_dist = distances[1][1]  # расстояние в центре
            if front_dist > 1.0:
                print("Перед роботом много пространства")
                # Можно что-то сделать, например, запомнить это
            
            return distances
        
        return None
    
    def follow_wall(self, duration=60):
        """Основной цикл следования вдоль стены"""
        print("Запуск следования вдоль стены")
        print(f"Целевое расстояние до стены: {self.target_distance:.2f}м")
        print(f"Скорость: {self.speed}%")
        print("Нажмите Ctrl+C для остановки")
        print("-" * 50)
        
        # Устанавливаем голову в начальное положение
        self.robot.set_pan(0)
        self.robot.set_tilt(0)
        sleep(0.8)
        
        start_time = time()
        
        try:
            while time() - start_time < duration:
                # 1. Получаем данные со всех датчиков
                distances = self.get_all_distances()
                
                # 2. Выводим информацию для отладки
                print(f"L: {distances['wall_distance']:.2f}m ({distances['wall_side']}) | "
                      f"Front: {distances['front']:.2f}m | "
                      f"Other: {distances['other_side']:.2f}m")
                
                # 3. Проверяем безопасность камеры
                if self.check_camera_safety():
                    # Если была опасность, продолжаем снова
                    continue
                
                # 4. Обрабатываем фронтальные препятствия
                if self.handle_front_obstacle(distances['front']):
                    # Если было препятствие, получаем новые данные
                    distances = self.get_all_distances()
                
                # 5. Корректируем движение относительно стены
                left_speed, right_speed = self.adjust_for_wall(
                    distances['wall_distance'], 
                    distances['wall_side']
                )
                
                # 6. Периодическое сканирование головой
                scan_result = self.periodic_head_scan()
                
                # 7. Применяем скорости движения
                self.robot.set_left(left_speed)
                self.robot.set_right(right_speed)
                
                # 8. Короткая задержка для стабильности
                sleep(0.1)
                
        except KeyboardInterrupt:
            print("\nПрервано пользователем")
        except Exception as e:
            print(f"\nОшибка: {e}")
        finally:
            # Гарантированная остановка
            print("\nОстанавливаю робота...")
            self.robot.stop_all()
            self.robot.set_pan(0)
            self.robot.set_tilt(0)
            sleep(0.5)
            print("Робот остановлен")


# Тестирование
if __name__ == "__main__":
    print("Инициализация робота...")
    bot = Robot()
    
    # Создаем объект для следования вдоль стены
    follower = WallFollower(bot)
    
    # Настрой параметры при необходимости
    follower.target_distance = 0.20  # 30 см от стены (можешь изменить)
    follower.speed = 60              # скорость движения (можешь изменить)
    follower.head_scan_interval = 3.0  # сканировать каждые 3 секунды
    
    # Запускаем на 2 минуты (120 секунд)
    follower.follow_wall(duration=120)
