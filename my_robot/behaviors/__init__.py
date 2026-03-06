def __init__(self):
    super().__init__('room_explorer')
    
    # Параметры
    self.declare_parameter('wheel_base', 0.09)  # добавить первой
    self.declare_parameter('base_speed', 60)
    self.declare_parameter('loop_hz', 10)
