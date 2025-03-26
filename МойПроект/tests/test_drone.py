#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Модуль тестирования для поверхностного дрона.

Этот модуль содержит тесты для проверки основных компонентов и функциональности
поверхностного дрона с автономным управлением и солнечным питанием.

Автор: Manus
Лицензия: MIT
"""

import unittest
import sys
import os
import math
from unittest.mock import patch, MagicMock

# Добавляем путь к исходному коду в PYTHONPATH
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../src')))

# Импортируем классы из основного модуля
from main import (
    SolarPanel, Battery, GPSModule, EnvironmentalSensors,
    MotionController, DataStorage, MissionPlanner, SurfaceDrone
)


class TestSolarPanel(unittest.TestCase):
    """Тесты для класса SolarPanel."""
    
    def setUp(self):
        """Настройка тестового окружения."""
        self.solar_panel = SolarPanel(area=2.0, efficiency=0.2)
    
    def test_initialization(self):
        """Тест инициализации солнечной панели."""
        self.assertEqual(self.solar_panel.area, 2.0)
        self.assertEqual(self.solar_panel.efficiency, 0.2)
        self.assertEqual(self.solar_panel.current_power, 0.0)
    
    def test_calculate_power(self):
        """Тест расчета генерируемой мощности."""
        # Проверка с нулевым излучением
        power = self.solar_panel.calculate_power(0.0)
        self.assertEqual(power, 0.0)
        
        # Проверка с типичным значением излучения
        power = self.solar_panel.calculate_power(1000.0)
        expected_power = 2.0 * 0.2 * 1000.0  # area * efficiency * irradiance
        self.assertEqual(power, expected_power)
        self.assertEqual(self.solar_panel.current_power, expected_power)


class TestBattery(unittest.TestCase):
    """Тесты для класса Battery."""
    
    def setUp(self):
        """Настройка тестового окружения."""
        self.battery = Battery(capacity=100.0, max_discharge_rate=20.0, max_charge_rate=10.0)
    
    def test_initialization(self):
        """Тест инициализации аккумулятора."""
        self.assertEqual(self.battery.capacity, 100.0)
        self.assertEqual(self.battery.current_charge, 100.0)
        self.assertEqual(self.battery.max_discharge_rate, 20.0)
        self.assertEqual(self.battery.max_charge_rate, 10.0)
    
    def test_discharge(self):
        """Тест разряда аккумулятора."""
        # Нормальный разряд
        energy_used = self.battery.discharge(10.0, 1.0)
        self.assertEqual(energy_used, 10.0)
        self.assertEqual(self.battery.current_charge, 90.0)
        
        # Разряд с превышением максимальной мощности
        energy_used = self.battery.discharge(30.0, 1.0)
        self.assertEqual(energy_used, 20.0)  # Ограничено max_discharge_rate
        self.assertEqual(self.battery.current_charge, 70.0)
        
        # Разряд с превышением доступной энергии
        energy_used = self.battery.discharge(20.0, 4.0)
        self.assertEqual(energy_used, 70.0)  # Ограничено current_charge
        self.assertEqual(self.battery.current_charge, 0.0)
    
    def test_charge(self):
        """Тест заряда аккумулятора."""
        # Сначала разрядим аккумулятор
        self.battery.discharge(50.0, 1.0)
        self.assertEqual(self.battery.current_charge, 50.0)
        
        # Нормальный заряд
        energy_stored = self.battery.charge(5.0, 1.0)
        self.assertEqual(energy_stored, 5.0)
        self.assertEqual(self.battery.current_charge, 55.0)
        
        # Заряд с превышением максимальной мощности
        energy_stored = self.battery.charge(15.0, 1.0)
        self.assertEqual(energy_stored, 10.0)  # Ограничено max_charge_rate
        self.assertEqual(self.battery.current_charge, 65.0)
        
        # Заряд с превышением емкости
        energy_stored = self.battery.charge(10.0, 4.0)
        self.assertEqual(energy_stored, 35.0)  # Ограничено оставшейся емкостью
        self.assertEqual(self.battery.current_charge, 100.0)
    
    def test_get_charge_level(self):
        """Тест получения уровня заряда в процентах."""
        self.assertEqual(self.battery.get_charge_level(), 100.0)
        
        self.battery.discharge(25.0, 1.0)
        self.assertEqual(self.battery.get_charge_level(), 75.0)
        
        self.battery.discharge(50.0, 1.0)
        self.assertEqual(self.battery.get_charge_level(), 25.0)


class TestGPSModule(unittest.TestCase):
    """Тесты для класса GPSModule."""
    
    def setUp(self):
        """Настройка тестового окружения."""
        self.gps = GPSModule(accuracy=2.0)
    
    def test_initialization(self):
        """Тест инициализации GPS-модуля."""
        self.assertEqual(self.gps.accuracy, 2.0)
        self.assertEqual(self.gps.get_position(), (0.0, 0.0))
    
    def test_set_get_position(self):
        """Тест установки и получения позиции."""
        self.gps.set_position(55.7558, 37.6173)
        lat, lon = self.gps.get_position()
        self.assertEqual(lat, 55.7558)
        self.assertEqual(lon, 37.6173)
    
    def test_calculate_distance(self):
        """Тест расчета расстояния между точками."""
        # Установим текущую позицию
        self.gps.set_position(55.7558, 37.6173)
        
        # Расчет расстояния до той же точки
        distance = self.gps.calculate_distance((55.7558, 37.6173))
        self.assertAlmostEqual(distance, 0.0, places=1)
        
        # Расчет расстояния до другой точки
        # Москва - Санкт-Петербург примерно 634 км
        distance = self.gps.calculate_distance((59.9343, 30.3351))
        self.assertGreater(distance, 600000)  # Более 600 км
        self.assertLess(distance, 650000)     # Менее 650 км


class TestEnvironmentalSensors(unittest.TestCase):
    """Тесты для класса EnvironmentalSensors."""
    
    def setUp(self):
        """Настройка тестового окружения."""
        self.sensors = EnvironmentalSensors()
    
    def test_initialization(self):
        """Тест инициализации сенсоров."""
        self.assertEqual(self.sensors.sensors["temperature"], 0.0)
        self.assertEqual(self.sensors.sensors["humidity"], 0.0)
        self.assertEqual(self.sensors.sensors["pressure"], 0.0)
        self.assertEqual(self.sensors.sensors["water_quality"], 0.0)
        self.assertEqual(self.sensors.sensors["air_quality"], 0.0)
    
    @patch('random.uniform')
    def test_read_sensors(self, mock_uniform):
        """Тест чтения данных с сенсоров."""
        # Настройка мока для генерации предсказуемых значений
        mock_uniform.return_value = 1.0
        
        # Чтение данных с сенсоров
        data = self.sensors.read_sensors()
        
        # Проверка, что все ожидаемые ключи присутствуют
        self.assertIn("temperature", data)
        self.assertIn("humidity", data)
        self.assertIn("pressure", data)
        self.assertIn("water_quality", data)
        self.assertIn("air_quality", data)
        
        # Проверка, что значения были обновлены
        self.assertEqual(data["temperature"], 20.0 + 1.0)
        self.assertEqual(data["humidity"], 60.0 + 1.0)
        self.assertEqual(data["pressure"], 1013.0 + 1.0)
        self.assertEqual(data["water_quality"], 7.0 + 1.0)
        self.assertEqual(data["air_quality"], 50.0 + 1.0)


class TestMotionController(unittest.TestCase):
    """Тесты для класса MotionController."""
    
    def setUp(self):
        """Настройка тестового окружения."""
        self.motion = MotionController(max_speed=5.0)
    
    def test_initialization(self):
        """Тест инициализации контроллера движения."""
        self.assertEqual(self.motion.max_speed, 5.0)
        self.assertEqual(self.motion.current_speed, 0.0)
        self.assertEqual(self.motion.current_direction, 0.0)
        self.assertEqual(self.motion.power_consumption, 0.0)
    
    def test_set_speed(self):
        """Тест установки скорости."""
        # Нормальная скорость
        self.motion.set_speed(3.0)
        self.assertEqual(self.motion.current_speed, 3.0)
        self.assertGreater(self.motion.power_consumption, 0.0)
        
        # Превышение максимальной скорости
        self.motion.set_speed(10.0)
        self.assertEqual(self.motion.current_speed, 5.0)  # Ограничено max_speed
    
    def test_set_direction(self):
        """Тест установки направления."""
        # Нормальное направление
        self.motion.set_direction(90.0)
        self.assertEqual(self.motion.current_direction, 90.0)
        
        # Нормализация угла
        self.motion.set_direction(370.0)
        self.assertEqual(self.motion.current_direction, 10.0)
    
    def test_get_power_consumption(self):
        """Тест получения энергопотребления."""
        self.motion.set_speed(2.0)
        power = self.motion.get_power_consumption()
        # Проверка, что энергопотребление рассчитывается корректно
        expected_power = 10.0 + (2.0 ** 2) * 0.5
        self.assertEqual(power, expected_power)


class TestDataStorage(unittest.TestCase):
    """Тесты для класса DataStorage."""
    
    def setUp(self):
        """Настройка тестового окружения."""
        # Используем временную директорию для тестов
        self.test_dir = "/tmp/drone_test_data/"
        self.storage = DataStorage(storage_path=self.test_dir)
    
    def tearDown(self):
        """Очистка после тестов."""
        # Удаляем тестовую директорию
        import shutil
        if os.path.exists(self.test_dir):
            shutil.rmtree(self.test_dir)
    
    def test_initialization(self):
        """Тест инициализации хранилища данных."""
        self.assertEqual(self.storage.storage_path, self.test_dir)
        self.assertEqual(self.storage.current_session_data, [])
        self.assertTrue(os.path.exists(self.test_dir))
    
    def test_store_data_point(self):
        """Тест сохранения точки данных."""
        test_data = {"temperature": 25.0, "position": (55.7558, 37.6173)}
        self.storage.store_data_point(test_data)
        
        # Проверка, что данные сохранены в текущей сессии
        self.assertEqual(len(self.storage.current_session_data), 1)
        
        # Проверка, что временная метка добавлена
        self.assertIn("timestamp", self.storage.current_session_data[0])
        
        # Проверка, что исходные данные сохранены
        self.assertEqual(self.storage.current_session_data[0]["temperature"], 25.0)
        self.assertEqual(self.storage.current_session_data[0]["position"], (55.7558, 37.6173))
    
    def test_save_session(self):
        """Тест сохранения сессии в файл."""
        # Добавляем тестовые данные
        test_data = {"temperature": 25.0, "position": (55.7558, 37.6173)}
        self.storage.store_data_point(test_data)
        
        # Сохраняем сессию
        file_path = self.storage.save_session("test_session")
        
        # Проверка, что файл создан
        self.assertTrue(os.path.exists(file_path))
        
        # Проверка содержимого файла
        import json
        with open(file_path, 'r') as f:
            saved_data = json.load(f)
        
        self.assertEqual(len(saved_data), 1)
        self.assertEqual(saved_data[0]["temperature"], 25.0)
        self.assertEqual(saved_data[0]["position"], [55.7558, 37.6173])  # JSON преобразует кортежи в списки
    
    def test_clear_session(self):
        """Тест очистки текущей сессии."""
        # Добавляем тестовые данные
        test_data = {"temperature": 25.0}
        self.storage.store_data_point(test_data)
        
        # Проверка, что данные добавлены
        self.assertEqual(len(self.storage.current_session_data), 1)
        
        # Очищаем сессию
        self.storage.clear_session()
        
        # Проверка, что данные очищены
        self.assertEqual(len(self.storage.current_session_data), 0)


class TestMissionPlanner(unittest.TestCase):
    """Тесты для класса MissionPlanner."""
    
    def setUp(self):
        """Настройка тестового окружения."""
        self.gps = GPSModule()
        self.mission_planner = MissionPlanner(self.gps)
    
    def test_initialization(self):
        """Тест инициализации планировщика миссий."""
        self.assertEqual(self.mission_planner.waypoints, [])
        self.assertEqual(self.mission_planner.current_waypoint_index, 0)
    
    def test_set_waypoints(self):
        """Тест установки маршрута."""
        waypoints = [(55.7558, 37.6173), (55.7600, 37.6200)]
        self.mission_planner.set_waypoints(waypoints)
        
        self.assertEqual(self.mission_planner.waypoints, waypoints)
        self.assertEqual(self.mission_planner.current_waypoint_index, 0)
    
    def test_get_next_waypoint(self):
        """Тест получения следующей точки маршрута."""
        # Пустой маршрут
        self.assertIsNone(self.mission_planner.get_next_waypoint())
        
        # Установка маршрута
        waypoints = [(55.7558, 37.6173), (55.7600, 37.6200)]
        self.mission_planner.set_waypoints(waypoints)
        
        # Получение первой точки
        next_waypoint = self.mission_planner.get_next_waypoint()
        self.assertEqual(next_waypoint, waypoints[0])
        
        # Переход к следующей точке
        self.mission_planner.advance_to_next_waypoint()
        next_waypoint = self.mission_planner.get_next_waypoint()
        self.assertEqual(next_waypoint, waypoints[1])
        
        # Переход за пределы маршрута
        self.mission_planner.advance_to_next_waypoint()
        self.assertIsNone(self.mission_planner.get_next_waypoint())
    
    def test_calculate_direction_to_waypoint(self):
        """Тест расчета направления к следующей точке маршрута."""
        # Установка текущей позиции
        self.gps.set_position(55.7558, 37.6173)
        
        # Установка маршрута
        waypoints = [(55.7600, 37.6200)]  # Точка на северо-восток от текущей
        self.mission_planner.set_waypoints(waypoints)
        
        # Расчет направления
        direction = self.mission_planner.calculate_direction_to_waypoint()
        
        # Направление должно быть в первом квадранте (0-90 градусов)
        self.assertGreaterEqual(direction, 0.0)
        self.assertLessEqual(direction, 90.0)
    
    def test_is_waypoint_reached(self):
        """Тест проверки достижения точки маршрута."""
        # Установка текущей позиции
        self.gps.set_position(55.7558, 37.6173)
        
        # Установка маршрута с точкой, совпадающей с текущей позицией
        waypoints = [(55.7558, 37.6173)]
        self.mission_planner.set_waypoints(waypoints)
        
        # Проверка достижения точки
        self.assertTrue(self.mission_planner.is_waypoint_reached())
        
        # Установка маршрута с удаленной точкой
        waypoints = [(55.8000, 37.7000)]  # Точка далеко от текущей
        self.mission_planner.set_waypoints(waypoints)
        
        # Проверка недостижения точки
        self.assertFalse(self.mission_planner.is_waypoint_reached())


class TestSurfaceDrone(unittest.TestCase):
    """Тесты для класса SurfaceDrone."""
    
    def setUp(self):
        """Настройка тестового окружения."""
        # Создаем мок для DataStorage, чтобы избежать записи в файловую систему
        with patch('main.DataStorage') as mock_data_storage:
            self.drone = SurfaceDrone()
            # Заменяем реальный объект DataStorage на мок
            self.drone.data_storage = MagicMock()
    
    def test_initialization(self):
        """Тест инициализации дрона."""
        self.assertIsInstance(self.drone.solar_panel, SolarPanel)
        self.assertIsInstance(self.drone.battery, Battery)
        self.assertIsInstance(self.drone.gps, GPSModule)
        self.assertIsInstance(self.drone.sensors, EnvironmentalSensors)
        self.assertIsInstance(self.drone.motion, MotionController)
        self.assertIsInstance(self.drone.mission_planner, MissionPlanner)
        
        self.assertFalse(self.drone.is_active)
        self.assertFalse(self.drone.monitoring_active)
    
    def test_start_stop(self):
        """Тест запуска и остановки дрона."""
        # Запуск
        self.drone.start()
        self.assertTrue(self.drone.is_active)
        
        # Остановка
        self.drone.stop()
        self.assertFalse(self.drone.is_active)
        self.assertEqual(self.drone.motion.current_speed, 0.0)
    
    def test_start_stop_monitoring(self):
        """Тест запуска и остановки мониторинга."""
        # Запуск мониторинга
        self.drone.start_monitoring(interval=30)
        self.assertTrue(self.drone.monitoring_active)
        self.assertEqual(self.drone.monitoring_interval, 30)
        
        # Остановка мониторинга
        self.drone.stop_monitoring()
        self.assertFalse(self.drone.monitoring_active)
    
    def test_set_mission(self):
        """Тест установки миссии."""
        waypoints = [(55.7558, 37.6173), (55.7600, 37.6200)]
        self.drone.set_mission(waypoints)
        
        self.assertEqual(self.drone.mission_planner.waypoints, waypoints)
    
    def test_collect_monitoring_data(self):
        """Тест сбора данных мониторинга."""
        # Настройка мока для чтения сенсоров
        self.drone.sensors.read_sensors = MagicMock(return_value={"temperature": 25.0})
        
        # Сбор данных
        self.drone.collect_monitoring_data()
        
        # Проверка, что данные были сохранены
        self.drone.data_storage.store_data_point.assert_called_once()
        
        # Проверка, что в сохраненных данных есть все необходимые поля
        args, _ = self.drone.data_storage.store_data_point.call_args
        data_point = args[0]
        
        self.assertIn("position", data_point)
        self.assertIn("battery_level", data_point)
        self.assertIn("solar_power", data_point)
        self.assertIn("speed", data_point)
        self.assertIn("direction", data_point)
        self.assertIn("temperature", data_point)
    
    def test_update(self):
        """Тест обновления состояния дрона."""
        # Запуск дрона
        self.drone.start()
        
        # Установка начальной позиции
        self.drone.gps.set_position(55.7558, 37.6173)
        
        # Установка скорости
        self.drone.motion.set_speed(1.0)
        
        # Обновление состояния
        self.drone.update(10.0)  # 10 секунд
        
        # Проверка, что позиция изменилась
        lat, lon = self.drone.gps.get_position()
        self.assertNotEqual((lat, lon), (55.7558, 37.6173))


if __name__ == '__main__':
    unittest.main()
