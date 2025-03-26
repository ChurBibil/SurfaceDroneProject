#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Основной модуль для управления поверхностным дроном с автономным управлением
и солнечным питанием для задач мониторинга.

Этот модуль реализует базовую функциональность для:
1. Управления движением дрона
2. Сбора данных с сенсоров
3. Планирования маршрута
4. Управления энергопотреблением
5. Обработки и хранения собранных данных

Автор: Manus
Лицензия: MIT
"""

import time
import math
import logging
import json
from datetime import datetime
from typing import Dict, List, Tuple, Optional, Any

# Настройка логирования
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler("drone.log"),
        logging.StreamHandler()
    ]
)

logger = logging.getLogger("SurfaceDrone")


class SolarPanel:
    """
    Класс для моделирования солнечной панели и расчета генерируемой энергии.
    """
    
    def __init__(self, area: float, efficiency: float):
        """
        Инициализация солнечной панели.
        
        Args:
            area: Площадь солнечной панели в квадратных метрах
            efficiency: Эффективность преобразования солнечной энергии (0-1)
        """
        self.area = area  # м²
        self.efficiency = efficiency  # КПД (0-1)
        self.current_power = 0.0  # Текущая генерируемая мощность в Вт
        logger.info(f"Солнечная панель инициализирована: площадь={area}м², КПД={efficiency}")
    
    def calculate_power(self, solar_irradiance: float) -> float:
        """
        Расчет генерируемой мощности на основе солнечного излучения.
        
        Args:
            solar_irradiance: Интенсивность солнечного излучения в Вт/м²
            
        Returns:
            float: Генерируемая мощность в Вт
        """
        self.current_power = self.area * self.efficiency * solar_irradiance
        logger.debug(f"Генерируемая мощность: {self.current_power:.2f} Вт")
        return self.current_power


class Battery:
    """
    Класс для моделирования аккумулятора дрона.
    """
    
    def __init__(self, capacity: float, max_discharge_rate: float, max_charge_rate: float):
        """
        Инициализация аккумулятора.
        
        Args:
            capacity: Емкость аккумулятора в Вт*ч
            max_discharge_rate: Максимальная скорость разряда в Вт
            max_charge_rate: Максимальная скорость заряда в Вт
        """
        self.capacity = capacity  # Вт*ч
        self.current_charge = capacity  # Текущий заряд в Вт*ч
        self.max_discharge_rate = max_discharge_rate  # Вт
        self.max_charge_rate = max_charge_rate  # Вт
        logger.info(f"Аккумулятор инициализирован: емкость={capacity}Вт*ч")
    
    def discharge(self, power: float, time_hours: float) -> float:
        """
        Разряд аккумулятора.
        
        Args:
            power: Потребляемая мощность в Вт
            time_hours: Время разряда в часах
            
        Returns:
            float: Фактически использованная энергия в Вт*ч
        """
        if power > self.max_discharge_rate:
            logger.warning(f"Запрошенная мощность {power}Вт превышает максимальную {self.max_discharge_rate}Вт")
            power = self.max_discharge_rate
        
        energy_required = power * time_hours
        energy_available = min(energy_required, self.current_charge)
        self.current_charge -= energy_available
        
        charge_percentage = (self.current_charge / self.capacity) * 100
        logger.debug(f"Разряд: {energy_available:.2f}Вт*ч, оставшийся заряд: {charge_percentage:.1f}%")
        
        return energy_available
    
    def charge(self, power: float, time_hours: float) -> float:
        """
        Заряд аккумулятора.
        
        Args:
            power: Зарядная мощность в Вт
            time_hours: Время заряда в часах
            
        Returns:
            float: Фактически полученная энергия в Вт*ч
        """
        if power > self.max_charge_rate:
            logger.warning(f"Зарядная мощность {power}Вт превышает максимальную {self.max_charge_rate}Вт")
            power = self.max_charge_rate
        
        energy_input = power * time_hours
        energy_capacity_left = self.capacity - self.current_charge
        energy_stored = min(energy_input, energy_capacity_left)
        self.current_charge += energy_stored
        
        charge_percentage = (self.current_charge / self.capacity) * 100
        logger.debug(f"Заряд: {energy_stored:.2f}Вт*ч, текущий заряд: {charge_percentage:.1f}%")
        
        return energy_stored
    
    def get_charge_level(self) -> float:
        """
        Получение текущего уровня заряда в процентах.
        
        Returns:
            float: Уровень заряда (0-100%)
        """
        return (self.current_charge / self.capacity) * 100


class GPSModule:
    """
    Класс для моделирования GPS-модуля дрона.
    """
    
    def __init__(self, accuracy: float = 2.5):
        """
        Инициализация GPS-модуля.
        
        Args:
            accuracy: Точность GPS в метрах
        """
        self.accuracy = accuracy  # м
        self.current_position = (0.0, 0.0)  # (широта, долгота)
        logger.info(f"GPS-модуль инициализирован: точность={accuracy}м")
    
    def get_position(self) -> Tuple[float, float]:
        """
        Получение текущих GPS-координат.
        
        Returns:
            Tuple[float, float]: Координаты (широта, долгота)
        """
        # В реальной системе здесь был бы код для получения данных с GPS-приемника
        # Для симуляции просто возвращаем текущую позицию
        return self.current_position
    
    def set_position(self, latitude: float, longitude: float) -> None:
        """
        Установка текущих GPS-координат (для симуляции).
        
        Args:
            latitude: Широта
            longitude: Долгота
        """
        self.current_position = (latitude, longitude)
        logger.debug(f"Позиция обновлена: {latitude}, {longitude}")
    
    def calculate_distance(self, target_position: Tuple[float, float]) -> float:
        """
        Расчет расстояния до целевой точки.
        
        Args:
            target_position: Координаты целевой точки (широта, долгота)
            
        Returns:
            float: Расстояние в метрах
        """
        # Используем формулу гаверсинусов для расчета расстояния между двумя точками на сфере
        lat1, lon1 = self.current_position
        lat2, lon2 = target_position
        
        # Радиус Земли в метрах
        R = 6371000
        
        # Перевод в радианы
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        delta_lat = math.radians(lat2 - lat1)
        delta_lon = math.radians(lon2 - lon1)
        
        # Формула гаверсинусов
        a = math.sin(delta_lat/2) * math.sin(delta_lat/2) + \
            math.cos(lat1_rad) * math.cos(lat2_rad) * \
            math.sin(delta_lon/2) * math.sin(delta_lon/2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        distance = R * c
        
        return distance


class EnvironmentalSensors:
    """
    Класс для моделирования экологических сенсоров дрона.
    """
    
    def __init__(self):
        """
        Инициализация экологических сенсоров.
        """
        self.sensors = {
            "temperature": 0.0,  # °C
            "humidity": 0.0,     # %
            "pressure": 0.0,     # гПа
            "water_quality": 0.0,  # pH
            "air_quality": 0.0,    # AQI
        }
        logger.info("Экологические сенсоры инициализированы")
    
    def read_sensors(self) -> Dict[str, float]:
        """
        Чтение данных со всех сенсоров.
        
        Returns:
            Dict[str, float]: Словарь с показаниями сенсоров
        """
        # В реальной системе здесь был бы код для получения данных с физических сенсоров
        # Для симуляции генерируем случайные значения
        import random
        
        self.sensors["temperature"] = 20.0 + random.uniform(-5.0, 5.0)
        self.sensors["humidity"] = 60.0 + random.uniform(-10.0, 10.0)
        self.sensors["pressure"] = 1013.0 + random.uniform(-10.0, 10.0)
        self.sensors["water_quality"] = 7.0 + random.uniform(-0.5, 0.5)
        self.sensors["air_quality"] = 50.0 + random.uniform(-20.0, 20.0)
        
        logger.debug(f"Показания сенсоров: {self.sensors}")
        return self.sensors


class MotionController:
    """
    Класс для управления движением дрона.
    """
    
    def __init__(self, max_speed: float):
        """
        Инициализация контроллера движения.
        
        Args:
            max_speed: Максимальная скорость в м/с
        """
        self.max_speed = max_speed  # м/с
        self.current_speed = 0.0  # м/с
        self.current_direction = 0.0  # градусы (0 - север, 90 - восток и т.д.)
        self.power_consumption = 0.0  # Вт
        logger.info(f"Контроллер движения инициализирован: макс. скорость={max_speed}м/с")
    
    def set_speed(self, speed: float) -> None:
        """
        Установка скорости движения.
        
        Args:
            speed: Целевая скорость в м/с
        """
        if speed > self.max_speed:
            logger.warning(f"Запрошенная скорость {speed}м/с превышает максимальную {self.max_speed}м/с")
            speed = self.max_speed
        
        self.current_speed = speed
        # Расчет энергопотребления (упрощенная модель)
        self.power_consumption = 10.0 + (speed ** 2) * 0.5  # Базовое + квадратичная зависимость от скорости
        
        logger.debug(f"Скорость установлена: {speed}м/с, потребление: {self.power_consumption:.2f}Вт")
    
    def set_direction(self, direction: float) -> None:
        """
        Установка направления движения.
        
        Args:
            direction: Направление в градусах (0-359)
        """
        # Нормализация угла
        direction = direction % 360
        self.current_direction = direction
        logger.debug(f"Направление установлено: {direction}°")
    
    def get_power_consumption(self) -> float:
        """
        Получение текущего энергопотребления.
        
        Returns:
            float: Энергопотребление в Вт
        """
        return self.power_consumption


class DataStorage:
    """
    Класс для хранения и обработки собранных данных.
    """
    
    def __init__(self, storage_path: str = "data/"):
        """
        Инициализация хранилища данных.
        
        Args:
            storage_path: Путь для сохранения данных
        """
        self.storage_path = storage_path
        self.current_session_data = []
        
        # Создаем директорию для хранения данных, если она не существует
        import os
        os.makedirs(storage_path, exist_ok=True)
        
        logger.info(f"Хранилище данных инициализировано: путь={storage_path}")
    
    def store_data_point(self, data: Dict[str, Any]) -> None:
        """
        Сохранение точки данных в текущей сессии.
        
        Args:
            data: Словарь с данными для сохранения
        """
        # Добавляем временную метку
        data["timestamp"] = datetime.now().isoformat()
        self.current_session_data.append(data)
        logger.debug(f"Данные сохранены: {data}")
    
    def save_session(self, session_name: Optional[str] = None) -> str:
        """
        Сохранение текущей сессии данных в файл.
        
        Args:
            session_name: Имя сессии (если None, используется текущая дата/время)
            
        Returns:
            str: Путь к сохраненному файлу
        """
        if session_name is None:
            session_name = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        file_path = f"{self.storage_path}{session_name}.json"
        
        with open(file_path, 'w') as f:
            json.dump(self.current_session_data, f, indent=2)
        
        logger.info(f"Сессия сохранена: {file_path}, {len(self.current_session_data)} записей")
        return file_path
    
    def clear_session(self) -> None:
        """
        Очистка данных текущей сессии.
        """
        self.current_session_data = []
        logger.debug("Текущая сессия очищена")


class MissionPlanner:
    """
    Класс для планирования миссий мониторинга.
    """
    
    def __init__(self, gps: GPSModule):
        """
        Инициализация планировщика миссий.
        
        Args:
            gps: Экземпляр GPS-модуля
        """
        self.gps = gps
        self.waypoints = []  # Список точек маршрута [(lat, lon), ...]
        self.current_waypoint_index = 0
        logger.info("Планировщик миссий инициализирован")
    
    def set_waypoints(self, waypoints: List[Tuple[float, float]]) -> None:
        """
        Установка маршрута миссии.
        
        Args:
            waypoints: Список координат точек маршрута [(lat, lon), ...]
        """
        self.waypoints = waypoints
        self.current_waypoint_index = 0
        logger.info(f"Маршрут установлен: {len(waypoints)} точек")
    
    def get_next_waypoint(self) -> Optional[Tuple[float, float]]:
        """
        Получение следующей точки маршрута.
        
        Returns:
            Optional[Tuple[float, float]]: Координаты следующей точки или None, если маршрут завершен
        """
        if not self.waypoints or self.current_waypoint_index >= len(self.waypoints):
            logger.info("Маршрут завершен")
            return None
        
        next_waypoint = self.waypoints[self.current_waypoint_index]
        logger.debug(f"Следующая точка маршрута: {next_waypoint}")
        return next_waypoint
    
    def advance_to_next_waypoint(self) -> None:
        """
        Переход к следующей точке маршрута.
        """
        self.current_waypoint_index += 1
        if self.current_waypoint_index < len(self.waypoints):
            logger.debug(f"Переход к точке {self.current_waypoint_index}: {self.waypoints[self.current_waypoint_index]}")
        else:
            logger.info("Достигнута последняя точка маршрута")
    
    def calculate_direction_to_waypoint(self) -> Optional[float]:
        """
        Расчет направления к следующей точке маршрута.
        
        Returns:
            Optional[float]: Направление в градусах или None, если маршрут завершен
        """
        next_waypoint = self.get_next_waypoint()
        if next_waypoint is None:
            return None
        
        current_lat, current_lon = self.gps.get_position()
        target_lat, target_lon = next_waypoint
        
        # Расчет направления (азимута) от текущей позиции к целевой
        # Формула: θ = atan2(sin(Δlong).cos(lat2), cos(lat1).sin(lat2) − sin(lat1).cos(lat2).cos(Δlong))
        
        # Перевод в радианы
        lat1 = math.radians(current_lat)
        lon1 = math.radians(current_lon)
        lat2 = math.radians(target_lat)
        lon2 = math.radians(target_lon)
        
        delta_lon = lon2 - lon1
        
        y = math.sin(delta_lon) * math.cos(lat2)
        x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(delta_lon)
        
        # Перевод из радиан в градусы и нормализация (0-360)
        direction = (math.degrees(math.atan2(y, x)) + 360) % 360
        
        logger.debug(f"Направление к следующей точке: {direction:.1f}°")
        return direction
    
    def is_waypoint_reached(self, distance_threshold: float = 10.0) -> bool:
        """
        Проверка достижения текущей точки маршрута.
        
        Args:
            distance_threshold: Пороговое расстояние в метрах для считания точки достигнутой
            
        Returns:
            bool: True, если точка достигнута, иначе False
        """
        next_waypoint = self.get_next_waypoint()
        if next_waypoint is None:
            return True
        
        distance = self.gps.calculate_distance(next_waypoint)
        is_reached = distance <= distance_threshold
        
        if is_reached:
            logger.info(f"Точка {self.current_waypoint_index} достигнута")
        
        return is_reached


class SurfaceDrone:
    """
    Основной класс для управления поверхностным дроном.
    """
    
    def __init__(self):
        """
        Инициализация дрона и всех его компонентов.
        """
        # Инициализация компонентов
        self.solar_panel = SolarPanel(area=1.5, efficiency=0.22)
        self.battery = Battery(capacity=500.0, max_discharge_rate=100.0, max_charge_rate=50.0)
        self.gps = GPSModule(accuracy=2.0)
        self.sensors = EnvironmentalSensors()
        self.motion = MotionController(max_speed=2.0)
        self.data_storage = DataStorage()
        self.mission_planner = MissionPlanner(self.gps)
        
        # Состояние дрона
        self.is_active = False
        self.monitoring_active = False
        self.monitoring_interval = 60  # секунды
        self.last_monitoring_time = 0
        
        logger.info("Поверхностный дрон инициализирован")
    
    def start(self) -> None:
        """
        Запуск дрона.
        """
        self.is_active = True
        logger.info("Дрон запущен")
    
    def stop(self) -> None:
        """
        Остановка дрона.
        """
        self.is_active = False
        self.motion.set_speed(0.0)
        logger.info("Дрон остановлен")
    
    def start_monitoring(self, interval: int = 60) -> None:
        """
        Запуск режима мониторинга.
        
        Args:
            interval: Интервал сбора данных в секундах
        """
        self.monitoring_active = True
        self.monitoring_interval = interval
        self.last_monitoring_time = 0  # Сбросим таймер, чтобы сразу начать мониторинг
        logger.info(f"Мониторинг запущен с интервалом {interval} сек")
    
    def stop_monitoring(self) -> None:
        """
        Остановка режима мониторинга.
        """
        self.monitoring_active = False
        logger.info("Мониторинг остановлен")
    
    def update(self, delta_time: float) -> None:
        """
        Обновление состояния дрона.
        
        Args:
            delta_time: Прошедшее время с последнего обновления в секундах
        """
        if not self.is_active:
            return
        
        # Преобразуем delta_time в часы для расчетов энергии
        delta_hours = delta_time / 3600.0
        
        # Симуляция солнечного излучения (упрощенно)
        # В реальной системе здесь был бы код для получения данных с датчика освещенности
        solar_irradiance = 800.0  # Вт/м² (среднее значение при ясном небе)
        
        # Расчет генерируемой энергии
        solar_power = self.solar_panel.calculate_power(solar_irradiance)
        
        # Расчет потребляемой энергии
        power_consumption = self.motion.get_power_consumption() + 5.0  # Базовое потребление электроники
        
        # Энергетический баланс
        energy_balance = solar_power - power_consumption
        
        if energy_balance > 0:
            # Избыток энергии - заряжаем батарею
            self.battery.charge(energy_balance, delta_hours)
        else:
            # Недостаток энергии - разряжаем батарею
            self.battery.discharge(abs(energy_balance), delta_hours)
        
        # Проверка уровня заряда батареи
        battery_level = self.battery.get_charge_level()
        if battery_level < 10.0:
            logger.warning(f"Низкий уровень заряда батареи: {battery_level:.1f}%")
            # Снижаем скорость для экономии энергии
            self.motion.set_speed(self.motion.current_speed * 0.5)
        
        # Обновление позиции (симуляция)
        # В реальной системе здесь был бы код для получения данных с GPS
        current_lat, current_lon = self.gps.get_position()
        
        # Расчет нового положения на основе скорости и направления
        speed_m_per_sec = self.motion.current_speed
        direction_rad = math.radians(self.motion.current_direction)
        
        # Расстояние, пройденное за delta_time
        distance_m = speed_m_per_sec * delta_time
        
        # Примерное изменение координат (упрощенная модель)
        # 111111 м = 1 градус широты
        # 111111 * cos(latitude) м = 1 градус долготы
        lat_change = (distance_m * math.cos(direction_rad)) / 111111
        lon_change = (distance_m * math.sin(direction_rad)) / (111111 * math.cos(math.radians(current_lat)))
        
        new_lat = current_lat + lat_change
        new_lon = current_lon + lon_change
        
        self.gps.set_position(new_lat, new_lon)
        
        # Проверка достижения текущей точки маршрута
        if self.mission_planner.is_waypoint_reached():
            self.mission_planner.advance_to_next_waypoint()
            
            # Получаем следующую точку и направление
            next_direction = self.mission_planner.calculate_direction_to_waypoint()
            if next_direction is not None:
                self.motion.set_direction(next_direction)
        
        # Мониторинг окружающей среды
        self.last_monitoring_time += delta_time
        if self.monitoring_active and self.last_monitoring_time >= self.monitoring_interval:
            self.collect_monitoring_data()
            self.last_monitoring_time = 0
    
    def collect_monitoring_data(self) -> None:
        """
        Сбор данных мониторинга.
        """
        # Получение данных с сенсоров
        sensor_data = self.sensors.read_sensors()
        
        # Добавление GPS-координат и других данных
        data_point = {
            "position": self.gps.get_position(),
            "battery_level": self.battery.get_charge_level(),
            "solar_power": self.solar_panel.current_power,
            "speed": self.motion.current_speed,
            "direction": self.motion.current_direction,
            **sensor_data
        }
        
        # Сохранение данных
        self.data_storage.store_data_point(data_point)
        logger.info("Данные мониторинга собраны и сохранены")
    
    def set_mission(self, waypoints: List[Tuple[float, float]]) -> None:
        """
        Установка миссии для дрона.
        
        Args:
            waypoints: Список координат точек маршрута [(lat, lon), ...]
        """
        self.mission_planner.set_waypoints(waypoints)
        
        # Устанавливаем направление к первой точке
        direction = self.mission_planner.calculate_direction_to_waypoint()
        if direction is not None:
            self.motion.set_direction(direction)
        
        logger.info(f"Миссия установлена: {len(waypoints)} точек")
    
    def save_mission_data(self, mission_name: Optional[str] = None) -> str:
        """
        Сохранение данных миссии.
        
        Args:
            mission_name: Имя миссии (если None, используется текущая дата/время)
            
        Returns:
            str: Путь к сохраненному файлу
        """
        return self.data_storage.save_session(mission_name)


def main():
    """
    Основная функция для демонстрации работы дрона.
    """
    # Создаем экземпляр дрона
    drone = SurfaceDrone()
    
    # Устанавливаем маршрут мониторинга (координаты точек)
    waypoints = [
        (55.7558, 37.6173),  # Пример координат (Москва)
        (55.7600, 37.6200),
        (55.7650, 37.6250),
        (55.7700, 37.6300),
        (55.7650, 37.6350),
        (55.7600, 37.6300),
        (55.7558, 37.6173),  # Возвращаемся в исходную точку
    ]
    
    drone.set_mission(waypoints)
    
    # Запускаем дрон и мониторинг
    drone.start()
    drone.start_monitoring(interval=30)  # Сбор данных каждые 30 секунд
    
    # Устанавливаем начальную скорость
    drone.motion.set_speed(1.0)  # 1 м/с
    
    # Симуляция работы дрона в течение 1 часа
    simulation_time = 3600  # секунды
    time_step = 10  # секунды
    
    print("Запуск симуляции работы дрона...")
    
    for t in range(0, simulation_time, time_step):
        # Обновляем состояние дрона
        drone.update(time_step)
        
        # Выводим текущее состояние каждую минуту
        if t % 60 == 0:
            lat, lon = drone.gps.get_position()
            battery_level = drone.battery.get_charge_level()
            speed = drone.motion.current_speed
            
            print(f"Время: {t//60} мин, Позиция: ({lat:.6f}, {lon:.6f}), "
                  f"Батарея: {battery_level:.1f}%, Скорость: {speed:.1f} м/с")
        
        # Имитация реального времени
        time.sleep(0.01)  # Небольшая задержка для снижения нагрузки на CPU
    
    # Останавливаем дрон и сохраняем данные
    drone.stop_monitoring()
    drone.stop()
    
    data_file = drone.save_mission_data("demo_mission")
    print(f"Симуляция завершена. Данные сохранены в {data_file}")


if __name__ == "__main__":
    main()
