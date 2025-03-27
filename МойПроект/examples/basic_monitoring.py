#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Пример базового мониторинга с использованием поверхностного дрона.

Этот пример демонстрирует настройку и запуск поверхностного дрона
для выполнения базовой миссии мониторинга окружающей среды.

Лицензия: MIT
"""

import sys
import os
import time
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime

# Добавляем путь к исходному коду в PYTHONPATH
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../src')))

# Импортируем класс дрона из основного модуля
from main import SurfaceDrone


def setup_monitoring_mission():
    """
    Настройка и запуск миссии мониторинга.
    """
    print("Инициализация поверхностного дрона...")
    drone = SurfaceDrone()
    
    # Создаем маршрут мониторинга (прямоугольный периметр вокруг озера)
    print("Настройка маршрута мониторинга...")
    waypoints = [
        (55.7500, 37.6000),  # Северо-западный угол
        (55.7500, 37.6100),  # Северо-восточный угол
        (55.7400, 37.6100),  # Юго-восточный угол
        (55.7400, 37.6000),  # Юго-западный угол
        (55.7500, 37.6000),  # Возвращение в исходную точку
    ]
    
    # Устанавливаем начальную позицию дрона
    drone.gps.set_position(55.7500, 37.6000)
    
    # Устанавливаем маршрут
    drone.set_mission(waypoints)
    
    # Запускаем дрон и мониторинг
    print("Запуск дрона и активация мониторинга...")
    drone.start()
    drone.start_monitoring(interval=30)  # Сбор данных каждые 30 секунд
    
    # Устанавливаем скорость движения
    drone.motion.set_speed(0.5)  # 0.5 м/с для детального мониторинга
    
    # Симуляция миссии мониторинга
    print("Начало миссии мониторинга...")
    
    # Массивы для хранения данных для последующей визуализации
    timestamps = []
    positions = []
    temperatures = []
    battery_levels = []
    
    # Симуляция работы дрона в течение 1 часа
    simulation_time = 3600  # секунды
    time_step = 10  # секунды
    
    for t in range(0, simulation_time, time_step):
        # Обновляем состояние дрона
        drone.update(time_step)
        
        # Каждую минуту выводим информацию и собираем данные для графиков
        if t % 60 == 0:
            lat, lon = drone.gps.get_position()
            battery_level = drone.battery.get_charge_level()
            
            # Получаем данные с сенсоров
            sensor_data = drone.sensors.read_sensors()
            temperature = sensor_data["temperature"]
            
            print(f"Время: {t//60} мин, Позиция: ({lat:.6f}, {lon:.6f}), "
                  f"Температура: {temperature:.1f}°C, Батарея: {battery_level:.1f}%")
            
            # Сохраняем данные для графиков
            timestamps.append(t // 60)  # в минутах
            positions.append((lat, lon))
            temperatures.append(temperature)
            battery_levels.append(battery_level)
        
        # Имитация реального времени
        time.sleep(0.01)  # Небольшая задержка для снижения нагрузки на CPU
    
    # Останавливаем дрон и сохраняем данные
    print("Завершение миссии мониторинга...")
    drone.stop_monitoring()
    drone.stop()
    
    # Сохраняем данные миссии
    mission_name = f"basic_monitoring_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    data_file = drone.save_mission_data(mission_name)
    print(f"Данные миссии сохранены в {data_file}")
    
    # Визуализация результатов
    visualize_mission_results(timestamps, positions, temperatures, battery_levels)


def visualize_mission_results(timestamps, positions, temperatures, battery_levels):
    """
    Визуализация результатов миссии мониторинга.
    
    Args:
        timestamps: Список временных меток (в минутах)
        positions: Список позиций (широта, долгота)
        temperatures: Список значений температуры
        battery_levels: Список уровней заряда батареи
    """
    # Создаем фигуру с несколькими графиками
    fig = plt.figure(figsize=(15, 10))
    
    # График 1: Маршрут дрона
    ax1 = fig.add_subplot(2, 2, 1)
    lats, lons = zip(*positions)
    ax1.plot(lons, lats, 'b-', marker='o')
    ax1.set_title('Маршрут дрона')
    ax1.set_xlabel('Долгота')
    ax1.set_ylabel('Широта')
    ax1.grid(True)
    
    # Добавляем стрелки для обозначения направления движения
    for i in range(len(positions) - 1):
        lat1, lon1 = positions[i]
        lat2, lon2 = positions[i + 1]
        dx = lon2 - lon1
        dy = lat2 - lat1
        ax1.arrow(lon1, lat1, dx, dy, head_width=0.0005, head_length=0.001, 
                  fc='blue', ec='blue', length_includes_head=True)
    
    # График 2: Температура
    ax2 = fig.add_subplot(2, 2, 2)
    ax2.plot(timestamps, temperatures, 'r-', marker='o')
    ax2.set_title('Температура')
    ax2.set_xlabel('Время (мин)')
    ax2.set_ylabel('Температура (°C)')
    ax2.grid(True)
    
    # График 3: Уровень заряда батареи
    ax3 = fig.add_subplot(2, 2, 3)
    ax3.plot(timestamps, battery_levels, 'g-', marker='o')
    ax3.set_title('Уровень заряда батареи')
    ax3.set_xlabel('Время (мин)')
    ax3.set_ylabel('Заряд (%)')
    ax3.grid(True)
    
    # График 4: Тепловая карта температуры
    ax4 = fig.add_subplot(2, 2, 4)
    
    # Создаем сетку для тепловой карты
    min_lat = min(lats) - 0.001
    max_lat = max(lats) + 0.001
    min_lon = min(lons) - 0.001
    max_lon = max(lons) + 0.001
    
    grid_size = 20
    lat_grid = np.linspace(min_lat, max_lat, grid_size)
    lon_grid = np.linspace(min_lon, max_lon, grid_size)
    
    # Интерполяция температуры на сетку
    temp_grid = np.zeros((grid_size, grid_size))
    
    # Простая интерполяция - для каждой точки сетки находим ближайшую точку измерения
    for i, lat in enumerate(lat_grid):
        for j, lon in enumerate(lon_grid):
            # Находим ближайшую точку измерения
            min_dist = float('inf')
            closest_temp = 0
            
            for k in range(len(positions)):
                pos_lat, pos_lon = positions[k]
                dist = (lat - pos_lat) ** 2 + (lon - pos_lon) ** 2
                
                if dist < min_dist:
                    min_dist = dist
                    closest_temp = temperatures[k]
            
            temp_grid[i, j] = closest_temp
    
    # Отображаем тепловую карту
    im = ax4.imshow(temp_grid, extent=[min_lon, max_lon, min_lat, max_lat], 
                    origin='lower', cmap='jet', aspect='auto')
    ax4.set_title('Тепловая карта температуры')
    ax4.set_xlabel('Долгота')
    ax4.set_ylabel('Широта')
    plt.colorbar(im, ax=ax4, label='Температура (°C)')
    
    # Добавляем маршрут на тепловую карту
    ax4.plot(lons, lats, 'k-', marker='o', linewidth=1)
    
    # Настройка общего вида
    plt.tight_layout()
    
    # Сохраняем график
    plt.savefig('mission_results.png')
    print("График результатов миссии сохранен в mission_results.png")
    
    # Показываем график
    plt.show()


if __name__ == "__main__":
    setup_monitoring_mission()
