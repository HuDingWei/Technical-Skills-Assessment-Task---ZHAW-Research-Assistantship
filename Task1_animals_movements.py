# -*- coding: utf-8 -*-
"""
Created on Fri Jul  5 12:17:07 2024

@author: Hu Ding-Wei
"""
import numpy as np
import matplotlib.pyplot as plt
import csv
import random
import math
import twd97
from pyproj import Proj, Transformer

from flask import Flask, request, jsonify
from flask_socketio import SocketIO, emit
import threading
import time
import json
import socket

#############################################   TASK1   #############################################

# Earth Radius(m)
EARTH_RADIUS = 6371000

# Define WGS84 and UTM coordinate systems
transformer_to_utm = Transformer.from_crs("epsg:4326", "epsg:32632")     # WGS84 to UTM zone 32N
transformer_to_latlon = Transformer.from_crs("epsg:32632", "epsg:4326")  # UTM zone 32N to WGS84

# latitude and longitude coordinates(WGS84), Example: Zurich
latitude = 47.3769
longitude = 8.5417
lonlat = (latitude, longitude)

def generate_modified_degree90_movement(num_points, distance, origin_xy):
    """
    Generates animal movement data.
    """
    movements = []
    (x, y) = origin_xy[0]
    movements.append((x, y))
    print(movements)

    # Initialize the previous direction angle to a random direction
    previous_theta = random.uniform(0, 2 * np.pi)
    steps_since_last_change = 0
    total_turn_angle = 0

    for _ in range(num_points - 1):
        distance = random.uniform(0, 0.3) #Random step size
        
        # If the cumulative angle exceeds 90 degrees, the direction is maintained
        if abs(total_turn_angle) < np.pi / 2:
            theta_change = random.uniform(-np.pi / 12, np.pi / 12)  # Allow up to ±15 degrees of rotation per step
            total_turn_angle += theta_change
        else:
            theta_change = 0  # When the accumulated turning angle exceeds 90 degrees, there will be no turning
        
        theta = previous_theta + theta_change
        x += distance * np.cos(theta)
        y += distance * np.sin(theta)
        movements.append((x, y))
        
        # Update the previous direction angle
        previous_theta = theta
        
        # The accumulated angle is reset after every 6 steps
        steps_since_last_change += 1
        if steps_since_last_change >= 6:
            steps_since_last_change = 0
            total_turn_angle = 0

    return movements

def latlon_to_utm(coordinates_latlon):
    lon, lat = coordinates_latlon
    xy_movements = []
    # Convert latitude and longitude to UTM coordinates
    x, y = transformer_to_utm.transform(lon, lat)
    xy_movements.append((x, y))
    return xy_movements

def utm_to_lonlat(coordinates_utm):
    # x, y = coordinates_utm[]
    latlon_movements =[]
    # Convert UTM coordinates to latitude and longitude
    for x, y in coordinates_utm:
        lon, lat = transformer_to_latlon.transform(x, y)
        latlon_movements.append((lon, lat))
    return latlon_movements

def save_to_csv(movements, filename):
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['Longitude', 'Latitude'])
        for movement in movements:
            writer.writerow(movement)

def plot_movements(movements):
    x, y = zip(*movements)
    plt.plot(x, y, marker='o')
    plt.xlabel('Longitude')
    plt.ylabel('Latitude')
    plt.title('Animal Movement Trajectory')
    plt.show()


num_points = 50000
distance = random.uniform(0, 0.3)  # Random distance for each step(m)
origin = (longitude, latitude)  # WGS84 Zurcih（lon, lat）

utm_movements = latlon_to_utm(origin)
movements = generate_modified_degree90_movement(num_points, distance, utm_movements)
wgs84_movements = utm_to_lonlat(movements)
# wgs84_movements = convert_to_wgs84(movements, origin)
# save_to_csv(utm_movements, 'UTM32_movements.csv')
save_to_csv(wgs84_movements, 'WGS84_movements.csv')
plot_movements(wgs84_movements)