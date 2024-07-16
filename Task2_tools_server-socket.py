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

#############################################   TASK2-Server   #############################################

# Earth Radius(m)
EARTH_RADIUS = 6371000

# Define WGS84 and UTM coordinate systems
transformer_to_utm = Transformer.from_crs("epsg:4326", "epsg:32632")     # WGS84 to UTM zone 32N
transformer_to_latlon = Transformer.from_crs("epsg:32632", "epsg:4326")  # UTM zone 32N to WGS84

# latitude and longitude coordinates(WGS84), Example: Zurich
latitude = 47.3769
longitude = 8.5417
lonlat = (latitude, longitude)

distance = random.uniform(0, 0.3)  # Random distance for each step(m)

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

####################################################################################################

animal_movements = {}
lock = threading.Lock()

def generate_animals_threading(animal_id, num_points, origin_xy, regularity, client):
    """
    Generates animal movement data and sends it to the client.
    """
    movements = []
    xy = latlon_to_utm(lonlat)
    (x, y) = origin_xy[0]
    x = x + xy[0][0]
    y = y + xy[0][1]
    movements.append((x, y))
    
    # Initialize the previous direction angle to a random direction
    previous_theta = random.uniform(0, 2 * np.pi)
    steps_since_last_change = 0
    total_turn_angle = 0

    for _ in range(num_points):
        distance = random.uniform(0, 0.3)
        
        # If the cumulative angle exceeds 90 degrees, the direction is maintained
        if abs(total_turn_angle) < np.pi / 2:
            theta_change = random.uniform(-np.pi / 12, np.pi / 12) # Allow up to ±15 degrees of rotation per step
            total_turn_angle += theta_change
        else:
            theta_change = 0 # When the accumulated turning angle exceeds 90 degrees, there will be no turning
            
        theta = previous_theta + theta_change
        x += distance * np.cos(theta)
        y += distance * np.sin(theta)
        movements.append((x, y))
        
        # Update the previous direction angle
        previous_theta = theta
        
        # The accumulated angle is reset every 6 steps
        steps_since_last_change += 1
        if steps_since_last_change >= 6:
            steps_since_last_change = 0
            total_turn_angle = 0

        with lock:
            lon, lat = transformer_to_latlon.transform(x, y)
            animal_movements[animal_id] = (animal_id, lon, lat, x, y)
            print(f"Sending position for animal {animal_id}: lon={lon}, lat={lat}")
            data = json.dumps({'animal_id': animal_id, 'longitude': lon, 'latitude': lat})
            try:
                client.sendall(data.encode('utf-8') + b'\n')
            except socket.error as e:
                print(f"Error sending data: {e}")
                break

        time.sleep(regularity)

def start_movement_server():
    """
    Starts the movement server and listens for client connections.
    """
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        server.bind(('0.0.0.0', 5001))
        server.listen(5)
        print("Server listening on port 5001")
    except socket.error as e:
        print(f"Server error: {e}")
        return
        
    try:
        client, addr = server.accept()
        print(f"Connection from {addr}")
    except socket.error as e:
        print(f"Error accepting connection: {e}")
        return

    # try:
    #     data = conn.recv(1024).decode('utf-8')
    # except data as data:
    #     print(f"Error decoding: {data}")
    #     return

    while True:
        try:
            data = client.recv(1024).decode('utf-8')
            if not data:
                break
            request_data = json.loads(data)
            num_animals = request_data['num_animals']
            num_points = request_data['num_points']
            origin_xy = request_data['origin_xy']
            regularity = request_data.get('regularity', 0.1)
            for animal_id in range(num_animals):
                threading.Thread(target=generate_animals_threading, args=(animal_id, num_points, origin_xy, regularity, client)).start()
        except json.JSIONDecoderError as e:
            print(f"Error decoding JSON data: {e}")
        except socket.error as e:
            print(f"Error receiving data: {e}")
            break

    client.close()

if __name__ == '__main__':
    start_movement_server()