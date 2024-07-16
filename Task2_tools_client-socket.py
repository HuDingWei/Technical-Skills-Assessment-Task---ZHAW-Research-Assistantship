# -*- coding: utf-8 -*-
"""
Created on Tue Jul  9 18:07:13 2024

@author: Hu Ding-Wei
"""
import requests
import websocket
import json
import time
# from threading import Thread
import threading
import socket
import numpy as np
from pyproj import Transformer

#############################################   TASK2-Client   #############################################

# Initialize Transformer for coordinate transformation
transformer_to_utm = Transformer.from_crs("epsg:4326", "epsg:32632")

# Store received animal movement data
animal_movements = {}
lock = threading.Lock()

running = True  # Flag used to control thread exit
def start_movement():
    global running
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client.connect(('127.0.0.1', 5001))
    
    data = {
        "num_animals": 3,
        "num_points": 100,
        "origin_xy": [(0, 0)],
        "regularity": 0.1
    }
    client.sendall(json.dumps(data).encode('utf-8') + b'\n')

    while running:
        try:
            client.settimeout(5.0)  # Set the timeout to 5 second
            response = client.recv(1024).decode('utf-8')
            if not response:
                break
            messages = response.split('\n')
            with lock:
                for message in messages:
                    if message:
                        data = json.loads(message)
                        animal_id = data['animal_id']
                        lon, lat = data['longitude'], data['latitude']
                        x, y = transformer_to_utm.transform(lat, lon)
                        if animal_id not in animal_movements:
                            animal_movements[animal_id] = []
                        animal_movements[animal_id].append((lon, lat, x, y))
                        print(f"Animal ID: {animal_id}, Longitude: {lon}, Latitude: {lat}, UTM X: {x}, UTM Y: {y}")
        except socket.timeout:
            continue  # If it times out, continue looping to check the running flag                
        except socket.error:
            break

    client.close()

def calculate_metrics():
    global running
    while running:
        time.sleep(10)  # Indicators are calculated every 10 seconds
        with lock:
            if animal_movements:
                for animal_id, movements in animal_movements.items():
                    total_distance = 0
                    for i in range(1, len(movements)):
                        prev_point = movements[i-1]
                        curr_point = movements[i]
                        distance = np.sqrt((curr_point[2] - prev_point[2])**2 + (curr_point[3] - prev_point[3])**2)
                        total_distance += distance
                    print(f"Animal ID: {animal_id}, Total distance travelled: {total_distance:.2f} meters")
            else:
                print("No animal movement data available")

if __name__ == "__main__":
    # Start the thread that receives data
    recv_thread = threading.Thread(target=start_movement)
    recv_thread.start()

    # Start the thread for calculating indicators
    metrics_thread = threading.Thread(target=calculate_metrics)
    metrics_thread.start()

    # Wait for 15 seconds to complete data reception
    time.sleep(15)
    
    # Stop thread
    running = False

    # Wait for thread to complete
    recv_thread.join()
    metrics_thread.join()
    print("Process finished")