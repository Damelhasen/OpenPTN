"""Author : Johan Sheby
   Date : 25/09/2025
   Project :  AlphaOS
"""
## Importing the required libraries##
import os
from datetime import datetime

from rtlsdr import RtlSdr as sdr
import numpy as np
import time
from pymavlink import mavutil
import folium 
from folium.plugins import HeatMap
import pandas as pd

###INIT###
MAVLINK_PORT = os.getenv(
    "MAVLINK_PORT",
    "COM9" if os.name == "nt" else "/dev/ttyACM0"
)
###Functions###
def clear_terminal():
    """Clears the terminal screen."""
    os.system('cls' if os.name == 'nt' else 'clear')

def write(file_path,data) :
    "writes data into a file"
    with open(file_path, "w", encoding ="utf-8") as file :
        file.write(data)

def listen_freq(sdr_device, freq: float):
    """
    Function to listen to a specific frequency using the selected SDR device.

    Parameters:
    sdr_device: The SDR device instance
    freq (float): The frequency to listen to in   Mhz
    taken from "https://github.com/Damelhasen/SDR_SCANNER" 
    """
    
    sdr_device.center_freq = (freq*10**6)


    samples = sdr_device.read_samples(256*1024)

    return samples

def get_mavlink_data(master,timeout=1):
    """
    Function  to read MAVLink from FC serial port """
    global msg
    msg = master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=timeout)
    if msg is None:
        return None
    valid = msg.fix_type >= 3 and msg.satellites_visible >= 6
    return {
        "valid": valid,
        "sats_visible": msg.satellites_visible,
        "lat": msg.lat / 1e7,
        "lon": msg.lon / 1e7,
        "alt": msg.alt / 1e3,
        "speed": msg.vel / 100,
    }



def create_map (map_file:str) :
    datafile = pd.read_csv(map_file)
    datafile = datafile.dropna(subset=["lat","long","intensity"])
    heat_data = datafile[["lat","long","intensity"]].values.tolist()
    map_center = [datafile["lat"].mean(), datafile["long"].mean()]
    visual_map = folium.Map(location=map_center,zoom_start=9,tiles="openstreetmap")

    folium.TileLayer(
        tiles="https://server.arcgisonline.com/ArcGIS/rest/services/Canvas/World_Light_Gray_Base/MapServer/tile/{z}/{y}/{x}",
        attr="Esri, HERE, Garmin",
        name="Esri Light Gray",
    ).add_to(visual_map)
    
    HeatMap(
        data=heat_data,
        radius = 15,
        blur = 10,
        max = float(datafile["intensity"].max())
        ).add_to(visual_map)

    visual_map.save("heatmap.html")


def dashboard(Radio_Db, mavlink_data, freq, samples):
    """
    Function to display the dashboard with radio and MAVLink data.
    """
    clear_terminal()
    print("""                                                 
  ▄▄▄▄   ▄▄       ▄▄            ▄▄▄▄▄    ▄▄▄▄▄▄▄ 
▄██▀▀██▄ ██       ██          ▄███████▄ █████▀▀▀ 
███  ███ ██ ████▄ ████▄  ▀▀█▄ ███   ███  ▀████▄  
███▀▀███ ██ ██ ██ ██ ██ ▄█▀██ ███▄▄▄███    ▀████ 
███  ███ ██ ████▀ ██ ██ ▀█▄██  ▀█████▀  ███████▀ 
            ██                                   
            ▀▀                                   """)
    satellites = mavlink_data["sats_visible"] if mavlink_data else "unavailable"
    print(f"Radio Signal Strength: {Radio_Db:.2f} dB\nFrequency: {freq:.2f} MHz\nGPS Data: {mavlink_data or 'unavailable'}\nSatellites: {satellites}\nSamples Collected: {len(samples)}")
    time.sleep(0.5)  # Pause for half a second to allow the user to read the output
    

def main():
    """Main function to run the AlphaOS dashboard."""
    master = mavutil.mavlink_connection(MAVLINK_PORT, baud=115200)
    master.wait_heartbeat()
    with open("log.csv", mode="w", newline="") as log:
        log.write("lat,long,intensity\n")
    sdr_device = sdr()
    while master.motors_armed():
        freq_1 = float(input("Enter frequency 1 in MHz: ")) 
        samples = listen_freq(sdr_device, freq_1)
        Radio_Db = 10 * np.log10(np.mean(np.abs(samples)**2))  # Calculate signal strength in dB
        mavlink_data = get_mavlink_data(master)
        dashboard(Radio_Db, mavlink_data, freq_1, samples)
        if mavlink_data and mavlink_data["valid"]:
            with open("log.csv", mode="a", newline="") as log:
                log.write(f"{mavlink_data['lat']},{mavlink_data['lon']},{Radio_Db}\n")

if __name__ == "__main__":
    main()
