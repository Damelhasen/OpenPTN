"""Author : Johan Sheby
   Date : 25/09/2025
   Project :  AlphaOS
"""
## Importing the required libraries##
from doctest import master
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
sdr = sdr()
MAVLINK_PORT = os.getenv("MAVLINK_PORT", "COM9" if os.name == "nt" else "/dev/matek_fc")
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

def set_armed(master, should_arm: bool) -> None:
    """
    Request the motors to be armed or disarmed.
    """
    if master.motors_armed() == should_arm:
        return

    if should_arm:
        master.arducopter_arm()
    else:
        master.arducopter_disarm()

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


def dashboard(Radio_Db, mavlink_data, freq, armed):
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
    print(f"Radio strength: {Radio_Db:.2f} dB\nFrequency: {freq:.2f} MHz")
    print(f"Motors: {'ARMED' if armed else 'DISARMED'}")
    if mavlink_data is None:
        print("Navigation data: unavailable")
    else:
        print(
            f"GPS valid: {mavlink_data['valid']}\n"
            f"Satellites: {mavlink_data['sats_visible']}\n"
            f"Latitude: {mavlink_data['lat']:.7f}\n"
            f"Longitude: {mavlink_data['lon']:.7f}\n"
            f"Altitude: {mavlink_data['alt']:.1f} m\n"
            f"Speed: {mavlink_data['speed']:.2f} m/s"
        )
    time.sleep(0.5)  # Pause for half a second to allow the user to read the output
    

def main():
    """Main function to run the AlphaOS dashboard."""
    master = mavutil.mavlink_connection(MAVLINK_PORT, baud=115200)
    master.wait_heartbeat()
    with open("log.csv", mode= "w") as log:
        log.write("long, lat, intensity\n")

    freq_1 = float(input("Enter frequency 1 in MHz: "))
    while True:
        samples = listen_freq(sdr, freq_1)
        Radio_Db = 10 * np.log10(np.mean(np.abs(samples)**2))  # Calculate signal strength in dB
        mavlink_data = get_mavlink_data(master)
        dashboard(Radio_Db, mavlink_data, freq_1, master.motors_armed())
if __name__ == "__main__":
    main()