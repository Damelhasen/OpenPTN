"""Author : Johan Sheby
   Date : 25/09/2025
   Project :  AlphaOS
"""
## Importing the required libraries##
import os
from datetime import datetime

from rtlsdr import RtlSdr as sdr

import numpy as np
import serial
import struct
import time

from pymavlink import mavutil

###INIT###
sdr = sdr()
serial_port = serial.Serial('/dev/matek_fc', 115200, timeout=0.1)
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
    print(f"Radio Signal Strength: {Radio_Db:.2f} dB IIIIIIIIII \nFrequency: {freq:.2f} MHz\n GPS Data: {mavlink_data}\nSamples Collected: {len(samples)}")
    time.sleep(0.5)  # Pause for half a second to allow the user to read the output
    clear_terminal()

def main():
    """Main function to run the AlphaOS dashboard."""
    master = mavutil.mavlink_connection('/dev/matek_fc', baud=115200)
    while True:
        freq = 433.92  # Example frequency in MHz
        samples = listen_freq(sdr, freq)
        Radio_Db = 10 * np.log10(np.mean(np.abs(samples)**2))  # Calculate signal strength in dB
        mavlink_data = get_mavlink_data(master)
        dashboard(Radio_Db, mavlink_data, freq, samples)