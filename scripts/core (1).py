"""Author : Johan Sheby
   Date : 23/06/2025
   Project : Multipurpose SDR CLI Toolkit
"""
######IMPORTS#####
import os
from time import sleep, time
from datetime import datetime

try:
    from rtlsdr import RtlSdr as sdr
except Exception:
    sdr = None
import numpy as np
import math 
import json
import matplotlib.pyplot as plt
from dataclasses import dataclass

#####INITS########


####FUNCTIONS#####


def clear_terminal():
    """Clear the terminal screen"""
    os.system('cls' if os.name == 'nt' else 'clear')
    

def write(file_path:str , write_material) :
    with open(file_path, "w", encoding="utf-8") as file:
        file.write(write_material)


def listen_freq(sdr_device, freq: float):
    """
    Function to listen to a specific frequency using the selected SDR device.

    Parameters:
    sdr_device: The SDR device instance
    freq (float): The frequency to listen to in   Mhz
    """
    
    sdr_device.center_freq = (freq*10**6)


    samples = sdr_device.read_samples(256*1024)

    return samples
   

def scanner(sdr_device, freq_start: float, freq_end: float, step: float , step_time:float , peak_check : bool):
    """
    Function to scan a range of frequencies using selected SDR device """
    
    freq_start = float(freq_start) 
    freq_end = float(freq_end)

    @dataclass
    class ScanResults:
        full_range: dict
        peak_freq: float
        peak_power: float

    results = {}

    sdr_device.center_freq = (freq_start*10**6)

    # Calculate decimal precision based on step size
    decimal_places = len(str(step).split('.')[-1]) if '.' in str(step) else 0
    
    for freq in np.arange(freq_start, freq_end+step, step):
        freq = round(freq, decimal_places + 1)
        sleep(step_time)
        samples = sdr_device.read_samples(256*1024)
        power = np.mean(np.abs(samples) ** 2)
        db = 10 * np.log10(power + 1e-10)
        results[freq] = db
        
    if peak_check == True and results:
        peak_freq = max(results, key=results.get)
        peak_power = results[peak_freq]
    else:
        peak_freq = None
        peak_power = None
    return ScanResults(results, peak_freq, peak_power)

def plot(data_sourcefile : str) :

 with open(data_sourcefile , 'r') as f :
     data = json.load(f)
     frequencies = sorted([float(freq) for freq in data.keys()])
     db_values = [data[str(f)] for f in frequencies]
     plt.figure(figsize = (12,16))

     plt.plot(frequencies, db_values, 'b-o', linewidth=2, markersize=6)
     plt.xlabel('Frequency (MHz)')
     plt.ylabel('Power (dB)')
     plt.title('SDR Frequency Scan')
     plt.grid(True, alpha=0.3)
     plt.tight_layout()
     plt.show()

def main_menu():
    clear_terminal()
    print("SDR Scanner Main Menu")
    print("1. Custom Scan")
    print("2. Band Scan")
    print("3. Exit")
    choice = input("Select an option: ")
    
    if choice == '1':
        custom_scan()
    elif choice == '2':
        band_scan()
    elif choice == '3':
        print("Exiting...")
        exit()
    else:
        print("Invalid choice. Please try again.")
        sleep(2)
        main_menu()
def custom_scan():
    global sdr_device

    clear_terminal()
    freq_start_selection = float(input("Input start frequency (Mhz) :  "))
    clear_terminal()
    end_freq_selection = float(input("Input End frequency (Mhz) : "))
    clear_terminal()
    freq_step_selection = float(input("Input step frequency (Mhz) : "))
    clear_terminal()
    step_time_selection = float(input("Input step time (Seconds) : "))
    clear_terminal()
    clear_terminal()
    filenameinput = str(input("What Should the result File Called ? : "))
    clear_terminal()
    
    print(f"starting Scan from {freq_start_selection} to {end_freq_selection}")
    scan_result = scanner(sdr_device,freq_start_selection,end_freq_selection, freq_step_selection, step_time_selection,True)
    clear_terminal()
    
    write(f"sdr_scanner/{filenameinput}.json", json.dumps(scan_result.full_range, indent=2))    
    print(f"Result saved to '{filenameinput}.json' ")
    plot(f"sdr_scanner/{filenameinput}.json")
    print(f"Peak Frequency: {scan_result.peak_freq} MHz, Peak Power: {scan_result.peak_power} dB")
def band_scan():
    global sdr_device
    clear_terminal()

    input_band= input("Press Enter to start Band Scan (1.FM, 2.AM, 3.Aviation 4.Maritime) : ")
    
    if input_band == "1" :
        print("Starting FM Band Scan from 88.0 MHz to 108.0 MHz")
        scan_result = scanner(sdr_device, 88.0, 108.0, 0.2, 0.5, True)
        write(f"sdr_scanner/fm_scan_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.json", json.dumps(scan_result.full_range, indent=2))
        print("FM Band Scan Complete ")
    elif input_band == "2" :
        print("Starting AM Band Scan from 530.0 KHz to 1700.0 KHz")
        scan_result = scanner(sdr_device, 530.0, 1700.0, 1.0, 0.5, True)
        write(f"sdr_scanner/am_scan_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.json", json.dumps(scan_result.full_range, indent=2))
        print("AM Band Scan Complete ")
    
    elif input_band == "3" :
        print("Starting Aviation Band Scan from 118.0 MHz to 137.0 MHz")
        scan_result = scanner(sdr_device, 118.0, 137.0, 0.2, 0.5, True)
        write(f"sdr_scanner/aviation_scan_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.json", json.dumps(scan_result.full_range, indent=2))
        print("Aviation Band Scan Complete ")

    elif input_band == "4" :
        print("Starting Maritime Band Scan from 156.0 MHz to 174.0 MHz")
        scan_result = scanner(sdr_device, 156.0, 174.0, 0.2, 0.5, True)
        write(f"sdr_scanner/maritime_scan_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.json", json.dumps(scan_result.full_range, indent=2))
        print("Maritime Band Scan Complete ")
        
##################
def main():
    global sdr_device

    SDR_SELECTION = input("Select SDR Device (1 for RTL-SDR, 2 for Airspy): ")
    if SDR_SELECTION == '1':
        sdr_device = sdr()
    
    sdr_device.sample_rate = 2.4e6
    sdr_device.center_freq = 98.6e6
    sdr_device.gain = 'auto'
    main_menu()
    clear_terminal()





if __name__ == "__main__":
    main()


        


 