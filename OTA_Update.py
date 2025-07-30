#!/usr/bin/env python3
import argparse
import glob
import os
import subprocess
import sys
import time
from zeroconf import ServiceBrowser, Zeroconf

# --- Configuration ---
# For PlatformIO, the path is usually .pio/build/<env_name>/firmware.bin
PLATFORMIO_BUILD_DIR = ".pio/build"
MDNS_SERVICE_TYPE = "_http._tcp.local."
MDNS_SERVICE_NAME = "esp32-ota" # Must match mdns_hostname_set() in your C code
MDNS_TIMEOUT_SEC = 5 # Seconds to wait for mDNS discovery

class DeviceListener:
    """Listener for mDNS service discovery."""
    def __init__(self):
        self.device_info = None

    def remove_service(self, zeroconf, type, name):
        pass # Not interested in service removal

    def add_service(self, zeroconf, type, name):
        if MDNS_SERVICE_NAME in name:
            info = zeroconf.get_service_info(type, name)
            if info:
                self.device_info = info
                print(f"✅ Found device '{name}' at IP {self.ip_address}")

    @property
    def ip_address(self):
        if self.device_info:
            return ".".join(map(str, self.device_info.addresses[0]))
        return None

def find_firmware_file(path_arg):
    """Find the project's firmware .bin file for PlatformIO."""
    if path_arg:
        if not os.path.exists(path_arg):
            print(f"❌ Error: Firmware file not found at '{path_arg}'")
            sys.exit(1)
        return path_arg

    # Auto-detect in the PlatformIO build directory
    build_dir = os.path.join(os.getcwd(), PLATFORMIO_BUILD_DIR)
    if not os.path.isdir(build_dir):
        print(f"❌ Error: PlatformIO build directory '{PLATFORMIO_BUILD_DIR}' not found.")
        print("   Please run from your project's root directory or specify a path with --file.")
        sys.exit(1)

    # Search for firmware.bin in any environment subfolder
    firmware_files = glob.glob(f"{build_dir}/*/firmware.bin")

    if len(firmware_files) == 1:
        firmware_path = firmware_files[0]
        # Get the environment name from the path for a nicer message
        env_name = os.path.basename(os.path.dirname(firmware_path))
        print(f"✅ Found firmware for environment '{env_name}': {firmware_path}")
        return firmware_path
    
    if len(firmware_files) > 1:
        print(f"❌ Error: Found {len(firmware_files)} 'firmware.bin' files for different environments.")
        for f in firmware_files:
             print(f"   - {f}")
        print("   Please specify the correct one using the --file argument.")
        sys.exit(1)

    print(f"❌ Error: Could not find 'firmware.bin' in any environment within '{build_dir}'.")
    print("   Have you built the project yet? ('pio run')")
    sys.exit(1)


def discover_device_ip(ip_arg):
    """Discover the device IP using mDNS or use the provided one."""
    if ip_arg:
        print(f"✅ Using specified IP address: {ip_arg}")
        return ip_arg
    
    print(f"👀 Searching for '{MDNS_SERVICE_NAME}' on the network for {MDNS_TIMEOUT_SEC} seconds...")
    zeroconf = Zeroconf()
    listener = DeviceListener()
    browser = ServiceBrowser(zeroconf, MDNS_SERVICE_TYPE, listener)
    
    time.sleep(MDNS_TIMEOUT_SEC)
    browser.cancel()
    zeroconf.close()
    
    if not listener.ip_address:
        print(f"❌ Error: Could not find device with mDNS name '{MDNS_SERVICE_NAME}'.")
        print("   Is it connected to the same network? Try specifying the IP with --ip.")
        sys.exit(1)
    
    return listener.ip_address

def run_ota_update(ip, firmware_file):
    """Executes the curl command to perform the OTA update."""
    print("\n🚀 Starting OTA update...")
    print(f"   Target IP: {ip}")
    print(f"   Firmware:  {firmware_file}")

    command = [
        "curl", "-v", "-X", "POST",
        "--data-binary", f"@{firmware_file}",
        f"http://{ip}/"
    ]
    
    try:
        process = subprocess.Popen(command, stdout=sys.stdout, stderr=sys.stderr)
        process.wait()
        
        if process.returncode == 0:
            print("\n🎉 OTA Update Complete! The device should be rebooting.")
        else:
            print(f"\n❌ OTA Update Failed. curl exited with code {process.returncode}.")
            
    except FileNotFoundError:
        print("\n❌ Error: 'curl' command not found. Please ensure it's installed and in your PATH.")
    except Exception as e:
        print(f"\n❌ An unexpected error occurred: {e}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="ESP32 OTA Update Script for PlatformIO")
    parser.add_argument("--file", help="Path to the firmware .bin file")
    parser.add_argument("--ip", help="IP address of the ESP32 device")
    args = parser.parse_args()
    
    target_ip = discover_device_ip(args.ip)
    firmware_path = find_firmware_file(args.file)
    
    run_ota_update(target_ip, firmware_path)