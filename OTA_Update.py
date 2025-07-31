#!/usr/bin/env python3
import argparse
import glob
import os
import subprocess
import sys
import socket # Use the standard socket library for name resolution

# --- Configuration ---
PLATFORMIO_BUILD_DIR = ".pio/build"
# The full mDNS hostname to resolve
MDNS_HOSTNAME = "esp32-ota.local" 

def find_firmware_file(path_arg):
    """Find the project's firmware .bin file for PlatformIO."""
    if path_arg:
        if not os.path.exists(path_arg):
            print(f"❌ Error: Firmware file not found at '{path_arg}'")
            sys.exit(1)
        return path_arg

    build_dir = os.path.join(os.getcwd(), PLATFORMIO_BUILD_DIR)
    if not os.path.isdir(build_dir):
        print(f"❌ Error: PlatformIO build directory '{PLATFORMIO_BUILD_DIR}' not found.")
        sys.exit(1)

    firmware_files = glob.glob(f"{build_dir}/*/firmware.bin")

    if len(firmware_files) == 1:
        firmware_path = firmware_files[0]
        env_name = os.path.basename(os.path.dirname(firmware_path))
        print(f"✅ Found firmware for environment '{env_name}': {firmware_path}")
        return firmware_path
    
    if len(firmware_files) > 1:
        print(f"❌ Error: Found {len(firmware_files)} 'firmware.bin' files.")
        print("   Please specify the correct one using the --file argument.")
        sys.exit(1)

    print(f"❌ Error: Could not find 'firmware.bin' in any environment within '{build_dir}'.")
    sys.exit(1)


def discover_device_ip(ip_arg):
    """Discover the device IP using the system's resolver."""
    if ip_arg:
        print(f"✅ Using specified IP address: {ip_arg}")
        return ip_arg
    
    print(f"👀 Resolving hostname '{MDNS_HOSTNAME}' using system resolver...")
    try:
        # Use the OS to resolve the .local address
        ip_address = socket.gethostbyname(MDNS_HOSTNAME)
        print(f"✅ Found device at IP: {ip_address}")
        return ip_address
    except socket.gaierror:
        print(f"❌ Error: Could not resolve hostname '{MDNS_HOSTNAME}'.")
        print("   Is the device on and connected to the same network?")
        print("   You can still try updating using the --ip flag.")
        sys.exit(1)

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
        print("\n❌ Error: 'curl' command not found. Please ensure it's installed.")
    except Exception as e:
        print(f"\n❌ An unexpected error occurred: {e}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="ESP32 OTA Update Script")
    parser.add_argument("--file", help="Path to the firmware .bin file")
    parser.add_argument("--ip", help="IP address of the ESP32 (bypasses mDNS)")
    args = parser.parse_args()
    
    target_ip = discover_device_ip(args.ip)
    firmware_path = find_firmware_file(args.file)
    
    run_ota_update(target_ip, firmware_path)