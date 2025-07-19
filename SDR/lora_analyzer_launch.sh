#!/bin/bash

# Script Name: gr-lora_setup.sh
# Description: This script automates the setup and execution of gr-lora for LoRa signal analysis
#              with GNU Radio. It conditionally installs necessary packages and builds
#              gr-lora and its dependencies only if they are not already present,
#              to avoid redundant installations.
# Usage:       Run this script from the current directory.
#
# How to get a signal recording for the analysis:
#    1. Set up your LoRa transceiver to actually transmit some LoRa packets.
#    2. Use a compatible SDR (Software Defined Radio) device (in my case RTL-SDR V4) to record the signal.
#       - For actual recording I use SDR++ software with the following settings:
#         - Sample Rate: 2.4 MHz
#         - Center Frequency: [Your LoRa Frequency]
#         - Record Format: WAV
#         - Sample Type: Float32
#    3. Run this very script to open up the GNU Radio Companion flowgraph and replace the
#       .wav file in the Wav File Source block with your recorded file and adjust the settings
#       as needed (especially your LoRa parameters in the LoRa Receiver block).
#    4. Run the flowgraph. You should see:
#       - The LoRa packets being decoded and printed in the console.
#       - The waveform visualized by the Qt GUI block.
#
# --- Configuration ---
# Set the name of your GNU Radio Companion flowgraph file
FLOWGRAPH_FILE="LoRa_flowchart.grc"

# --- Functions for Conditional Installation ---

# Check if a command exists
command_exists () {
    type "$1" &> /dev/null ;
}

# Check if a package is installed (for apt)
package_installed_apt() {
    dpkg -s "$1" &> /dev/null
}

# --- Main Script Logic ---

echo "Starting gr-lora setup and launch script..."

# 1. Update package lists
echo "Updating apt package lists..."
sudo apt update || { echo "Error: apt update failed. Exiting."; exit 1; }

# 2. Install GNU Radio (if not already installed)
if ! command_exists gnuradio-companion; then
    echo "GNU Radio not found. Installing GNU Radio..."
    sudo apt install -y gnuradio || { echo "Error: GNU Radio installation failed. Exiting."; exit 1; }
else
    echo "GNU Radio is already installed."
fi

# 3. Install gr-lora dependencies (if not already installed)
REQUIRED_DEPS="liblog4cpp5-dev git build-essential autoconf libtool"
echo "Checking for gr-lora dependencies..."
for dep in $REQUIRED_DEPS; do
    if ! package_installed_apt "$dep"; then
        echo "Installing dependency: $dep"
        sudo apt install -y "$dep" || { echo "Error: Failed to install $dep. Exiting."; exit 1; }
    else
        echo "Dependency $dep is already installed."
    fi
done

# 4. Install liquid-dsp (gr-lora's dependency)
# Check if liquid-dsp is built/installed by looking for a key header/library
# This check is heuristic; a more robust check might involve checking /usr/local/lib or /usr/local/include
if [ ! -f /usr/local/lib/libliquid.so ] || [ ! -f /usr/local/include/liquid/liquid.h ]; then
    echo "liquid-dsp not found or not built. Building and installing liquid-dsp..."
    if [ -d "liquid-dsp" ]; then
        (cd liquid-dsp && \
        ./bootstrap.sh && \
        ./configure && \
        make -j$(nproc) && \
        sudo make install && \
        sudo ldconfig) || { echo "Error: liquid-dsp build/install failed. Exiting."; exit 1; }
    else
        echo "Error: liquid-dsp directory not found. Please clone gr-lora repository correctly."
        exit 1
    fi
else
    echo "liquid-dsp appears to be installed."
fi

# 5. Build and Install gr-lora
# Check if gr-lora module is present in GNU Radio's search path
# This is a basic check; a full check would involve looking for compiled modules
if ! gnuradio-config-info --enabled-blocks | grep -q "lora"; then
    echo "gr-lora not found or not built. Building and installing gr-lora..."
    if [ -d "gr-lora" ]; then
        (cd gr-lora && \
        mkdir -p build && \
        cd build && \
        cmake .. && \
        make -j$(nproc) && \
        sudo make install && \
        sudo ldconfig) || { echo "Error: gr-lora build/install failed. Exiting."; exit 1; }
    else
        echo "Error: gr-lora directory not found. Please clone gr-lora repository correctly."
        exit 1
    fi
else
    echo "gr-lora appears to be installed."
fi

# 6. Launch GNU Radio Companion
echo "Launching GNU Radio Companion with $FLOWGRAPH_FILE..."
if [ -f "$FLOWGRAPH_FILE" ]; then
    gnuradio-companion "$FLOWGRAPH_FILE" &
    echo "GNU Radio Companion launched. Check your desktop for the window."
else
    echo "Error: GNU Radio Companion flowgraph '$FLOWGRAPH_FILE' not found in the current directory."
    echo "Please ensure the script is run from the directory containing your flowgraph."
fi

echo "Script finished."