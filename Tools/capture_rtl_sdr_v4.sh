#!/usr/bin/env bash
# ══════════════════════════════════════════════════════════════════════════════
#  capture_rtl_sdr_v4.sh — Capture IQ samples from an RTL-SDR dongle and
#                           convert them to a NumPy file for offline analysis.
# ══════════════════════════════════════════════════════════════════════════════
#
# ─── WHAT THIS SCRIPT DOES ───────────────────────────────────────────────────
#   1. Uses the `rtl_sdr` command-line tool to record raw IQ (In-phase /
#      Quadrature) samples from an RTL-SDR USB dongle.
#   2. Converts the raw unsigned-8-bit interleaved IQ file into a NumPy
#      complex64 array (.npy) that Python scripts can load directly.
#
# ─── WHAT IS RTL-SDR? ────────────────────────────────────────────────────────
#   RTL-SDR is an extremely cheap (<$30) software-defined radio receiver built
#   around the Realtek RTL2832U DVB-T TV tuner chip. When paired with an
#   appropriate tuner front-end (e.g. Rafael Micro R820T2 or R828D), the
#   dongle can receive radio signals roughly from 24 MHz to 1.766 GHz, which
#   covers FM radio, ADS-B aircraft transponders, ISM-band IoT devices, and —
#   crucially for this project — sub-GHz LoRa transmissions (868 / 915 MHz).
#
#   Popular hardware variants (all plug into a USB-A port):
#     • RTL-SDR Blog V3 / V4  — the most common and well-supported dongle.
#       V4 uses the R828D tuner and adds an HF direct-sampling capability.
#       Purchase: https://www.rtl-sdr.com/buy-rtl-sdr-dvb-t-dongles/
#       AliExpress official store: "RTLSDRBlog Store" (significantly cheaper).
#     • Nooelec NESDR Smart (v5) — another popular option with a TCXO for
#       better frequency stability (lower ppm drift).
#     • Generic DVB-T dongles with RTL2832U + R820T2 — cheapest, but often
#       have worse frequency accuracy and thermal drift.
#
#   What you need physically:
#     • One RTL-SDR dongle (USB stick).
#     • An antenna tuned for the frequency of interest. For 868 MHz LoRa the
#       small telescopic whip that ships with the RTL-SDR Blog kit works, but
#       a quarter-wave ground-plane antenna cut to ~8.2 cm gives better results.
#     • A USB extension cable can help move the dongle away from your PC to
#       reduce USB/CPU noise picked up by the front-end.
#
# ─── INSTALLING rtl_sdr (librtlsdr) ──────────────────────────────────────────
#   `rtl_sdr` is the command-line capture utility provided by the librtlsdr
#   (a.k.a. rtl-sdr) open-source project: https://github.com/osmocom/rtl-sdr
#
#   Debian / Ubuntu / Raspberry Pi OS:
#       sudo apt update
#       sudo apt install rtl-sdr librtlsdr-dev
#
#   Fedora / RHEL:
#       sudo dnf install rtl-sdr rtl-sdr-devel
#
#   Arch Linux:
#       sudo pacman -S rtl-sdr
#
#   macOS (Homebrew):
#       brew install librtlsdr
#
#   Build from source (any Linux):
#       sudo apt install git cmake build-essential libusb-1.0-0-dev  # deps
#       git clone https://github.com/osmocom/rtl-sdr.git
#       cd rtl-sdr && mkdir build && cd build
#       cmake .. -DINSTALL_UDEV_RULES=ON
#       make -j$(nproc) && sudo make install && sudo ldconfig
#
#   After installation, verify with:
#       rtl_test -t          # should detect the dongle and print tuner type
#
#   IMPORTANT — Linux udev permissions:
#     By default, only root can access USB devices. To allow your normal user
#     account to use the dongle, install the udev rules that ship with
#     librtlsdr (the `-DINSTALL_UDEV_RULES=ON` flag above does this).
#     If you installed via apt, the rules are placed automatically. Then:
#       sudo udevadm control --reload-rules
#       sudo udevadm trigger
#     Unplug and re-plug the dongle after this step.
#
#   IMPORTANT — Blacklisting the DVB-T kernel driver:
#     Linux may auto-load the `dvb_usb_rtl28xxu` kernel module, which claims
#     the device for TV reception and prevents rtl_sdr from opening it.
#     Create a blacklist file:
#       echo 'blacklist dvb_usb_rtl28xxu' | sudo tee /etc/modprobe.d/rtlsdr-blacklist.conf
#     Then reboot or manually unload:
#       sudo rmmod dvb_usb_rtl28xxu
#
# ─── USAGE ────────────────────────────────────────────────────────────────────
#   1. Plug in your RTL-SDR dongle.
#   2. Run:  bash capture_rtl_sdr_v4.sh
#   3. Wait for the capture to finish (default: 10 seconds).
#   4. Analyse the result with the companion Python script:
#        python3 lora_symbol_observer.py
#      Make sure MODE = 'file' and IQ_FILE = 'last_capture.npy' are set in
#      that script (see Tools/lora_symbol_observer.py).
#
# ══════════════════════════════════════════════════════════════════════════════

# Exit immediately if any command fails (prevents silent errors).
set -e

# ─── CAPTURE PARAMETERS ──────────────────────────────────────────────────────

# Centre frequency in Hz to tune the RTL-SDR to.
# We deliberately tune 200 kHz ABOVE the actual LoRa carrier (869.525 MHz)
# to avoid the RTL-SDR's "DC spike" — a hardware artefact that creates a
# spurious signal right at the tuned centre frequency. By tuning to
# 869.725 MHz the LoRa signal appears at −200 kHz in the captured baseband
# spectrum, safely away from the DC spike. The analysis script compensates
# for this shift via its F_OFFSET = −200 000 Hz setting.
FREQ=869725000

# Sample rate in samples per second. 2.4 MS/s is a good trade-off: it is
# well within the RTL2832U's stable range (≤ 2.56 MS/s without dropped
# samples on most hosts) and gives us enough bandwidth to comfortably
# contain a 125 kHz LoRa channel plus the 200 kHz DC-avoidance offset.
RATE=2400000

# Duration of the capture in seconds. 10 s is usually enough to catch
# several LoRa packets (a single SF8/BW125 packet is only ~50–200 ms
# depending on payload length).
SECONDS=10

# Total number of IQ sample *pairs* to record. rtl_sdr's -n flag counts
# individual bytes (each sample pair = 2 bytes: one I, one Q), but the
# tool internally interprets -n as the number of *bytes* to read, so
# SAMPLES here equals RATE * SECONDS (number of complex samples), and
# rtl_sdr will actually read 2× that many bytes from the device.
SAMPLES=$((RATE * SECONDS))

# Output file names.
OUTPUT_IQ="last_capture.iq"    # Raw unsigned-8-bit interleaved IQ dump
OUTPUT_NPY="last_capture.npy"  # Converted NumPy complex64 array

echo "Recording ${SECONDS}s (${SAMPLES} samples) at ${FREQ} Hz → ${OUTPUT_IQ}"

# ─── STEP 1: RAW IQ CAPTURE ──────────────────────────────────────────────────
# rtl_sdr flags:
#   -f  Centre frequency in Hz
#   -s  Sample rate in Hz
#   -g  Tuner gain in dB.  0 = automatic gain control (AGC), which lets the
#       dongle's internal amplifier adjust itself. For weak signals you can
#       try explicit gain values (e.g. 40) — list supported values with
#       `rtl_test -t`.
#   -n  Number of samples to read, then exit cleanly (without this flag
#       rtl_sdr would record indefinitely until Ctrl-C).
#
# The output file contains raw bytes: I0 Q0 I1 Q1 I2 Q2 ...
# Each I/Q value is an unsigned 8-bit integer (0–255) where 127.5 represents
# zero — this is the native format of the RTL2832U ADC.
rtl_sdr -f ${FREQ} -s ${RATE} -g 0 -n ${SAMPLES} "${OUTPUT_IQ}"

echo "Capture finished. Converting ${OUTPUT_IQ} → ${OUTPUT_NPY} ..."

# ─── STEP 2: CONVERT RAW IQ → NUMPY ──────────────────────────────────────────
# The embedded Python script below performs the format conversion:
#   1. Read the raw bytes as a flat uint8 array.
#   2. Convert to float32 and centre around zero by subtracting 127.5, then
#      normalise to the range [−1, +1] by dividing by 127.5.
#   3. Deinterleave: even indices → I (real), odd indices → Q (imaginary).
#      Combine into a complex64 array (I + jQ).
#   4. Save as a .npy file that any Python/NumPy script can load with
#      np.load().
python3 << EOF
import numpy as np

iq_file  = "${OUTPUT_IQ}"
out_file = "${OUTPUT_NPY}"

print(f"Loading {iq_file} ...")
# Read every byte of the raw capture into a 1-D uint8 array.
raw = np.fromfile(iq_file, dtype=np.uint8)

# ── Normalisation ──
# The RTL2832U ADC outputs unsigned 8-bit values (0–255).
# Subtract the mid-point (127.5) so that the signal is centred on zero,
# then divide by 127.5 to scale into the [−1, +1] range as float32.
iq = (raw.astype(np.float32) - 127.5) / 127.5

# ── Deinterleave into complex samples ──
# Raw layout: [I0, Q0, I1, Q1, I2, Q2, ...]
# Even indices (0, 2, 4, …) are the In-phase (real) component.
# Odd  indices (1, 3, 5, …) are the Quadrature (imaginary) component.
iq = iq[0::2] + 1j * iq[1::2]

print(f"Saving {out_file} ({len(iq)} complex samples)")
np.save(out_file, iq)
EOF

# ─── DONE ─────────────────────────────────────────────────────────────────────
echo
echo "Capture and conversion complete."
echo "Next step:"
echo "  python3 lora_symbol_observer.py"
echo
echo "Ensure MODE = 'file' and IQ_FILE = '${OUTPUT_NPY}' in the script."