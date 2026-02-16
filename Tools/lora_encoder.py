#!/usr/bin/env python3
"""
SINGLE-WAV LoRa ENCODER (educational)

This version writes **only one WAV file** (no per-symbol files).
By default it writes a mono real part WAV (easy to inspect in Audacity)
so you immediately see the chirp spectrogram. If you prefer a proper
I/Q file for SDR playback, change EXPORT_MODE to 'iq'.

Run:
    python lora_encoder.py

Edit parameters at the top as needed.
"""

import numpy as np
from scipy.io import wavfile
from scipy.signal import resample_poly
import os
import math

# ------------------ User parameters ------------------
INPUT_BYTE = 0xA3      # demo byte
SF = 8                 # spreading factor
CR = '4/8'             # coding rate (we implement 4/8 path)
BW = 125_000           # Hz (sample-rate used for baseband)

# LFSR seed for whitening (example). Replace with chip seed for bit-exact HW.
LFSR_INIT = [0,0,0,0,0,0,0,0,1]

# Export mode: 'real' = mono real part (best for Audacity spectrogram)
#              'iq'  = stereo I/Q (left=I right=Q) - good for SDR replay
EXPORT_MODE = 'iq'
OUT_DIR = 'chirps_wav'
OUT_PREFIX = f'byte_{INPUT_BYTE:02X}'

# ------------------ Utilities ------------------
def byte_to_bits_lsb_first(byte):
    return [(byte >> i) & 1 for i in range(8)]

def bits_to_int_lsb_first(bits):
    return sum(bit << i for i, bit in enumerate(bits))

# ------------------ Whitening (9-bit LFSR x^9+x^5+1) ------------------
def whiten_bits(bits, lfsr_init):
    lfsr = lfsr_init[:]
    out = []
    for b in bits:
        out_bit = b ^ lfsr[8]
        out.append(out_bit)
        feedback = lfsr[8] ^ lfsr[4]
        for i in range(8, 0, -1):
            lfsr[i] = lfsr[i-1]
        lfsr[0] = feedback
    return out

# ------------------ Hamming (4->8) ------------------
def hamming_4_8(nibble):
    d0, d1, d2, d3 = nibble
    p0 = d0 ^ d1 ^ d2
    p1 = d1 ^ d2 ^ d3
    p2 = d0 ^ d1 ^ d3
    p3 = d0 ^ d2 ^ d3
    return [d0, d1, d2, d3, p0, p1, p2, p3]

# ------------------ Padding ------------------
def pad_codewords(codewords, sf):
    while len(codewords) < sf:
        codewords.append([0]*len(codewords[0]))
    return codewords

# ------------------ Diagonal interleaver ------------------
def interleave(codewords, sf):
    rows = len(codewords[0])
    cols = sf
    D = [[0]*cols for _ in range(rows)]
    for c, cw in enumerate(codewords):
        for r, bit in enumerate(cw):
            D[r][c] = bit
    I = [[0]*cols for _ in range(rows)]
    for i in range(rows):
        for j in range(cols):
            src_col = (i - (j + 1)) % sf
            I[i][j] = D[j][src_col]
    return D, I

# ------------------ LoRa Gray (modified) ------------------
def lora_gray(v, sf):
    M = 1 << sf
    v = (v - 1) % M
    return v ^ (v >> 1)

# ------------------ Chirp generation (base up-chirp) ------------------
def generate_base_chirp(sf, bw):
    N = 1 << sf
    Ts = N / bw
    t = np.arange(N) / bw
    f0 = -bw / 2.0
    k = bw / Ts
    phase = 2 * np.pi * (f0*t + 0.5 * k * t**2)
    return np.exp(1j * phase)

def generate_symbol(base_chirp, symbol):
    return np.roll(base_chirp, symbol)

# ------------------ Single-WAV exporter ------------------
def save_single_wav(chirps, sample_rate, mode='real', prefix='lora'):
    """Save exactly one WAV file.
    mode='real' -> mono real part (Re(I+jQ)) which Audacity spectrogram shows nicely
    mode='iq'  -> stereo I/Q (left=I right=Q) for SDR playback
    """
    os.makedirs(OUT_DIR, exist_ok=True)
    full = np.concatenate(chirps)

    if mode == 'iq':
        arr = np.vstack([np.real(full), np.imag(full)]).T.astype(np.float64)
        # normalize
        m = np.max(np.abs(arr)) or 1.0
        arr = arr / m
        arr_i16 = (arr * 32767.0).astype(np.int16)
        fname = os.path.join(OUT_DIR, f"{prefix}_packet_iq.wav")
        wavfile.write(fname, sample_rate, arr_i16)
        print(f"Wrote I/Q WAV: {fname} (stereo, I=left,Q=right, sr={sample_rate})")
        return fname

    # default: real part mono
    real_part = np.real(full).astype(np.float64)
    # normalize
    m = np.max(np.abs(real_part)) or 1.0
    real_part = real_part / m
    # write at same sample rate (BW) so spectrogram correctly shows BW
    real_i16 = (real_part * 32767.0).astype(np.int16)
    fname = os.path.join(OUT_DIR, f"{prefix}_packet_real.wav")
    wavfile.write(fname, sample_rate, real_i16)
    print(f"Wrote real part WAV: {fname} (mono, sr={sample_rate})")
    return fname

# ------------------ Main pipeline ------------------
def main():
    print("=== LoRa ENCODER (single-WAV) ===\n")
    bits = byte_to_bits_lsb_first(INPUT_BYTE)
    print(f"Input byte 0x{INPUT_BYTE:02X} -> bits LSB-first: {bits}")

    whitened = whiten_bits(bits, LFSR_INIT)
    wnibbles = [whitened[0:4], whitened[4:8]]
    print(f"Whitened nibbles: {wnibbles}")

    codewords = [hamming_4_8(n) for n in wnibbles]
    codewords = pad_codewords(codewords, SF)

    D, I = interleave(codewords, SF)
    print("Interleaved I matrix (rows -> symbol bits, LSB-first):")
    for r in I:
        print(''.join(str(b) for b in r))

    symbols = [bits_to_int_lsb_first(row) for row in I]
    gray = [lora_gray(v, SF) for v in symbols]
    print("Symbol ints:", symbols)
    print("Gray ints:", gray)

    base = generate_base_chirp(SF, BW)
    chirps = [generate_symbol(base, g) for g in gray]

    # Save exactly one WAV according to EXPORT_MODE
    outfile = save_single_wav(chirps, sample_rate=BW, mode=EXPORT_MODE, prefix=OUT_PREFIX)
    print(f"\nSaved single WAV: {outfile}")
    print("=== DONE ===")

if __name__ == '__main__':
    main()