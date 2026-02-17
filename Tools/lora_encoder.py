#!/usr/bin/env python3
"""
SINGLE-WAV LoRa ENCODER (educational, with accurate preamble and high-rate upsampling)
"""

import numpy as np
from scipy.io import wavfile
from scipy.signal import resample_poly
import os

# User parameters
INPUT_BYTE = 0xA3
SF = 8
CR = '4/8'  # Fixed to 4/8 for this example
BW = 125_000
SAMPLE_RATE = 2_400_000  # Match SDR++ (or set to 3_200_000)
F_OFFSET = 500_000  # Hz offset to shift signal from DC
SYNC_WORD = 0x34  # Common for public/LoRaWAN
EXPORT_MODE = 'iq'  # 'real' or 'iq'
OUT_DIR = 'chirps_wav'
OUT_PREFIX = f'byte_{INPUT_BYTE:02X}_with_preamble_2.4MHz'

# Utilities
def byte_to_bits_lsb_first(byte):
    return [(byte >> i) & 1 for i in range(8)]

def bits_to_int_lsb_first(bits):
    return sum(bit << i for i, bit in enumerate(bits))

# Whitening (9-bit LFSR, described in the Semtech AN1200.18
# https://semtech.my.salesforce.com/sfc/p/#E0000000JelG/a/2R000000HSOn/5O4_iba4ULdi6rfa1q7oENCQ9LGFWGwTjS32Loibdoo
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

# Hamming (4/8 standard)
def hamming_4_8(nibble):
    d0, d1, d2, d3 = nibble
    p0 = d0 ^ d1 ^ d2
    p1 = d1 ^ d2 ^ d3
    p2 = d0 ^ d1 ^ d3
    p3 = d0 ^ d2 ^ d3
    return [d0, d1, d2, d3, p0, p1, p2, p3]

# Padding to SF codewords
def pad_codewords(codewords, sf):
    while len(codewords) < sf:
        codewords.append([0] * len(codewords[0]))
    return codewords

# Diagonal interleaver
def interleave(codewords, sf):
    rows = len(codewords[0])
    cols = sf
    D = [[0] * cols for _ in range(rows)]
    for c, cw in enumerate(codewords):
        for r, bit in enumerate(cw):
            D[r][c] = bit
    I = [[0] * cols for _ in range(rows)]
    for i in range(rows):
        for j in range(cols):
            src_col = (i - (j + 1)) % sf
            I[i][j] = D[j][src_col]
    return D, I

# LoRa Gray coding
def lora_gray(v, sf):
    M = 1 << sf
    v = (v - 1) % M
    return v ^ (v >> 1)

# Base chirp generation
def generate_base_chirp(sf, bw):
    N = 1 << sf
    Ts = N / bw
    t = np.arange(N) / bw
    f0 = -bw / 2.0
    k = bw / Ts
    phase = 2 * np.pi * (f0 * t + 0.5 * k * t**2)
    return np.exp(1j * phase)

def generate_symbol(base_chirp, symbol):
    return np.roll(base_chirp, symbol)

# WAV exporter (float32 for SDR++)
def save_single_wav(full, sample_rate, mode='iq', prefix='lora'):
    os.makedirs(OUT_DIR, exist_ok=True)
    if mode == 'iq':
        arr = np.vstack([np.real(full), np.imag(full)]).T
        m = np.max(np.abs(arr)) or 1.0
        arr = (arr / m).astype(np.float32)
        fname = os.path.join(OUT_DIR, f"{prefix}_packet_iq.wav")
        wavfile.write(fname, sample_rate, arr)
        print(f"Wrote float32 I/Q WAV: {fname}")
        return fname
    # real mode
    real_part = np.real(full)
    m = np.max(np.abs(real_part)) or 1.0
    real_part = (real_part / m).astype(np.float32)
    fname = os.path.join(OUT_DIR, f"{prefix}_packet_real.wav")
    wavfile.write(fname, sample_rate, real_part)
    print(f"Wrote float32 real WAV: {fname}")
    return fname

# Main pipeline
def main():
    bits = byte_to_bits_lsb_first(INPUT_BYTE)
    whitened = whiten_bits(bits, [0, 0, 0, 0, 0, 0, 0, 0, 1])
    wnibbles = [whitened[0:4], whitened[4:8]]
    codewords = [hamming_4_8(n) for n in wnibbles]
    codewords = pad_codewords(codewords, SF)
    D, I = interleave(codewords, SF)
    symbols = [bits_to_int_lsb_first(row) for row in I]
    gray = [lora_gray(v, SF) for v in symbols]

    N = 1 << SF
    base_up = generate_base_chirp(SF, BW)
    base_down = np.conj(base_up)

    # Preamble: 8 unmod upchirps, 2 mod sync upchirps, 2.25 unmod downchirps
    preamble_up = [generate_symbol(base_up, 0) for _ in range(8)]

    # Sync words: nibbles shifted by (SF-4)
    shift = SF - 4
    nib_high = (SYNC_WORD & 0xF0) >> 4
    nib_low = SYNC_WORD & 0x0F
    sync_sym1 = nib_high << shift
    sync_sym2 = nib_low << shift
    sync_up = [generate_symbol(base_up, sync_sym1), generate_symbol(base_up, sync_sym2)]

    # Data chirps
    data_chirps = [generate_symbol(base_up, g) for g in gray]

    # Downchirps: 2 full + 0.25 quarter
    down_full = [generate_symbol(base_down, 0) for _ in range(2)]
    down_quarter = generate_symbol(base_down, 0)[:int(N * 0.25)]

    all_chirps = preamble_up + sync_up + down_full + [down_quarter] + data_chirps

    full = np.concatenate(all_chirps)

    # Upsample to SAMPLE_RATE (rational resample for exact ratio)
    base_rate = BW
    up = int(SAMPLE_RATE / base_rate * 5)  # Adjust for rational; e.g., 96/5 for 125k to 2.4M
    down = 5
    full_upsampled = resample_poly(full, up, down)

    # Apply frequency offset
    t = np.arange(len(full_upsampled)) / SAMPLE_RATE
    full_upsampled *= np.exp(1j * 2 * np.pi * F_OFFSET * t)

    outfile = save_single_wav(full_upsampled, SAMPLE_RATE, mode=EXPORT_MODE, prefix=OUT_PREFIX)
    print(f"Saved: {outfile}")

if __name__ == '__main__':
    main()