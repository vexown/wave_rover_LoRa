#!/usr/bin/env python3
"""
LoRa PHY Layer — Stage-by-Stage Capture & Verification Script
==============================================================
Companion to lora_encoder.py. Captures IQ samples from an RTL-SDR,
then runs the decode pipeline printing intermediate values at every
stage so you can compare them against the encoder's output.

Usage
-----
1. Configure CAPTURE settings below to match your Heltec TX config.
2. Run this script BEFORE triggering a transmission on the Heltec,
   or set MODE = 'file' and point IQ_FILE at a previously saved capture.
3. Compare the printed intermediate values with those printed by lora_encoder.py.

Pipeline (inverse of encoder):
──────────────────────────────
  IQ samples
    ① Detect preamble  (Schmidl-Cox autocorrelation)
    ② Synchronise      (cross-correlate instantaneous frequency)
    ③ Dechirp + FFT    →  chip values
    ④ Gray encode      →  interleaved symbol values
    ⑤ Deinterleave     →  whitened codewords
    ⑥ De-whiten        →  Hamming codewords
    ⑦ Hamming decode   →  nibbles  →  payload bytes
"""

import numpy as np
from scipy.signal import resample_poly
import matplotlib.pyplot as plt
import os
import sys

# ─────────────────────────── CONFIGURATION ────────────────────────────────── #

MODE        = 'sdr'        # 'sdr' = live capture,  'file' = load saved IQ WAV
IQ_FILE     = 'capture.npy'  # used when MODE = 'file'  (.npy complex64 array)

# Must match your Heltec TX settings exactly
CENTER_FREQ = 869.725e6   # tune 200 kHz above the actual signal. The center freq is exactly where the RTL-SDR's DC spike sits. The DC spike can swamp the signal.
F_OFFSET    = -200_000    # shift the captured signal back down by 200 kHz
SF          = 8
CR          = 4            # parity bits  (so 4/8 coding rate)
BW          = 125_000      # Hz
SAMPLE_RATE = 2_400_000    # samples/sec  — same as encoder
RECORD_SECONDS = 10

# RTL-SDR gain.  'auto' works for a nearby transmitter; increase if weak signal.
SDR_GAIN    = 'auto'

# Preamble detection threshold for Schmidl-Cox metric (0–1).
# Lower = more sensitive but more false positives.
PREAMBLE_THRESHOLD = 0.7

# How many symbols to capture after the preamble+sync block.
# = SF rows of the interleaver  (one per chip value)
N_DATA_SYMBOLS = SF

PLOT = True   # set False to suppress matplotlib windows

# ══════════════════════════════════════════════════════════════════════════════
#  HELPERS — shared with encoder (duplicated here for self-containment)
# ══════════════════════════════════════════════════════════════════════════════

def byte_to_bits_lsb(byte):
    return [(byte >> i) & 1 for i in range(8)]

def bits_lsb_to_int(bits):
    return sum(b << i for i, b in enumerate(bits))

def make_base_upchirp(sf, bw):
    N  = 1 << sf
    t  = np.arange(N) / bw
    f0 = -bw / 2.0
    k  = bw ** 2 / N
    return np.exp(1j * 2 * np.pi * (f0 * t + 0.5 * k * t ** 2))

def gray_encode(v):
    return v ^ (v >> 1)

def gray_decode(g):
    v = g
    mask = v >> 1
    while mask:
        v ^= mask
        mask >>= 1
    return v

# ══════════════════════════════════════════════════════════════════════════════
#  §1 — CAPTURE
# ══════════════════════════════════════════════════════════════════════════════

def capture_from_sdr(n_samples: int) -> np.ndarray:
    try:
        from rtlsdr import RtlSdr
    except ImportError:
        print("ERROR: pyrtlsdr not installed.  Run:  pip install pyrtlsdr")
        sys.exit(1)

    sdr = RtlSdr()
    sdr.sample_rate = SAMPLE_RATE
    sdr.center_freq = CENTER_FREQ
    sdr.gain        = SDR_GAIN

    # RTL-SDR v4 overflows on large single reads — chunk it
    CHUNK_SIZE = 32768   # must be a multiple of 512 (USB packet size)
    n_chunks   = -(-n_samples // CHUNK_SIZE)   # ceiling division
    print(f"  Capturing {n_chunks} × {CHUNK_SIZE} samples "
          f"at {CENTER_FREQ/1e6:.3f} MHz …")

    chunks = []
    for i in range(n_chunks):
        chunks.append(sdr.read_samples(CHUNK_SIZE))

    sdr.close()
    return np.concatenate(chunks).astype(np.complex64)[:n_samples]


def load_from_file(path: str) -> np.ndarray:
    ext = os.path.splitext(path)[1].lower()
    if ext == '.npy':
        return np.load(path).astype(np.complex64)
    elif ext == '.wav':
        from scipy.io import wavfile
        rate, data = wavfile.read(path)
        if data.ndim == 2:          # stereo IQ WAV (I=left, Q=right)
            return (data[:, 0] + 1j * data[:, 1]).astype(np.complex64)
        return data.astype(np.complex64)
    else:
        raise ValueError(f"Unsupported file format: {ext}")


def remove_dc_offset(samples: np.ndarray) -> np.ndarray:
    """Subtract mean to eliminate RTL-SDR DC spike."""
    return samples - np.mean(samples)


def shift_frequency(samples: np.ndarray, f_shift: float,
                    sample_rate: float) -> np.ndarray:
    """Shift signal by -f_shift Hz to move carrier back to DC."""
    t = np.arange(len(samples)) / sample_rate
    return samples * np.exp(-1j * 2 * np.pi * f_shift * t)


def downsample_to_bw(samples: np.ndarray, sample_rate: float,
                     bw: float, lcm: int = 5) -> np.ndarray:
    """Downsample from sample_rate to BW (chip rate) using rational resampling."""
    up   = lcm
    down = int(sample_rate / bw * lcm)
    return resample_poly(samples, up, down)

# ══════════════════════════════════════════════════════════════════════════════
#  §2 — PREAMBLE DETECTION  (Schmidl-Cox, paper §3.1)
# ══════════════════════════════════════════════════════════════════════════════
#
#  The timing metric M(d) = |P(d)|² / R(d)² measures normalised
#  autocorrelation over one symbol length L.  It plateaus inside
#  the preamble where consecutive symbols are identical.
# ─────────────────────────────────────────────────────────────────────────────

def schmidl_cox(samples: np.ndarray, L: int) -> np.ndarray:
    """
    Compute the Schmidl-Cox timing metric M(d) for all sample positions.
    L = symbol length in samples = 2^SF (at chip rate).
    """
    n  = len(samples)
    P  = np.zeros(n, dtype=complex)
    R  = np.zeros(n)

    for d in range(n - 2 * L):
        P[d] = np.sum(np.conj(samples[d:d+L]) * samples[d+L:d+2*L])
        R[d] = np.sum(np.abs(samples[d+L:d+2*L]) ** 2)

    # Avoid division by zero
    R    = np.where(R < 1e-12, 1e-12, R)
    M    = (np.abs(P) ** 2) / (R ** 2)
    return M


def find_preamble(samples: np.ndarray, sf: int,
                  threshold: float) -> int:
    """
    Return the sample index of the first detected preamble.
    Uses Schmidl-Cox metric thresholded at *threshold*.
    Returns -1 if no preamble found.
    """
    L = 1 << sf
    M = schmidl_cox(samples, L)

    # Find first sustained plateau above threshold
    above = np.where(M > threshold)[0]
    if len(above) == 0:
        return -1
    return int(above[0])

# ══════════════════════════════════════════════════════════════════════════════
#  §3 — SYNCHRONISATION  (paper §3.1, Equation 6)
# ══════════════════════════════════════════════════════════════════════════════
#
#  Cross-correlate instantaneous frequency of received signal with that of a
#  locally generated base chirp.  The peak gives the symbol boundary.
#  Operating on instantaneous frequency (not complex values) makes this
#  invariant to carrier frequency offset — a key insight of the paper.
# ─────────────────────────────────────────────────────────────────────────────

def instantaneous_frequency(samples: np.ndarray) -> np.ndarray:
    """
    Estimate instantaneous frequency from the derivative of the phase.
    Returns an array one sample shorter than the input.
    """
    phase = np.unwrap(np.angle(samples))
    return np.diff(phase) / (2 * np.pi)   # normalised to cycles/sample


def refine_symbol_start(samples: np.ndarray, base_chirp: np.ndarray,
                        rough_start: int) -> int:
    """
    Slide a window of one symbol length around *rough_start* and find the
    position that maximises cross-correlation of instantaneous frequencies.
    Returns the refined sample index.
    """
    L         = len(base_chirp)
    ifreq_ref = instantaneous_frequency(base_chirp)
    search    = range(max(0, rough_start - L), min(len(samples) - 2*L,
                                                    rough_start + L))
    best_idx  = rough_start
    best_val  = -np.inf

    for d in search:
        window    = samples[d : d + L]
        ifreq_rx  = instantaneous_frequency(window)
        corr      = np.correlate(ifreq_rx, ifreq_ref, mode='valid')
        val       = float(np.max(np.abs(corr)))
        if val > best_val:
            best_val = val
            best_idx = d

    return best_idx

# ══════════════════════════════════════════════════════════════════════════════
#  §4 — DECHIRP + FFT → CHIP VALUES  (paper §2.1, Equation 2)
# ══════════════════════════════════════════════════════════════════════════════
#
#  For each received symbol window of N = 2^SF samples:
#    1. Multiply element-wise by conjugate base chirp  (dechirp)
#    2. Take FFT, find the bin with maximum magnitude
#    3. That bin index is the chip value (before Gray encoding)
# ─────────────────────────────────────────────────────────────────────────────

def dechirp_symbol(symbol_samples: np.ndarray,
                   base_upchirp: np.ndarray) -> int:
    """
    Dechirp one symbol and return its raw chip value (FFT argmax).
    symbol_samples must be exactly N = 2^SF samples.
    """
    dechirped = symbol_samples * np.conj(base_upchirp)
    spectrum  = np.abs(np.fft.fft(dechirped))
    return int(np.argmax(spectrum))


def extract_chip_values(samples: np.ndarray, symbol_start: int,
                        n_symbols: int, base_upchirp: np.ndarray,
                        skip_symbols: int = 0) -> list[int]:
    """
    Extract *n_symbols* chip values starting at *symbol_start*.
    *skip_symbols* lets you jump over preamble / sync / freq-sync blocks.
    """
    N       = len(base_upchirp)
    start   = symbol_start + skip_symbols * N
    chips   = []
    for i in range(n_symbols):
        s0   = start + i * N
        s1   = s0 + N
        if s1 > len(samples):
            print(f"  WARNING: ran out of samples at symbol {i}")
            break
        chip = dechirp_symbol(samples[s0:s1], base_upchirp)
        chips.append(chip)
    return chips

# ══════════════════════════════════════════════════════════════════════════════
#  §5 — DEINTERLEAVING  (paper §2.2)
# ══════════════════════════════════════════════════════════════════════════════
#
#  Inverse of the encoder's diagonal interleave.
#  chip_matrix I → data matrix D → codewords
# ─────────────────────────────────────────────────────────────────────────────

def deinterleave(chip_matrix: list[list[int]], sf: int,
                 cr: int) -> list[list[int]]:
    """
    Diagonal deinterleave: SF rows of SF bits → (4+CR) codewords of SF bits.

    This is the inverse permutation of interleave() in the encoder:
        D[j][src_col] = I[i][j]   where src_col = (i - j - 1) % SF
    Solving for D:
        D[j][(i - j - 1) % SF] = I[i][j]
    """
    rows   = 4 + cr   # codeword bit length = number of rows in D
    cols   = sf        # SF chip values = SF columns in D

    # Rebuild D from I using the inverse permutation
    D = [[0] * cols for _ in range(rows)]
    for i in range(rows):
        for j in range(cols):
            src_col      = (i - j - 1) % sf
            D[j][src_col] = chip_matrix[i][j]

    # Each column of D is one codeword
    codewords = [
        [D[r][c] for r in range(rows)]
        for c in range(cols)
    ]
    return codewords


def chip_values_to_matrix(chip_values: list[int], sf: int,
                          cr: int) -> list[list[int]]:
    """
    Unpack chip integer values into a bit matrix of shape (4+CR) × SF.
    Row i = bit i of each chip value (LSB-first unpacking).
    """
    rows = 4 + cr
    return [
        [(chip_values[j] >> i) & 1 for j in range(sf)]
        for i in range(rows)
    ]

# ══════════════════════════════════════════════════════════════════════════════
#  §6 — DEWHITENING  (paper §2.3 / §3.3)
# ══════════════════════════════════════════════════════════════════════════════

def _lfsr_next_byte(lfsr):
    w = 0
    for bit_pos in range(8):
        out_bit  = lfsr[8]
        w       |= out_bit << (7 - bit_pos)
        feedback = lfsr[8] ^ lfsr[4]
        lfsr     = [feedback] + lfsr[:8]
    return w, lfsr


def dewhiten_codewords(whitened_bytes: list[int]) -> list[int]:
    """
    XOR each codeword byte with the SX1262 LFSR whitening sequence.
    XOR is its own inverse, so de-whitening is identical to whitening.
    """
    lfsr      = [1, 0, 0, 0, 0, 0, 0, 0, 0]
    result    = []
    for b in whitened_bytes:
        w, lfsr = _lfsr_next_byte(lfsr)
        result.append(b ^ w)
    return result

# ══════════════════════════════════════════════════════════════════════════════
#  §7 — HAMMING DECODE  (paper §2.3, Figure 7)
# ══════════════════════════════════════════════════════════════════════════════
#
#  Data bits are at bit indices 0–3 (LSBs) of each codeword byte.
#  Parity bits occupy indices 4–7 in the layout: p0, p1, p3, p2 (MSB→LSB).
#
#  For now we just extract data bits directly (no error correction).
#  To add error correction, implement the parity check equations from
#  the encoder's hamming_encode_nibble() and flip the offending bit.
# ─────────────────────────────────────────────────────────────────────────────

def hamming_decode_nibble(cw_byte: int) -> list[int]:
    """
    Extract the 4 data bits from a Hamming codeword byte.
    Data bits d0–d3 sit at bit positions 0–3 (LSBs).
    """
    return [(cw_byte >> i) & 1 for i in range(4)]


def decode_codewords_to_bytes(codeword_bytes: list[int]) -> bytes:
    """
    Hamming-decode pairs of codewords (lo nibble, hi nibble) → payload bytes.
    """
    payload = []
    # Walk in pairs; odd leftovers are padding, discard them
    for i in range(0, len(codeword_bytes) - 1, 2):
        lo_bits = hamming_decode_nibble(codeword_bytes[i])
        hi_bits = hamming_decode_nibble(codeword_bytes[i + 1])
        byte    = bits_lsb_to_int(lo_bits + hi_bits)
        payload.append(byte)
    return bytes(payload)

# ══════════════════════════════════════════════════════════════════════════════
#  PLOTTING UTILITIES
# ══════════════════════════════════════════════════════════════════════════════

def plot_spectrogram(samples: np.ndarray, sample_rate: float,
                     title: str = "Spectrogram") -> None:
    plt.figure(figsize=(12, 4))
    plt.specgram(samples, NFFT=256, Fs=sample_rate, noverlap=128,
                 cmap='inferno', sides='twosided')
    plt.title(title)
    plt.xlabel("Time (s)")
    plt.ylabel("Frequency (Hz)")
    plt.colorbar(label="Power (dB)")
    plt.tight_layout()
    plt.show()


def plot_fft_peak(symbol_samples: np.ndarray,
                  base_upchirp: np.ndarray, chip_value: int) -> None:
    dechirped = symbol_samples * np.conj(base_upchirp)
    spectrum  = np.abs(np.fft.fft(dechirped))
    N         = len(spectrum)
    plt.figure(figsize=(10, 3))
    plt.plot(spectrum)
    plt.axvline(chip_value, color='red', linestyle='--',
                label=f'Peak @ bin {chip_value}')
    plt.title("Dechirped FFT — first data symbol")
    plt.xlabel("Bin (= chip value)")
    plt.ylabel("|FFT|")
    plt.legend()
    plt.tight_layout()
    plt.show()


def plot_schmidl_cox(M: np.ndarray, threshold: float,
                     preamble_idx: int) -> None:
    plt.figure(figsize=(12, 3))
    plt.plot(M, label='M(d)')
    plt.axhline(threshold, color='orange', linestyle='--',
                label=f'Threshold {threshold}')
    plt.axvline(preamble_idx, color='red', linestyle='-',
                label=f'Detected @ {preamble_idx}')
    plt.title("Schmidl-Cox Timing Metric")
    plt.xlabel("Sample index")
    plt.ylabel("M(d)")
    plt.legend()
    plt.tight_layout()
    plt.show()

# ══════════════════════════════════════════════════════════════════════════════
#  MAIN PIPELINE
# ══════════════════════════════════════════════════════════════════════════════

def main():
    print("=" * 60)
    print("  LoRa PHY Decoder — Stage-by-Stage Verification")
    print("=" * 60)

    N = 1 << SF   # samples per symbol at chip rate

    # ── ① Capture ─────────────────────────────────────────────────────────
    print("① Acquiring IQ samples …")
    if MODE == 'sdr':
        # Capture enough for preamble (8+2+2.25 symbols) + data (SF symbols)
        n_preamble_syms = 8 + 2 + 3          # generous overcount
        n_capture = int(RECORD_SECONDS * SAMPLE_RATE)
        raw = capture_from_sdr(n_capture)
    else:
        print(f"   Loading from file: {IQ_FILE}")
        raw = load_from_file(IQ_FILE)

    raw = remove_dc_offset(raw)
    print(f"   Samples captured: {len(raw)}")

    if PLOT:
        plot_spectrogram(raw, SAMPLE_RATE, "Raw capture — full bandwidth")

    # ── ② Shift carrier and downsample to chip rate ───────────────────────
    print(f"② Shifting by -{F_OFFSET/1e3:.0f} kHz and downsampling to "
          f"{BW/1e3:.0f} kHz …")
    baseband  = shift_frequency(raw, F_OFFSET, SAMPLE_RATE)
    chip_rate = downsample_to_bw(baseband, SAMPLE_RATE, BW)
    print(f"   Baseband samples at chip rate: {len(chip_rate)}")

    # ── ③ Preamble detection (Schmidl-Cox) ────────────────────────────────
    print("③ Detecting preamble (Schmidl-Cox) …")
    M           = schmidl_cox(chip_rate, N)
    rough_start = find_preamble(chip_rate, SF, PREAMBLE_THRESHOLD)

    if rough_start < 0:
        print("   ERROR: No preamble detected.  Check frequency, gain, "
              "and that the Heltec is transmitting.")
        return

    print(f"   Rough preamble start: sample {rough_start}")

    if PLOT:
        plot_schmidl_cox(M, PREAMBLE_THRESHOLD, rough_start)

    # ── ④ Fine synchronisation ────────────────────────────────────────────
    print("④ Refining symbol boundary …")
    base_up    = make_base_upchirp(SF, BW)
    sym_start  = refine_symbol_start(chip_rate, base_up, rough_start)
    print(f"   Refined symbol start: sample {sym_start}")

    # ── ⑤ Extract chip values ─────────────────────────────────────────────
    #  Skip:  8 preamble  +  2 frame-sync  +  2 full downchirps
    #         +  1 quarter downchirp (we treat it as one symbol slot worth of
    #            samples but only read 0.25 of it — simplest: skip 3 symbols
    #            total for freq-sync and absorb the error into sync tolerance)
    PREAMBLE_SYMS    = 8
    FRAME_SYNC_SYMS  = 2
    FREQ_SYNC_SYMS   = 3   # 2 full + 1 quarter (rounded up)
    total_skip       = PREAMBLE_SYMS + FRAME_SYNC_SYMS + FREQ_SYNC_SYMS

    print(f"⑤ Extracting {N_DATA_SYMBOLS} data chip values "
          f"(skipping {total_skip} preamble/sync symbols) …")
    chip_values = extract_chip_values(chip_rate, sym_start, N_DATA_SYMBOLS,
                                      base_up, skip_symbols=total_skip)
    print(f"   Raw chip values (FFT argmax): {chip_values}")

    # Compare against encoder output here:
    print("   ← compare with encoder's 'Chip values (raw)'")

    if PLOT and chip_values:
        s0 = sym_start + total_skip * N
        plot_fft_peak(chip_rate[s0:s0+N], base_up, chip_values[0])

    # ── ⑥ Gray encode chip values → interleaved symbols ───────────────────
    print("⑥ Gray encoding chip values …")
    gray_symbols = [gray_encode(c) for c in chip_values]
    print(f"   Gray-encoded symbols: {gray_symbols}")
    print("   ← compare with encoder's 'Chip values (raw)' "
          "(these should match the interleaver output)")

    # ── ⑦ Deinterleave → whitened codeword bits ───────────────────────────
    print("⑦ Deinterleaving …")
    chip_bit_matrix  = chip_values_to_matrix(gray_symbols, SF, CR)
    codeword_bits    = deinterleave(chip_bit_matrix, SF, CR)

    # Pack each codeword bit list into a byte
    codeword_bytes = [bits_lsb_to_int(cw) for cw in codeword_bits]
    print(f"   Whitened codeword bytes: "
          f"{[f'0x{b:02X}' for b in codeword_bytes]}")
    print("   ← compare with encoder's 'Codeword bytes (post-whiten)'")

    # ── ⑧ Dewhiten ────────────────────────────────────────────────────────
    print("⑧ Dewhitening …")
    plain_codeword_bytes = dewhiten_codewords(codeword_bytes)
    print(f"   Plain codeword bytes:    "
          f"{[f'0x{b:02X}' for b in plain_codeword_bytes]}")
    print("   ← compare with encoder's 'Codeword bytes (pre-whiten)'")

    # ── ⑨ Hamming decode → payload ─────────────────────────────────────────
    print("⑨ Hamming decoding …")
    payload = decode_codewords_to_bytes(plain_codeword_bytes)
    print(f"   Decoded payload (hex): {payload.hex().upper()}")
    print(f"   Decoded payload (str): {payload!r}")
    print()
    print("=" * 60)
    print("  Stage comparison checklist:")
    print("=" * 60)
    print("  ⑤ chip_values (raw)          → encoder: 'Chip values (raw)'")
    print("  ⑦ whitened codeword bytes    → encoder: 'Codeword bytes (post-whiten)'")
    print("  ⑧ plain codeword bytes       → encoder: 'Codeword bytes (pre-whiten)'")
    print("  ⑨ payload                    → your original INPUT_BYTES")
    print()
    print("  If stage N matches but stage N+1 doesn't, that stage")
    print("  has a bug (or a hardware/config mismatch).")
    print("=" * 60)


if __name__ == '__main__':
    main()