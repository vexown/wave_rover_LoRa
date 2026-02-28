#!/usr/bin/env python3
"""
LoRa PHY Layer Encoder — Educational Implementation
====================================================
Based on: Robyns et al., "A Multi-Channel Software Decoder for the LoRa
          Modulation Scheme", PECCS 2018.
          GitHub: https://github.com/rpp0/gr-lora

Target chip : SX1262  (Semtech datasheet rev. 1.1)
              Note: SX1272 / RN2483 use a *different* whitening LFSR that
              Robyns et al. reverse-engineered empirically (Section 3.3).
              The LFSR implemented here follows the SX1262 datasheet.
              Swap whiten_codewords() for a lookup-table if you need exact
              SX1272 / RN2483 wire-compatibility.

Processing pipeline (paper Figure 5 / Section 2):
──────────────────────────────────────────────────
  Raw payload bytes
    ① Split into 4-bit nibbles
    ② Hamming encode each nibble  →  codewords of (4 + CR) bits  [§2.3]
    ③ Whiten codewords with LFSR                                  [§2.3]
    ④ Pad codeword list to SF entries (zero-fill)
    ⑤ Diagonal interleave into SF × (4+CR) chip matrix            [§2.2]
    ⑥ Each matrix row → integer symbol value (LSB-first)
    ⑦ Gray-decode symbol value → chirp time-shift index            [§2.1]
       (gray_decode = Gray→binary; the RX reverses this with gray_encode)
    ⑧ Modulate: roll base upchirp by shift index                  [§2.1]
    ⑨ Prepend: preamble + frame-sync + freq-sync                  [§2.4]
    ⑩ Upsample, apply carrier offset, write IQ WAV
"""

import numpy as np
from scipy.io import wavfile
from scipy.signal import resample_poly
import os

# ─────────────────────────── USER PARAMETERS ──────────────────────────────── #

INPUT_BYTES = bytes([0x54])   # Payload to encode (arbitrary length)
SF          = 8               # Spreading Factor: 7–12
CR          = 4               # Coding Rate parity bits: 1–4  (gives 4/(4+CR))
BW          = 125_000         # Channel bandwidth in Hz
SAMPLE_RATE = 2_400_000       # Output sample rate (must be ≥ BW)
F_OFFSET    = 200_000         # Shift signal away from DC (Hz)
SYNC_WORD   = 0x7F            # my personal sync word
EXPORT_MODE = 'iq'            # 'iq' (stereo float32) or 'real' (mono float32)
OUT_DIR     = 'lora_out'
OUT_PREFIX  = f'lora_SF{SF}_CR{CR}'

# ══════════════════════════════════════════════════════════════════════════════
#  §1 — BIT / BYTE HELPERS
# ══════════════════════════════════════════════════════════════════════════════

def byte_to_bits_lsb(byte: int) -> list[int]:
    """Return the 8 bits of *byte* ordered LSB → MSB."""
    return [(byte >> i) & 1 for i in range(8)]

def bits_lsb_to_int(bits: list[int]) -> int:
    """Reconstruct an integer from bits ordered LSB → MSB."""
    return sum(b << i for i, b in enumerate(bits))

def bits_lsb_to_byte(bits: list[int]) -> int:
    """Pack 8 LSB-first bits into a byte integer."""
    assert len(bits) == 8
    return bits_lsb_to_int(bits)

# ══════════════════════════════════════════════════════════════════════════════
#  §2 — WHITENING  (paper Section 2.3)
# ══════════════════════════════════════════════════════════════════════════════
#
#  The paper states (§2.3):
#    "whitening is defined as an operation where the data is XOR-ed with a
#     9-bit LFSR after synchronization."
#  and crucially that the whitening applies to *codewords* (after Hamming),
#  not to raw payload bytes.
#
#  The SX1262 datasheet (§6.2.3.4) specifies:
#    Polynomial : x⁹ + x⁵ + 1  →  feedback = out[0] XOR out[4]
#    Initial state: register 0x06B8 = 0x01 (MSB), 0x06B9 = 0x00 (LSB)
#    i.e. state = [X8..X0] = [1,0,0,0,0,0,0,0,0]
#
#  ⚠  Robyns et al. found that SX1272 / RN2483 use a *different* sequence
#     (§3.3).  They derived it empirically by transmitting all-zero payloads
#     and XOR-ing received codewords with 0x00.  Their gr-lora source at
#     github.com/rpp0/gr-lora contains the resulting lookup table.
#     Replace this function with that table for SX1272/RN2483 compatibility.
# ─────────────────────────────────────────────────────────────────────────────

def _lfsr_next_byte(lfsr: list[int]) -> tuple[int, list[int]]:
    """
    Advance a 9-bit LFSR (x⁹+x⁵+1) by 8 steps and return one whitening byte.
    State list is [X8, X7, …, X0]; X0 is the output bit.
    Returns (whitening_byte, updated_lfsr).
    """
    w = 0
    for bit_pos in range(8):
        out_bit   = lfsr[8]                    # X0 is output
        w        |= out_bit << (7 - bit_pos)   # pack MSB-first into byte
        feedback  = lfsr[8] ^ lfsr[4]          # taps: X0 ⊕ X4
        lfsr      = [feedback] + lfsr[:8]       # shift right, insert feedback
    return w, lfsr


def whiten_codewords(codeword_bytes: list[int]) -> list[int]:
    """
    XOR each codeword byte with one byte of the SX1262 whitening sequence.

    Parameters
    ----------
    codeword_bytes : list of ints, each 0–255
        Hamming-encoded codewords as packed bytes.

    Returns
    -------
    list of ints — whitened codeword bytes.
    """
    # SX1262 initial LFSR state: [X8..X0] = [1,0,0,0,0,0,0,0,0]
    lfsr = [1, 0, 0, 0, 0, 0, 0, 0, 0]

    whitened = []
    for cw_byte in codeword_bytes:
        w, lfsr  = _lfsr_next_byte(lfsr)
        whitened.append(cw_byte ^ w)

    return whitened

# ══════════════════════════════════════════════════════════════════════════════
#  §3 — HAMMING (4 / 4+CR) CODING  (paper Section 2.3 and Figure 7)
# ══════════════════════════════════════════════════════════════════════════════
#
#  LoRa uses a *modified* Hamming code (paper §2.3).  Figure 7 reveals the
#  bit layout for CR = 4 (8-bit codewords):
#
#    Bit index:  7    6    5    4    3    2    1    0
#    LoRa role:  p0   p1   p3   p2   d3   d2   d1   d0
#
#  Data bits d0–d3 occupy the four LSBs (indices 0–3).
#  Parity bits p0–p3 occupy the four MSBs (indices 4–7), reordered as shown.
#
#  Parity equations (from matching Figure 7's example: data=0001, parity=0111):
#    p0 = d1 ⊕ d2 ⊕ d3   → bit 7
#    p1 = d0 ⊕ d2 ⊕ d3   → bit 6
#    p2 = d0 ⊕ d1 ⊕ d3   → bit 4
#    p3 = d0 ⊕ d1 ⊕ d2   → bit 5
#
#  For CR < 4 fewer parity bits are included (the last 4-CR parity bits are
#  simply dropped), giving codewords of length 4 + CR bits.
# ─────────────────────────────────────────────────────────────────────────────

def hamming_encode_nibble(nibble_bits: list[int], cr: int) -> list[int]:
    """
    Hamming-encode a 4-bit nibble into a (4+CR)-bit LoRa codeword.

    Parameters
    ----------
    nibble_bits : [d0, d1, d2, d3]  (LSB-first)
    cr          : coding-rate parity bits (1–4)

    Returns
    -------
    list of (4+CR) bits: [d0, d1, d2, d3, p0, p1, p2, p3][:4+cr]
    """
    d0, d1, d2, d3 = nibble_bits

    # Parity equations per Figure 7 of Robyns et al.
    p0 = d1 ^ d2 ^ d3
    p1 = d0 ^ d2 ^ d3
    p2 = d0 ^ d1 ^ d3   # occupies bit 4 in the packed byte
    p3 = d0 ^ d1 ^ d2   # occupies bit 5 in the packed byte

    # Full codeword (data in LSBs, parity in MSBs) — trim to 4+CR bits
    full = [d0, d1, d2, d3, p0, p1, p2, p3]
    return full[: 4 + cr]


def encode_payload(payload: bytes, cr: int) -> list[list[int]]:
    """
    Split payload into nibbles and Hamming-encode each one.

    Returns a list of codewords; each codeword is a list of (4+CR) bits.
    """
    codewords = []
    for byte in payload:
        bits   = byte_to_bits_lsb(byte)
        lo_nib = bits[0:4]   # lower nibble (d0..d3)
        hi_nib = bits[4:8]   # upper nibble (d0..d3)
        codewords.append(hamming_encode_nibble(lo_nib, cr))
        codewords.append(hamming_encode_nibble(hi_nib, cr))
    return codewords


def pack_codewords_to_bytes(codewords: list[list[int]]) -> list[int]:
    """
    Pack each (4+CR)-bit codeword into a byte (zero-pad on the left)
    so that whitening can be applied byte-by-byte.
    """
    packed = []
    for cw in codewords:
        # Pad to 8 bits on the MSB side (indices 4+CR .. 7 become 0)
        padded = cw + [0] * (8 - len(cw))
        packed.append(bits_lsb_to_byte(padded))
    return packed


def unpack_codewords_from_bytes(byte_list: list[int], cr: int) -> list[list[int]]:
    """Unpack byte-packed codewords back to bit lists of length (4+CR)."""
    codewords = []
    for b in byte_list:
        bits = byte_to_bits_lsb(b)
        codewords.append(bits[: 4 + cr])
    return codewords

# ══════════════════════════════════════════════════════════════════════════════
#  §4 — INTERLEAVING  (paper Section 2.2)
# ══════════════════════════════════════════════════════════════════════════════
#
#  The paper describes a diagonal interleaver operating on an
#  SF × (4+CR) bit matrix (SF rows = chip values, 4+CR columns = codeword bits).
#
#  At the receiver, deinterleaving proceeds as:
#    "The first chip value provides all first LSBs of the codewords,
#     the second chip value provides all second bits, …"
#  with the diagonal running *upward* (contrary to the patent).
#
#  For the encoder we apply the *inverse* (interleaving) operation.
#  The matrix D[row][col] is filled column-by-column from codewords,
#  then permuted diagonally to produce the chip matrix I whose rows
#  are the chip values transmitted.
# ─────────────────────────────────────────────────────────────────────────────

def pad_codewords(codewords: list[list[int]], sf: int) -> list[list[int]]:
    """Zero-pad the codeword list to exactly SF entries."""
    cw_len = len(codewords[0]) if codewords else (4 + CR)
    while len(codewords) < sf:
        codewords = codewords + [[0] * cw_len]
    return codewords[: sf]   # truncate if somehow longer


def interleave(codewords: list[list[int]], sf: int) -> list[list[int]]:
    """
    Diagonal interleaver: SF codewords of (4+CR) bits → SF chip rows of SF bits.

    Step 1 — build data matrix D[bit_idx][codeword_idx]:
        D[r][c] = bit r of codeword c
    Step 2 — diagonal permutation → chip matrix I[chip][bit]:
        I[i][j] = D[j][(i - j - 1) mod SF]

    Returns a list of SF rows; each row has SF bits (one chip value).
    """
    cw_len = len(codewords[0])   # 4 + CR
    rows   = cw_len              # one row per bit position in the codeword
    cols   = sf                  # one column per codeword (= SF codewords after padding)

    # Build D: rows = bit index (0..cw_len-1), cols = codeword index (0..SF-1)
    D = [[codewords[c][r] for c in range(cols)] for r in range(rows)]

    # Diagonal permutation (upward direction, per paper §2.2)
    I = [[0] * cols for _ in range(rows)]
    for i in range(rows):
        for j in range(cols):
            src_col   = (i - j - 1) % sf
            I[i][j]   = D[j][src_col]

    # Each row of I is one chip value (SF bits)
    return I

# ══════════════════════════════════════════════════════════════════════════════
#  §5 — GRAY CODING  (paper Section 2.1, Equation 2)
# ══════════════════════════════════════════════════════════════════════════════
#
#  The paper defines demodulation as:
#    i = Gray( argmax |FFT( x(t+t̂) · x̄(t) )| )
#
#  So the receiver applies Gray *encoding* (binary→Gray: x ^ (x>>1)) to the
#  FFT argmax to recover symbol i.  Working backwards, the transmitter must
#  apply Gray *inverse* / *decoding* (Gray→binary) to symbol i to obtain the
#  time-shift index t̂ that rolls the base chirp.
#
#  ⚠ Naming trap: "encode" / "decode" refer to the Gray-code operation,
#    NOT to the TX/RX role.  The TX uses gray_decode and the RX uses
#    gray_encode.  See lora_gray_encode.py's docstring for the full picture.
#
#    TX:  chirp_shift   = gray_decode(symbol_value)   [Gray→binary]
#    RX:  symbol_value   = gray_encode(bin_index)      [binary→Gray]
# ─────────────────────────────────────────────────────────────────────────────

def gray_encode(v: int) -> int:
    """binary → Gray code:  G(v) = v ^ (v >> 1).

    Used by the RX (lora_gray_encode.py) to recover symbol values from
    FFT bin indices.  Not used in the TX path.
    """
    return v ^ (v >> 1)

def gray_decode(g: int) -> int:
    """Gray code → binary (inverse of gray_encode).

    Used by the TX to convert symbol values into chirp time-shift indices.
    The TX applies gray_decode (not gray_encode!) because it is converting
    FROM Gray-code space (symbol values) INTO natural order (physical shifts).
    """
    v = g
    mask = v >> 1
    while mask:
        v    ^= mask
        mask >>= 1
    return v

# ══════════════════════════════════════════════════════════════════════════════
#  §6 — CHIRP GENERATION & MODULATION  (paper Section 2.1, Equation 1)
# ══════════════════════════════════════════════════════════════════════════════
#
#  Base (unmodulated) upchirp:
#    x(t) = exp(j·(2π(f0·t + ½·k·t²)))
#    where f0 = −BW/2,  k = BW/T,  T = 2^SF / BW
#    sampled at N = 2^SF samples over [0, T).
#
#  Modulating symbol i onto the base chirp (paper §2.1):
#    time shift  t̂ = gray_inverse(i) · T/2^SF = gray_inverse(i) samples
#    x_i(t)     = x(t + t̂)  ≡  np.roll(base_chirp, t̂)
# ─────────────────────────────────────────────────────────────────────────────

def make_base_upchirp(sf: int, bw: float) -> np.ndarray:
    """
    Generate one period of the base upchirp (N = 2^SF complex samples).
    Frequency sweeps from −BW/2 to +BW/2 over N samples (Eq. 1 of paper).
    """
    N  = 1 << sf
    T  = N / bw                  # symbol duration in seconds
    t  = np.arange(N) / bw       # time axis: 0, 1/BW, 2/BW, …, (N-1)/BW
    f0 = -bw / 2.0
    k  = bw / T                  # = bw² / N
    phase = 2 * np.pi * (f0 * t + 0.5 * k * t ** 2)
    return np.exp(1j * phase)

def modulate_symbol(base_upchirp: np.ndarray, symbol: int) -> np.ndarray:
    """
    Modulate an integer symbol onto the base chirp by circular time shift.

    *symbol* is the gray_decode'd value (i.e. the raw interleaver output
    passed through gray_decode to convert from Gray-code space to a
    natural-order shift index).  See §5 for the TX/RX Gray-code roles.
    """
    return np.roll(base_upchirp, symbol)

# ══════════════════════════════════════════════════════════════════════════════
#  §7 — PREAMBLE & FRAME SYNCHRONISATION SYMBOLS  (paper Section 2.4)
# ══════════════════════════════════════════════════════════════════════════════
#
#  LoRa PHY frame (Figure 3 / Section 2.4):
#    [Preamble: 8 base upchirps]
#    [Frame sync: 2 upchirps modulated with sync-word nibbles]
#    [Freq. sync: 2 full downchirps + 1 quarter downchirp]
#    [Header (optional)]
#    [Payload chirps]
#
#  Frame synchronisation (sync word encoding, §2.4):
#    The two sync-word symbols encode the high and low nibbles of SYNC_WORD.
#    Each nibble is left-shifted by (SF − 4) to place it in the correct
#    frequency bin of the SF-bit chip space.
# ─────────────────────────────────────────────────────────────────────────────

def build_preamble(base_up: np.ndarray, base_down: np.ndarray,
                   sync_word: int, sf: int, n_preamble: int = 8) -> np.ndarray:
    """
    Construct the LoRa preamble + synchronisation block.

    Returns concatenated complex samples for:
        8 × unmodulated upchirps
        2 × sync-word upchirps  (high nibble, low nibble)
        2 × full downchirps
        1 × quarter downchirp
    """
    # ── 8 unmodulated upchirps ──────────────────────────────────────────────
    preamble = [base_up.copy() for _ in range(n_preamble)]

    # ── 2 frame-sync upchirps (sync word nibbles, shifted by SF-4) ──────────
    shift    = sf - 4
    nib_hi   = (sync_word & 0xF0) >> 4
    nib_lo   =  sync_word & 0x0F
    sync_sym = [
        modulate_symbol(base_up, nib_hi << shift),
        modulate_symbol(base_up, nib_lo << shift),
    ]

    # ── 2 full downchirps + 1 quarter downchirp (freq. sync) ────────────────
    N             = len(base_down)
    down_full     = [base_down.copy(), base_down.copy()]
    down_quarter  = base_down[: N // 4]

    all_blocks = preamble + sync_sym + down_full + [down_quarter]
    return np.concatenate(all_blocks)

# ══════════════════════════════════════════════════════════════════════════════
#  §8 — OUTPUT: UPSAMPLE + FREQUENCY SHIFT + WAV EXPORT
# ══════════════════════════════════════════════════════════════════════════════

def upsample_signal(signal: np.ndarray, bw: float, sample_rate: float,
                    lcm_factor: int = 5) -> np.ndarray:
    """
    Rational-ratio upsample from the chip-rate (= BW) to *sample_rate*.
    Uses scipy.signal.resample_poly for exact rational resampling.

    up/down ratio: sample_rate / BW  =  (sample_rate * lcm_factor / BW) / lcm_factor
    """
    up   = int(sample_rate / bw * lcm_factor)
    down = lcm_factor
    return resample_poly(signal, up, down)


def apply_frequency_offset(signal: np.ndarray, f_offset: float,
                            sample_rate: float) -> np.ndarray:
    """Shift baseband signal by *f_offset* Hz (complex exponential mix)."""
    t = np.arange(len(signal)) / sample_rate
    return signal * np.exp(1j * 2 * np.pi * f_offset * t)


def save_iq_wav(signal: np.ndarray, sample_rate: int,
                filepath: str) -> None:
    """Write a normalised float32 stereo WAV file (I = left, Q = right)."""
    iq  = np.vstack([np.real(signal), np.imag(signal)]).T.astype(np.float32)
    iq /= np.max(np.abs(iq)) or 1.0
    wavfile.write(filepath, sample_rate, iq)
    print(f"  Wrote IQ WAV : {filepath}")


def save_real_wav(signal: np.ndarray, sample_rate: int,
                  filepath: str) -> None:
    """Write a normalised float32 mono WAV file (real part only)."""
    real = np.real(signal).astype(np.float32)
    real /= np.max(np.abs(real)) or 1.0
    wavfile.write(filepath, sample_rate, real)
    print(f"  Wrote real WAV: {filepath}")

# ══════════════════════════════════════════════════════════════════════════════
#  §9 — MAIN PIPELINE
# ══════════════════════════════════════════════════════════════════════════════

def main():
    print("=" * 60)
    print("  LoRa PHY Encoder  (Robyns et al. 2018)")
    print("=" * 60)
    print(f"  Payload   : {INPUT_BYTES.hex().upper()}")
    print(f"  SF={SF}  CR=4/{4+CR}  BW={BW//1000} kHz  Sync=0x{SYNC_WORD:02X}")
    print()

    # ── ① Hamming encode payload → codewords ──────────────────────────────
    print("① Hamming encoding …")
    codewords = encode_payload(INPUT_BYTES, CR)
    packed_cw = pack_codewords_to_bytes(codewords)
    print(f"   Codeword bytes (pre-whiten) : "
          f"{[f'0x{b:02X}' for b in packed_cw]}")

    # ── ② Whiten codewords (applied *after* Hamming, *before* interleaving) ─
    print("② Whitening codewords …")
    whitened_bytes = whiten_codewords(packed_cw)
    print(f"   Codeword bytes (post-whiten): "
          f"{[f'0x{b:02X}' for b in whitened_bytes]}")
    whitened_cw = unpack_codewords_from_bytes(whitened_bytes, CR)

    # ── ③ Pad to exactly SF codewords ─────────────────────────────────────
    print("③ Padding to SF codewords …")
    padded_cw = pad_codewords(whitened_cw, SF)
    print(f"   Codeword count: {len(padded_cw)}  (SF={SF})")

    # ── ④ Diagonal interleave → chip matrix ───────────────────────────────
    print("④ Interleaving …")
    chip_matrix = interleave(padded_cw, SF)
    chip_values = [bits_lsb_to_int(row) for row in chip_matrix]
    print(f"   Chip values (raw)  : {chip_values}")

    # ── ⑤ Gray-decode symbol values to get time-shift indices ─────────────
    #    gray_decode() here = Gray→binary.  The TX "decodes" from Gray-code
    #    space into natural order so the chirp shift is a plain integer.
    #    The RX will reverse this with gray_encode() (binary→Gray).
    print("⑤ Gray-decode → chirp time-shift indices …")
    shifts = [gray_decode(v) for v in chip_values]
    print(f"   Time-shift indices : {shifts}")

    # ── ⑥ Generate chirps ─────────────────────────────────────────────────
    print("⑥ Generating chirps …")
    base_up   = make_base_upchirp(SF, BW)
    base_down = np.conj(base_up)

    preamble_block = build_preamble(base_up, base_down, SYNC_WORD, SF)
    data_chirps    = np.concatenate([
        modulate_symbol(base_up, s) for s in shifts
    ])

    full_signal = np.concatenate([preamble_block, data_chirps])
    print(f"   Total baseband samples: {len(full_signal)}")

    # ── ⑦ Upsample + frequency shift ──────────────────────────────────────
    print(f"⑦ Upsampling {BW//1000} kHz → {SAMPLE_RATE//1000} kHz …")
    up_signal = upsample_signal(full_signal, BW, SAMPLE_RATE)

    print(f"   Applying {F_OFFSET//1000} kHz carrier offset …")
    tx_signal = apply_frequency_offset(up_signal, F_OFFSET, SAMPLE_RATE)

    # ── ⑧ Save WAV ─────────────────────────────────────────────────────────
    print("⑧ Saving WAV …")
    os.makedirs(OUT_DIR, exist_ok=True)
    if EXPORT_MODE == 'iq':
        path = os.path.join(OUT_DIR, f"{OUT_PREFIX}_iq.wav")
        save_iq_wav(tx_signal, SAMPLE_RATE, path)
    else:
        path = os.path.join(OUT_DIR, f"{OUT_PREFIX}_real.wav")
        save_real_wav(tx_signal, SAMPLE_RATE, path)

    print()
    print("Done.")


if __name__ == '__main__':
    main()