#!/usr/bin/env python3
"""
LoRa Symbol Observer
=========================================
This script answers one question: "what chip value is each symbol?"

A LoRa symbol is a chirp (a signal that sweeps from low to high frequency
over time). Each symbol carries information by starting that sweep at a
different frequency offset — that offset is called the "chip value" and
is an integer from 0 to 2^SF - 1 (so 0-255 for SF=8).

──────────────────────────────────────────────────────────────────────────────────────────
Note - Raw IQ samples must be captured separately using external tools
(e.g. rtl_sdr) and saved to a file (you can use capture_rtl_sdr_v4.sh to do this).
This script then performs deterministic, repeatable analysis on that recorded data.

This separation avoids SDR driver instability and allows the DSP
analysis below to be rerun many times on the exact same capture.
──────────────────────────────────────────────────────────────────────────────────────────

This script:
  1. Loads raw IQ samples from a file
  2. Shifts the signal to baseband and downsamples to the LoRa chip rate
  3. Finds where in the recording the transmission actually is
  4. Walks through the signal one symbol at a time and prints the chip value
     of each symbol

That's it. No decoding, no preamble detection algorithm, no assumptions
about frame structure. We just observe and describe what we see.

HOW TO READ THE OUTPUT TABLE (which is printed to the terminal)
──────────────────────────────────────────────────────────────────────────────────────────
  Sym       — symbol index counting from the detected burst start (0 = first)
  Start     — sample number in the chip-rate signal where this symbol begins
  Peak bin  — the chip value: which FFT bin had the most energy after dechirping.
              This is the "message" encoded in this symbol (0-255 for SF=8).
  Peak/Mean — signal quality ratio: peak bin energy divided by average bin energy.
              A clean single-frequency tone after dechirping concentrates all its
              energy into ONE bin, so a clean symbol has a very high ratio.
              > 20  = very clean symbol, chip value is reliable
              5-20  = usable but some noise present
              < 5   = noise only, no real signal here, chip value is meaningless

WHAT TO EXPECT IN A VALID LORA FRAME (SF=8, CR=4/8)
────────────────────────────────────────────────────────────────────────────────────
  Symbols 0-7   (8 symbols)   PREAMBLE
                               All 8 should show the SAME peak bin with PMR > 20.
                               The value is ideally bin 0 (unmodulated base chirp)
                               but may be a small consistent offset due to the
                               capture device's inherent frequency error (~few ppm).

  Symbols 8-9   (2 symbols)   FRAME SYNC (sync word)
                               Two symbols with DIFFERENT bin values from the
                               preamble and from each other. These encode the
                               network sync word nibbles, left-shifted by SF-4.
                               e.g. sync word 0x7F → hi nibble 0x7, lo nibble 0xF
                               → symbol bins: 0x7 << 4 = 112  and  0xF << 4 = 240

  Symbols 10-11 (2 symbols)   FREQUENCY SYNC (downchirps)
                               Two full downchirps (SX1262 SFD).
                               A downchirp sweeps HIGH to LOW frequency — the
                               opposite of our base upchirp. When we dechirp
                               a downchirp with an upchirp template, the chirp
                               does NOT cancel out cleanly, so energy spreads
                               across ALL bins instead of concentrating in one.
                               Result: these symbols show LOW PMR. This is not
                               an error — it is the expected and correct behaviour.

  Symbols 12-19 (8 symbols)   HEADER  (4+CR = 8 symbols, always CR=4)
                               Contains payload length, coding rate, CRC flag.
                               Transmitted at "reduced rate" (only SF-2 = 6 bits
                               per symbol are used in the interleaving matrix),
                               but still occupies 4+CR = 8 symbols on air.
                               Looks like normal data symbols in the table.

  Symbols 20+                 PAYLOAD
                               The actual encoded data bytes. Each row of the
                               interleaving matrix produces one chip value here.
                               For a 1-byte payload at CR=4/8 and SF=8:
                               2 nibbles x 1 codeword each, padded to SF=8
                               codewords → 8 payload symbols.

"""
import os
import numpy as np
from scipy.signal import resample_poly
from scipy.io import wavfile
import matplotlib.pyplot as plt
import argparse
import json

# ═══════════════════════════════════════════════════════════════════════
#  CONFIGURATION
# ═══════════════════════════════════════════════════════════════════════

IQ_FILE        = 'last_capture.npy'  # Path to raw IQ capture file. Accepted formats: .npy, .wav, .iq/.raw (rtl_sdr format)

# We intentionally tune 200 kHz ABOVE the actual LoRa frequency when capturing
# with the RTL-SDR. Reason: the RTL-SDR hardware produces a strong "DC spike"
# at its exact centre frequency. If we tuned directly to 869.525 MHz, the DC
# spike would sit right on top of our signal and corrupt it. By tuning 200 kHz
# higher, the LoRa signal lands at -200 kHz in the captured spectrum, safely
# away from the DC spike. F_OFFSET corrects for this in software when processing
# the saved capture.
CENTER_FREQ    = 869.725e6          # RTL-SDR tune freq used for captures = LoRa freq + 200 kHz
F_OFFSET       = -200_000           # Software correction: shift signal back down
                                    # by 200 kHz to put it at baseband (0 Hz)

SF             = 8                  # Spreading Factor
                                    # Determines: symbol length N = 2^SF. E.g. from SF8 N = 256 samples
                                    # Determines: chip value range = 0 to 2^SF-1. E.g. from SF8, 0-255.
                                    # Determines: symbol duration = N/BW. E.g. from SF8 and BW=125kHz, 256/125000 = 2.048 ms per symbol.

BW             = 125_000            # LoRa channel bandwidth in Hz = chip rate
                                    # At this rate, one symbol = exactly 2^SF samples

SAMPLE_RATE    = 2_400_000          # RTL-SDR sample rate in samples/sec
                                    # Downsample ratio to chip rate: 2400000/125000 = 19.2
                                    # NOTE: SAMPLE_RATE is the rate used during the original capture. Keep this
                                    #       in sync with how you recorded the data (e.g. with rtl_sdr -s 2400000).

N_SYMBOLS      = 32                 # Symbols to analyse and print.
                                    # Full SF=8 frame: ~2(pre-burst noise) + 8(preamble) + 2(sync) + 3(downchirp) + 8(header) + 8(payload) = 31 min
                                    #
                                    # NOTE: HARDCODED for SF=8, CR=4, 1-byte payload. For longer payloads or different SF/CR, increase
                                    #       this value.  Ideally compute dynamically from header decode results.

PLOT           = True               # Show matplotlib plots

# ═══════════════════════════════════════════════════════════════════════
#  FILE HANDLING FUNCTIONS
# ═══════════════════════════════════════════════════════════════════════

def load_from_file(path: str) -> np.ndarray:
    # Load a saved capture from disk and convert it to complex64 IQ samples.
    # Supported formats:
    #  - .npy : numpy array of complex samples (preferred; fastest to load)
    #  - .wav : two-channel WAV where left=I right=Q (float or int types)
    #  - .iq  : raw unsigned 8-bit interleaved I/Q produced by rtl_sdr
    if not os.path.exists(path):
        raise FileNotFoundError(f"IQ file not found: {path}")

    if path.endswith('.npy'):
        data = np.load(path)
        # Ensure complex dtype
        if np.iscomplexobj(data):
            return data.astype(np.complex64)
        # If user saved real+imag interleaved, try to interpret:
        if data.ndim == 1 and data.dtype.kind in ('f', 'i') and data.size % 2 == 0:
            f = data.astype(np.float32)
            iq = f[0::2] + 1j * f[1::2]
            return iq.astype(np.complex64)
        raise ValueError("Unsupported .npy layout: expected complex array or interleaved real/imag.")

    elif path.endswith('.wav'):
        rate, data = wavfile.read(path)
        # Normalize integer WAVs to [-1, +1]; float WAVs assumed already scaled
        if data.dtype.kind in ('i', 'u'):
            # find max possible value for this integer type
            maxv = float(np.iinfo(data.dtype).max)
            data = data.astype(np.float32) / maxv
        elif data.dtype.kind == 'f':
            data = data.astype(np.float32)

        if data.ndim == 2:
            # stereo WAV where channel0 = I, channel1 = Q
            return (data[:, 0] + 1j * data[:, 1]).astype(np.complex64)
        # mono WAV: treat samples as real part only
        return data.astype(np.complex64)

    elif path.endswith('.iq') or path.endswith('.raw'):
        # raw rtl_sdr output: unsigned 8-bit interleaved I/Q
        raw = np.fromfile(path, dtype=np.uint8)
        if raw.size == 0:
            raise ValueError(f"No data in IQ file: {path}")
        if raw.size % 2 != 0:
            raw = raw[:-1]
        # convert to float32 in [-1, 1]
        f = (raw.astype(np.float32) - 127.5) / 127.5
        iq = f[0::2] + 1j * f[1::2]
        return iq.astype(np.complex64)

    raise ValueError(f"Unknown file type: {path}")

# ═════════════════════════════════════════════════════════════════════==
#  SIGNAL PROCESSING FUNCTIONS
# ═════════════════════════════════════════════════════════════════════==
#
# The following functions perform the core DSP:
#  - make_base_upchirp: generates the reference upchirp used to dechirp symbols
#  - dechirp: multiplies incoming symbol by conj(reference) and computes FFT
#  - get_peak_bin: parabolic interpolation around the FFT peak to estimate bin
# ═════════════════════════════════════════════════════════════════════==

def make_base_upchirp(sf: int, bw: float) -> np.ndarray:
    """
    Generate one period of the LoRa base upchirp: N = 2^SF complex samples.

    The chirp sweeps linearly from -BW/2 to +BW/2 over N samples.
    Mathematical form (Robyns et al. 2018, Eq. 1):
        x[n] = exp(j * 2π * (f0 * n/BW  +  ½ * k * (n/BW)²))
    where f0 = -BW/2  and  k = BW²/N

    This is the REFERENCE signal we use to dechirp received symbols.
    We keep one copy and reuse it for every symbol — it never changes.
    """
    N  = 1 << sf
    t  = np.arange(N) / bw   # time axis: [0, 1/BW, 2/BW, ..., (N-1)/BW]
    f0 = -bw / 2.0
    k  = bw ** 2 / N
    return np.exp(1j * 2 * np.pi * (f0 * t + 0.5 * k * t ** 2))


def dechirp(symbol_samples: np.ndarray, base_upchirp: np.ndarray) -> np.ndarray:
    """
    Dechirp one symbol window and return the FFT magnitude spectrum.
    The index of the largest value = the chip value encoded in this symbol.

    HOW IT WORKS:
    A LoRa symbol with chip value C is a base chirp circularly shifted by C:
        symbol_C[n] = base_chirp[(n + C) mod N]

    In frequency domain, this time shift becomes a phase ramp. When we
    multiply symbol_C by conj(base_chirp), the chirp sweep cancels out and
    we're left with a pure complex sinusoid oscillating at frequency C/N:
        symbol_C[n] * conj(base_chirp[n])  ≈  exp(j * 2π * C/N * n)

    The FFT of a pure sinusoid at frequency C/N has a single spike at bin C.
    So argmax(|FFT(...)|) gives us C directly.

    WHY THIS FAILS FOR DOWNCHIRPS:
    A downchirp is conj(base_chirp). Multiplying by conj(base_chirp) gives:
        conj(base_chirp) * conj(base_chirp) = conj(base_chirp)²
    This is NOT a pure sinusoid — it's a double-speed chirp, which the FFT
    sees as energy smeared across all bins. Hence low PMR for downchirps.
    """
    return np.abs(np.fft.fft(symbol_samples * np.conj(base_upchirp)))


def get_peak_bin(spectrum: np.ndarray) -> float:
    # Parabolic interpolation around the peak gives sub-bin resolution.
    N = len(spectrum)
    k = np.argmax(spectrum)
    y0 = spectrum[(k - 1) % N]
    y1 = spectrum[k]
    y2 = spectrum[(k + 1) % N]
    denom = y0 + y2 - 2 * y1
    if abs(denom) < 1e-12:
        return float(k)
    delta = (y0 - y2) / (2 * denom)
    return (k + delta) % N

# ═════════════════════════════════════════════════════════════════════==
#  MAIN
# ═════════════════════════════════════════════════════════════════════==
def main():
    # ── CLI argument parsing ──────────────────────────────────────────
    parser = argparse.ArgumentParser(
        description="LoRa Symbol Observer — dechirp IQ capture and extract chip values."
    )
    parser.add_argument(
        '--no-plot', action='store_true', default=False,
        help='Suppress matplotlib plots (for headless / pipeline use).'
    )
    parser.add_argument(
        '--save-symbols', type=str, default=None, metavar='PATH',
        help='Save extracted symbol data to a JSON file at PATH.'
    )
    parser.add_argument(
        '--iq-file', type=str, default=None,
        help=f'Override IQ_FILE config (default: {IQ_FILE}).'
    )
    args = parser.parse_args()

    show_plots = PLOT and not args.no_plot
    save_path  = args.save_symbols
    iq_file    = args.iq_file if args.iq_file else IQ_FILE

    N = 1 << SF   # 256 samples per symbol at chip rate for SF=8

    # ── 1. Acquire (file-based) ────────────────────────────────────────
    print(f"  Loading {iq_file} …")
    raw = load_from_file(iq_file)

    # Quick sanity check: ensure we loaded something reasonable
    if raw is None or len(raw) == 0:
        raise RuntimeError("Loaded IQ data is empty. Check the capture file.")

    if len(raw) < N:
        print(f"Warning: loaded capture contains fewer than one symbol ({len(raw)} samples, need >= {N}).")
        # continue anyway; downstream code will handle it and likely report "end of signal"

    # Remove DC offset.
    # The capture device leaks a constant (DC) component into both I and Q.
    # In the frequency domain this appears as a spike at exactly 0 Hz.
    # Subtracting the mean of the entire capture removes it cleanly.
    raw -= np.mean(raw)
    print(f"  Samples: {len(raw)}  ({len(raw)/SAMPLE_RATE:.2f} s)")

    # ── 2. Frequency shift + downsample ──────────────────────────────
    print(f"Downsampling {SAMPLE_RATE//1000} kHz → {BW//1000} kHz …")

    # Frequency shift: multiply every sample by exp(-j*2π*F_OFFSET*t).
    # This rotates the entire spectrum by -F_OFFSET Hz. If the capture was
    # performed with CENTER_FREQ = LoRa_freq + 200 kHz, setting
    # F_OFFSET = -200_000 moves the LoRa signal back to baseband (0 Hz).
    t        = np.arange(len(raw)) / SAMPLE_RATE
    baseband = raw * np.exp(-1j * 2 * np.pi * F_OFFSET * t)

    # Rational downsample from SAMPLE_RATE to BW.
    # ────────────────────────────────────────────────────────────────────
    # WHY: A LoRa symbol is a chirp that sweeps linearly across the entire
    # bandwidth BW (from -BW/2 to +BW/2). Each symbol carries N = 2^SF samples.
    # For the dechirping algorithm to work optimally — and to extract chip
    # values with full frequency resolution — we need:
    #
    #     sample_rate = BW
    #
    # This ensures ONE SYMBOL = EXACTLY N SAMPLES, so the downstream FFT
    # has perfect alignment:
    #
    #   • After dechirping, a symbol with chip value C becomes a pure
    #     complex sinusoid at frequency: f_C = C * (BW / N)
    #
    #   • The FFT bin spacing (for N samples at rate BW) is: Δf = BW / N
    #
    #   • Therefore: sinusoid at f_C lands EXACTLY in FFT bin C
    #
    # PAYOFF: All energy concentrates in ONE bin (no spreading into neighbors),
    # maximizing SNR and ensuring argmax(FFT) directly gives the chip value
    # with zero ambiguity.
    #
    #   • KEY INSIGHT: 1-to-1 mapping: Chip value C → FFT bin C (no decimals, no spreading across bins)
    #
    # HOW: Compute the rational resampling ratio 2,400,000 / 125,000 = 19.2 = 96/5
    # resample_poly(signal, up, down) upsamples by 'up' (×5) then downsamples
    # by 'down' (÷96), applying a proper low-pass anti-aliasing filter at the
    # intermediate rate. Net effect: divide sample rate by 19.2 with zero aliasing.
    # Result: one LoRa symbol = exactly N = 256 samples at the chip rate.
    lcm      = 5   # lcm = least common multiple of 5 and 96, to achieve exact rational resampling
    up       = lcm                          # ×5
    down     = int(SAMPLE_RATE / BW * lcm)  # ÷96  (net effect: ÷19.2)
    chip_sig = resample_poly(baseband, up, down).astype(np.complex64)
    print(f"  Chip-rate samples: {len(chip_sig)}")

    # ── 3. Find exact symbol boundary ─────────────────────────────────
    # CHALLENGE: LoRa symbol boundaries don't align to arbitrary N-sample
    # block boundaries. When we captured this signal, the true symbols started
    # at some unknown fractional offset within our block grid. We need to find
    # that offset(0 to N-1 samples) to align our slicing correctly for
    # dechirping. Misalignment → chirp doesn't cancel cleanly → corrupted bins.
    #
    # STRATEGY: The preamble (first 8 unmodulated chirps) ALL encode chip
    # value 0 by definition. If we're aligned correctly, all preamble symbols
    # will dechirp to FFT bin values very close to 0 (or offset consistently
    # due to capture device frequency error, but still CONSISTENT across all 8).
    # If we're misaligned, the preamble bins will scatter randomly across 0-N.
    # We try all 256 possible offsets and pick the one where preamble bins
    # are most tightly clustered.
    #

    # STEP 1: Coarse burst detection via energy
    # ──────────────────────────────────────────────────────────────────────
    # Divide the signal into N-sample blocks and compute mean power in each.
    # This gives us a rough "loudness map": where is the LoRa transmission?
    energies = np.array([
        np.mean(np.abs(chip_sig[i : i + N]) ** 2)
        for i in range(0, len(chip_sig) - N, N)
    ])

    if energies.size == 0:
        print("No full symbols found in capture after downsampling. Exiting.")
        return

    print(f"Max energy at symbol: {np.argmax(energies)}")
    print(f"Noise floor (median): {np.median(energies):.4f}")
    print(f"Peak energy:          {np.max(energies):.4f}")

    # STEP 2: Find burst edge by walking back from peak
    # ──────────────────────────────────────────────────────────────────────
    # Starting at the loudest symbol, walk backward until energy drops below
    # 10x the noise floor. This marks the rising edge of the burst (where the
    # preamble begins). We stop walking when we hit a symbol that's "quiet enough"
    # to be pre-burst noise. This gives us a "rough start" aligned to N-sample
    # block boundaries.
    noise_floor     = np.median(energies)
    burst_peak      = np.argmax(energies)
    burst_thresh    = noise_floor * 10

    burst_start_sym = burst_peak
    for i in range(burst_peak, 0, -1):
        if energies[i] < burst_thresh:
            burst_start_sym = i
            break

    rough_start = burst_start_sym * N
    print(f"  Rough burst start: sample {rough_start}")

    # STEP 3: Fine-tune alignment by testing all sample offsets
    # ──────────────────────────────────────────────────────────────────────
    # The "rough start" is aligned to N-sample block boundaries, but the true
    # symbol start is unknown. We try every offset (offset = 0 to N-1) and for
    # each one, we dechirp symbols 2-9 (the preamble).
    #
    # For each offset, we compute the "circular variance" of the preamble bin
    # values. The key insight: unmodulated preambles should all be the same bin
    # (or very close), so circular variance should be LOW for correct alignment
    # and HIGH for misalignment.
    #
    # Why circular variance instead of regular variance?
    # Bins wrap around at N. If preamble bins are [255, 254, 0, 1, 2], regular
    # arithmetic mean ≈ 102.4 — nonsense! Circular mean: convert to angles on
    # a unit circle (bin b → angle 2π*b/N), average the angles, convert back.
    # Result: mean angle ≈ 0° → bin 0, which is correct.
    #
    # Circular variance (0 = perfect, 1 = random) is computed as:
    #   r = |mean(exp(j*angle))|  (resultant vector length)
    #   variance = 1 - r
    # If all angles point the same direction → r ≈ 1 → variance ≈ 0
    # If all angles are random → r ≈ 0 → variance ≈ 1
    print("  Scanning for symbol boundary alignment …")
    base_up       = make_base_upchirp(SF, BW)
    best_offset   = 0
    best_variance = np.inf

    for offset in range(N):
        trial_start = rough_start + offset
        bins = []
        for sym in range(2, 10):  # symbols 2-9 = preamble of length 8
            s0 = trial_start + sym * N
            if s0 + N > len(chip_sig):
                break
            spectrum = dechirp(chip_sig[s0 : s0 + N], base_up)
            bins.append(get_peak_bin(spectrum))

        if len(bins) < 6:
            continue

        # Compute circular mean and variance of bin angles
        angles   = [2 * np.pi * b / N for b in bins]
        mean_sin = np.mean([np.sin(a) for a in angles])
        mean_cos = np.mean([np.cos(a) for a in angles])
        r        = np.sqrt(mean_sin**2 + mean_cos**2)  # resultant vector length
        circ_var = 1 - r   # 0 = perfectly consistent, 1 = random

        if circ_var < best_variance:
            best_variance = circ_var
            best_offset   = offset

    start = rough_start + best_offset
    print(f"  Best alignment offset: {best_offset} samples  "
          f"(circular variance: {best_variance:.4f})")
    print(f"  Aligned symbol start:  sample {start}")


    # ── 4. Walk through symbols ───────────────────────────────────────
    # OVERVIEW: Iterate through the transmission one symbol at a time.
    # For each symbol, dechirp and extract the peak FFT bin (chip value) and
    # the Peak/Mean ratio (signal quality metric). Collect all this data for
    # frequency calibration in section 4b.
    #
    # TIMING COMPENSATION FOR SFD (Start of Frame Delimiter):
    # The LoRa frame structure is:
    #   Preamble (8 symbols) → Sync/Frame sync (2 symbols) → SFD (2 downchirps)
    #   → Data/Header/Payload
    #
    # The SFD downchirps have special timing: the receiver aligns symbol
    # boundaries based on the SFD phase offset. The consequence: data symbols
    # that come AFTER the SFD are shifted by 0.25 symbol (N//4 samples) relative
    # to the preamble-aligned grid. This is a known characteristic of the LoRa
    # modulation and receiver behavior.
    #
    # Implementation: After symbol 14 (= 2 preamble + 2 sync + 2 SFD downchirps,
    # with 2 pre-burst symbols before preamble starts), we add N//4 to the window
    # start position to account for this phase shift in data extraction.
    #
    # HARDCODED FRAME STRUCTURE: This assumes SF=8, CR=4, preamble length 8,
    # and 2 pre-burst noise symbols. Different configurations will shift these
    # boundaries. This should be computed from detected preamble position and
    # header decode (TODO).
    PREAMBLE_START  = 2          # Preamble starts at symbol 2 (after 2 pre-burst symbols)
    PREAMBLE_LEN    = 8          # Preamble is 8 unmodulated chirps
    SFD_END_SYM     = PREAMBLE_START + PREAMBLE_LEN + 2 + 2   # = 14 (after 2 sync + 2 SFD downchirps)
    sfd_quarter_shift = N // 4   # Apply N//4 sample shift to data symbols after SFD

    print()
    print(f"{'Sym':>4}  {'Start':>8}  {'Peak bin':>8}  {'Peak/Mean':>10}  Notes")
    print("-" * 60)

    spectra    = []              # FFT spectrum for each symbol (saved for the waterfall plot)
    chip_values = []             # Peak bin of each symbol (raw, before frequency calibration)
    peak_means = []              # Peak/Mean ratio for each symbol (signal quality metric)

    for i in range(N_SYMBOLS):
        # Determine window start position, accounting for SFD timing shift
        if i >= SFD_END_SYM:
            s0 = start + i * N + sfd_quarter_shift  # Data symbols: apply N//4 shift
        else:
            s0 = start + i * N                       # Preamble/Sync/SFD: no shift
        s1 = s0 + N
        if s1 > len(chip_sig):
            print(f"{i:4}  (end of signal)")
            break

        # Dechirp this symbol window and extract chip value
        spectrum  = dechirp(chip_sig[s0:s1], base_up)     # FFT after dechirping
        peak_bin  = get_peak_bin(spectrum)                 # Find peak with sub-bin resolution
        
        # Peak/Mean ratio (PMR): Signal quality metric
        # ────────────────────────────────────────────────────────────────────
        # WHY: After dechirping a clean LoRa symbol (upchirp), all signal energy
        # should concentrate in ONE FFT bin (the chip value). Noise spreads evenly
        # across all bins. PMR measures how "peaky" the spectrum is.
        #
        # INTERPRETATION:
        #   PMR = peak bin value / mean bin value
        #   Pure sinusoid (clean symbol): peak/mean = N = 256 (all energy in 1 of 256 bins)
        #   Pure white noise: peak/mean ≈ 1 (equal energy in all bins)
        #   Real signals: PMR 20-50 for high-quality symbols, 5-20 for degraded,
        #                 < 5 suggests noise or misalignment (no real signal)
        #
        # Use case: PMR filters for the preamble search in 4b. High PMR symbols
        # are more reliable for calibration because the chip value is less corrupted
        # by noise.
        peak_mean = float(np.max(spectrum) / (np.mean(spectrum) + 1e-12))
        
        spectra.append(spectrum)      # Save for waterfall plot
        chip_values.append(peak_bin)  # Save for frequency calibration
        peak_means.append(peak_mean)  # Save for preamble detection quality filtering

        # Heuristic annotations based on symbol index and PMR
        # These index ranges are HARDCODED for SF=8, CR=4, 1-byte payload.
        #   A different SF/CR/payload length will shift the boundaries.
        #   TODO: compute from detected preamble position + header decode.
        if i <= 1:
            note = '← possible pre-burst noise'
        elif 2 <= i <= 9 and peak_mean > 10:
            note = f'← preamble upchirp (bin {peak_bin:.0f} repeating)'
        elif 10 <= i <= 11:
            note = '← sync symbol'
        elif 12 <= i <= 13:
            note = '← downchirp (low PMR expected)'
        elif 14 <= i <= 21:
            note = '← header'
        elif peak_mean < 5.0:
            note = '← noise / no signal'
        else:
            note = '← payload'

        print(f"{i:4}  {s0:8}  {peak_bin:8.1f}  {peak_mean:10.1f}  {note}")

    # ── 4b. Frequency calibration from preamble ───────────────────────
    # GOAL: Measure and correct the capture device's frequency error so that
    # chip values are on their true intended positions.
    #
    # PHYSICS:
    # The preamble consists of 8 UNMODULATED base upchirps (chip value = 0).
    # In a perfect capture device (no frequency error), all preamble symbols
    # would dechirp to FFT bin 0. But real devices have frequency error — an
    # oscillator offset (thermal drift, aging, manufacturing tolerance) that
    # shifts ALL signal frequencies by a constant amount.
    #
    # MEASUREMENT:
    # If all preamble bins appear at offset δ instead of 0, that offset δ
    # (in bins) corresponds to a frequency error of δ × (BW / N) Hz.
    # We measure δ using circular mean (to handle wraparound at N), then
    # subtract δ from all observed chip values to recover the true encoded values.
    #
    # STRATEGY: Find the preamble run (longest consecutive sequence of symbols
    # with high signal quality and consistent bin value), then compute the
    # circular mean offset of those bins.
    print(f"\nFrequency calibration (from preamble):")

    # Step 1: Dynamically locate the preamble in the observation list
    # ────────────────────────────────────────────────────────────────────────
    # Why search instead of just using symbols 2-9 (hardcoded preamble position)?
    # The rough symbol alignment (Step 3 earlier) may have found boundaries
    # slightly off from the true transmitted boundaries. We search the captured
    # data for the longest run of symbols that look like unmodulated chirps:
    #   • HIGH PMR (≥20): Indicates strong, clean signal (not noise)
    #   • LOW BIN VARIANCE: All symbols have the same (or very similar) bin values,
    #     meaning they all encode the same chip value (which must be 0 for preamble)
    #
    # We search for runs of length 4-11 symbols within the first N_SYMBOLS of
    # the detected transmission. The longest such run is most likely the true
    # preamble (8 symbols, but we allow 4-11 to be robust to partial detections).
    min_pmr = 20.0           # Threshold for "high quality" signal
    max_var = 1.0            # Maximum allowed variance of bin values in a run
    best_start = 0
    best_len = 0
    best_var = np.inf
    for i in range(N_SYMBOLS - 4):  # Search starting positions
        for l in range(4, min(12, N_SYMBOLS - i)):  # Search run lengths (4-11 symbols)
            run_bins = chip_values[i:i + l]
            run_pmrs = peak_means[i:i + l]
            
            # Filter 1: All symbols in run must have PMR ≥ 20
            if np.min(run_pmrs) < min_pmr:
                continue  # Any symbol with low PMR ruins the run quality
            
            # Filter 2: Bin values must be tightly clustered (low variance)
            # Note: uses regular variance, not circular variance. This assumes
            # frequency error is small enough that bins don't wrap the 0-N boundary.
            # For typical frequency errors (few ppm), this is safe. However, if
            # error is large (rare case), regular variance could miss wraparound.
            run_var = np.var(run_bins)
            if run_var < max_var and l > best_len:
                # Keep the longest run with low variance
                best_len = l
                best_start = i
                best_var = run_var

    if best_len == 0:
        print("  No preamble found! Could not calibrate.")
        return

    print(f"  Detected preamble at symbols {best_start} – {best_start + best_len - 1} (variance {best_var:.4f})")
    preamble_bins = chip_values[best_start:best_start + best_len]

    # Step 2: Compute circular mean of preamble bin values
    # ────────────────────────────────────────────────────────────────────────
    # The preamble bins should all be equal (chip value 0), but due to frequency
    # error, they're offset by some amount δ. We compute δ using circular mean
    # (same technique as in Step 3 earlier) to handle wraparound at bin N.
    #
    # MATH:
    #   • Convert each bin to an angle: θ_b = 2π * b / N  (maps 0 to 2π)
    #   • Average the sine and cosine of angles
    #   • Recover mean angle: φ = atan2(mean_sin, mean_cos)
    #   • Convert back to bin: δ_bins = φ * N / (2π) mod N
    #
    # RESULT: δ_bins is the frequency offset in units of FFT bins.
    angles   = [2 * np.pi * b / N for b in preamble_bins]
    mean_sin = np.mean(np.sin(angles))
    mean_cos = np.mean(np.cos(angles))
    freq_offset_bins = (np.arctan2(mean_sin, mean_cos) * N / (2 * np.pi)) % N

    # Step 3: Convert bins → Hz → ppm (for user readability)
    # ────────────────────────────────────────────────────────────────────────
    # Each FFT bin represents a frequency span of BW/N Hz, so:
    #   frequency error [Hz] = offset [bins] × (BW [Hz] / N)
    # Example: offset = 2 bins, BW = 125 kHz, N = 256
    #   freq_error = 2 × (125000 / 256) ≈ 976 Hz
    #
    # PPM (parts per million) is the relative frequency error:
    #   error [ppm] = error [Hz] / center frequency [Hz] × 1e6
    freq_offset_hz  = freq_offset_bins * BW / N
    freq_offset_ppm = freq_offset_hz / CENTER_FREQ * 1e6

    print(f"  Preamble bins (raw) : {[round(b, 1) for b in preamble_bins]}")
    print(f"  Measured offset      : {freq_offset_bins:.1f} bins")
    print(f"  Offset in Hz         : {freq_offset_hz:.1f} Hz")
    print(f"  Capture device ppm error    : {freq_offset_ppm:.2f} ppm")

    # Step 4: Apply correction to all chip values
    # ────────────────────────────────────────────────────────────────────────
    # Each observed chip value is shifted by offset_bins due to frequency error.
    # Subtract the offset from all observations, wrapping at N.
    #
    # WHY IT WORKS:
    # If true chip value C was transmitted, but we observe (C + offset) mod N,
    # then subtracting offset recovers C (mod N). The modulo operation ensures
    # wraparound: if we observe C=250 and offset=10, true value is (250-10) mod 256 = 240.
    # If we observe C=5 and offset=10, true value is (5-10) mod 256 = 251 (wraps).
    #
    # EXPECTATION AFTER CORRECTION:
    # • Preamble symbols: should read ~0 (unmodulated base chirps)
    # • Data/Payload symbols: should read their true intended chip values
    corrected = [(c - freq_offset_bins) % N for c in chip_values]

    print(f"\nCorrected chip values (offset subtracted):")
    print(f"{'Sym':>4}  {'Raw bin':>7}  {'Corrected':>9}  Notes")
    print("-" * 45)
    for i, (raw_bin, cor) in enumerate(zip(chip_values, corrected)):
        if   i < best_start:
            note = '(pre-burst)'
        elif i < best_start + best_len:
            note = '← preamble (should be 0)'
        elif i < best_start + best_len + 2:
            note = '← sync symbol'
        elif i < best_start + best_len + 4:
            note = '← downchirp / freq-sync'
        else:
            note = ''
        print(f"{i:4}  {raw_bin:7.1f}  {cor:9.1f}  {note}")

    # ── 4c. Save symbol data to JSON ─────────────────────────────────
    if save_path:
        symbols = []
        for i, (raw_bin, cor, pmr) in enumerate(zip(chip_values, corrected, peak_means)):
            # Classify each symbol's role in the frame
            if i < best_start:
                role = 'pre-burst'
            elif i < best_start + best_len:
                role = 'preamble'
            elif i < best_start + best_len + 2:
                role = 'sync'
            elif i < best_start + best_len + 4:
                role = 'downchirp'
            elif i < best_start + best_len + 4 + (4 + 4):
                role = 'header'   # header always uses CR=4 → 4+4 = 8 symbols per block
            else:
                role = 'payload'

            symbols.append({
                'index':           i,
                'raw_bin':         round(raw_bin, 2),
                'corrected_bin':   round(cor, 2),
                'peak_mean_ratio': round(pmr, 2),
                'role':            role,
            })

        output = {
            'config': {
                'sf':           SF,
                'bw':           BW,
                'sample_rate':  SAMPLE_RATE,
                'center_freq':  CENTER_FREQ,
                'f_offset':     F_OFFSET,
                'n_bins':       N,
            },
            'calibration': {
                'freq_offset_bins': round(freq_offset_bins, 3),
                'freq_offset_hz':   round(freq_offset_hz, 2),
                'ppm_error':        round(freq_offset_ppm, 3),
                'preamble_start':   best_start,
                'preamble_length':  best_len,
            },
            'symbols': symbols,
        }

        with open(save_path, 'w') as f:
            json.dump(output, f, indent=2)
        print(f"\nSymbol data saved to: {save_path}")

    # ── 5. Plots ──────────────────────────────────────────────────────
    if show_plots and spectra:

        # SPECTROGRAM — raw signal, no dechirping
        # Shows the actual frequency content of each symbol over time.
        # Upchirps appear as diagonal lines sweeping UPWARD (low→high freq).
        # Downchirps appear as diagonal lines sweeping DOWNWARD (high→low freq).
        # All upchirps have the same slope — they differ only in their
        # starting position (= the chip value = the encoded data).
        view = chip_sig[start : start + N_SYMBOLS * N]
        nfft    = N // 2        # smaller window = better time resolution for chirps
        overlap = nfft * 3 // 4 # 75% overlap = smooth continuous lines

        plt.figure(figsize=(14, 5))
        plt.specgram(view, NFFT=nfft, Fs=BW, noverlap=overlap,
                    cmap='inferno', sides='twosided',
                    vmin=-80, vmax=0)   # adjust vmin if signal disappears or noise dominates
        plt.title("Spectrogram — upward diagonals = upchirps  |  downward = downchirps")
        plt.xlabel("Time (s)")
        plt.ylabel("Frequency (Hz)")
        plt.colorbar(label="dB")
        plt.tight_layout()
        plt.show()

        # DECHIRPED FFT WATERFALL
        # Each row = one symbol after dechirping.
        # After dechirping an upchirp, energy concentrates in ONE bin.
        # That bin's column position = the chip value.
        # Preamble:     8 rows with bright dot in the SAME column
        # Sync symbols: 2 rows with bright dots at DIFFERENT columns
        # Downchirps:   rows with NO single bright dot (energy smeared)
        # Header/data:  rows with bright dots at varying positions
        mat = np.array(spectra)
        plt.figure(figsize=(14, 8))
        plt.imshow(20 * np.log10(mat + 1e-12), aspect='auto',
                   origin='upper', cmap='inferno',
                   extent=[0, N, N_SYMBOLS, 0])
        plt.title("Dechirped FFT waterfall\n"
                  "Each row = one symbol  |  "
                  "Bright dot = chip value  |  "
                  "Smeared rows = downchirps (expected)")
        plt.xlabel(f"FFT bin  (chip value, 0 – {(1<<SF)-1})")
        plt.ylabel("Symbol index")
        plt.colorbar(label="dB")
        plt.tight_layout()
        plt.show()


if __name__ == '__main__':
    main()