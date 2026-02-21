#!/usr/bin/env python3
"""
LoRa Symbol Observer — Annotated Version
=========================================
This script answers one question: "what chip value is each symbol?"

A LoRa symbol is a chirp (a signal that sweeps from low to high frequency
over time). Each symbol carries information by starting that sweep at a
different frequency offset — that offset is called the "chip value" and
is an integer from 0 to 2^SF - 1 (so 0–255 for SF=8).

This script:
  1. Captures raw IQ samples from the RTL-SDR
  2. Shifts the signal to baseband and downsamples to the LoRa chip rate
  3. Finds where in the recording the transmission actually is
  4. Walks through the signal one symbol at a time and prints the chip value
     of each symbol

That's it. No decoding, no preamble detection algorithm, no assumptions
about frame structure. We just observe and describe what we see.

HOW TO READ THE OUTPUT TABLE
──────────────────────────────
  Sym       — symbol index counting from the detected burst start (0 = first)
  Start     — sample number in the chip-rate signal where this symbol begins
  Peak bin  — the chip value: which FFT bin had the most energy after dechirping.
              This is the "message" encoded in this symbol (0–255 for SF=8).
  Peak/Mean — signal quality ratio: peak bin energy divided by average bin energy.
              A clean single-frequency tone after dechirping concentrates all its
              energy into ONE bin, so a clean symbol has a very high ratio.
              > 20  = very clean symbol, chip value is reliable
              5–20  = usable but some noise present
              < 5   = noise only, no real signal here, chip value is meaningless

WHAT TO EXPECT IN A VALID LORA FRAME (SF=8, CR=4/8)
──────────────────────────────────────────────────────
  Symbols 0–7   (8 symbols)   PREAMBLE
                               All 8 should show the SAME peak bin with PMR > 20.
                               The value is ideally bin 0 (unmodulated base chirp)
                               but may be a small consistent offset due to the
                               RTL-SDR's inherent frequency error (~few ppm).

  Symbols 8–9   (2 symbols)   FRAME SYNC (sync word)
                               Two symbols with DIFFERENT bin values from the
                               preamble and from each other. These encode the
                               network sync word nibbles, left-shifted by SF-4.
                               e.g. sync word 0x7F → hi nibble 0x7, lo nibble 0xF
                               → symbol bins: 0x7 << 4 = 112  and  0xF << 4 = 240

  Symbols 10–12 (~3 symbols)  FREQUENCY SYNC (downchirps)
                               Two full downchirps + one quarter downchirp.
                               A downchirp sweeps HIGH to LOW frequency — the
                               opposite of our base upchirp. When we dechirp
                               a downchirp with an upchirp template, the chirp
                               does NOT cancel out cleanly, so energy spreads
                               across ALL bins instead of concentrating in one.
                               Result: these symbols show LOW PMR. This is not
                               an error — it is the expected and correct behaviour.

  Symbols 13–18 (6 symbols)   HEADER  (SF-2 = 6 symbols for SF=8)
                               Contains payload length, coding rate, CRC flag.
                               Transmitted at "reduced rate" (fewer rows in the
                               interleaving matrix) for extra robustness.
                               Looks like normal data symbols in the table.

  Symbols 19+                 PAYLOAD
                               The actual encoded data bytes. Each row of the
                               interleaving matrix produces one chip value here.
                               For a 1-byte payload at CR=4/8 and SF=8:
                               2 nibbles × 1 codeword each, padded to SF=8
                               codewords → 8 payload symbols.
"""

import numpy as np
from scipy.signal import resample_poly
from scipy.io import wavfile
import matplotlib.pyplot as plt
import sys

# ═══════════════════════════════════════════════════════════════════════
#  CONFIGURATION
# ═══════════════════════════════════════════════════════════════════════

MODE           = 'file'              # 'sdr' = live capture from RTL-SDR
                                    # 'file' = load previously saved .npy file
IQ_FILE        = 'last_capture.npy' # used when MODE = 'file'

# We intentionally tune 200 kHz ABOVE the actual LoRa frequency.
# Reason: the RTL-SDR hardware produces a strong "DC spike" at its exact
# centre frequency. If we tuned directly to 869.525 MHz, the DC spike
# would sit right on top of our signal and corrupt it.
# By tuning 200 kHz higher, the LoRa signal lands at -200 kHz in our
# captured spectrum, safely away from the DC spike.
# F_OFFSET then corrects for this in software.
CENTER_FREQ    = 869.725e6          # RTL-SDR tune freq = LoRa freq + 200 kHz
F_OFFSET       = -200_000           # Software correction: shift signal back down
                                    # by 200 kHz to put it at baseband (0 Hz)

SF             = 8                  # Spreading Factor
                                    # Determines: symbol length N = 2^SF = 256 samples
                                    # Determines: chip value range = 0 to 2^SF-1 = 255
                                    # Determines: symbol duration = N/BW = 2.048 ms

BW             = 125_000            # LoRa channel bandwidth in Hz = chip rate
                                    # At this rate, one symbol = exactly 2^SF samples

SAMPLE_RATE    = 2_400_000          # RTL-SDR sample rate in samples/sec
                                    # Downsample ratio to chip rate: 2400000/125000 = 19.2

RECORD_SECONDS = 10                 # Capture duration. Heltec transmits every ~4s
                                    # so 10s guarantees catching at least 2 packets

SDR_GAIN       = 'auto'             # RTL-SDR gain. Try 30 or 40 if signal is weak.

N_SYMBOLS      = 30                 # Symbols to analyse and print.
                                    # Full SF=8 frame: ~8+2+3+6+8 = 27 symbols min

PLOT           = True               # Show matplotlib plots

# ═══════════════════════════════════════════════════════════════════════
#  CAPTURE FUNCTIONS
# ═══════════════════════════════════════════════════════════════════════

def capture_from_sdr(n_samples: int) -> np.ndarray:
    """
    Capture n_samples complex IQ samples from the RTL-SDR.

    IQ samples are pairs of (I=real, Q=imaginary) values representing the
    amplitude and phase of the received signal. The RTL-SDR outputs them
    as a stream of interleaved uint8 values which pyrtlsdr converts to
    complex128 (we cast to complex64 to save memory).

    Chunked reads prevent LIBUSB_ERROR_OVERFLOW on the RTL-SDR v4.
    """
    try:
        from rtlsdr import RtlSdr
    except ImportError:
        print("ERROR: pip install pyrtlsdr")
        sys.exit(1)

    sdr             = RtlSdr()
    sdr.sample_rate = SAMPLE_RATE
    sdr.center_freq = CENTER_FREQ
    sdr.gain        = SDR_GAIN

    CHUNK  = 32768  # Safe chunk size: 64KB of uint8 pairs, fits in USB buffer
    chunks = int(np.ceil(n_samples / CHUNK))
    print(f"  Capturing {chunks} × {CHUNK} samples at {CENTER_FREQ/1e6:.3f} MHz …")
    data = np.concatenate([sdr.read_samples(CHUNK) for _ in range(chunks)])
    sdr.close()
    return data[:n_samples].astype(np.complex64)


def load_from_file(path: str) -> np.ndarray:
    if path.endswith('.npy'):
        return np.load(path).astype(np.complex64)
    elif path.endswith('.wav'):
        rate, data = wavfile.read(path)
        if data.ndim == 2:
            return (data[:, 0] + 1j * data[:, 1]).astype(np.complex64)
        return data.astype(np.complex64)
    raise ValueError(f"Unknown file type: {path}")

# ═══════════════════════════════════════════════════════════════════════
#  SIGNAL PROCESSING FUNCTIONS
# ═══════════════════════════════════════════════════════════════════════

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


# ═══════════════════════════════════════════════════════════════════════
#  MAIN
# ═══════════════════════════════════════════════════════════════════════

def main():
    N = 1 << SF   # 256 samples per symbol at chip rate for SF=8

    # ── 1. Acquire ────────────────────────────────────────────────────
    print("Acquiring …")
    if MODE == 'sdr':
        raw = capture_from_sdr(int(RECORD_SECONDS * SAMPLE_RATE))
        np.save('last_capture.npy', raw)
        print("  Saved → last_capture.npy")
    else:
        print(f"  Loading {IQ_FILE} …")
        raw = load_from_file(IQ_FILE)

    # Remove DC offset.
    # The RTL-SDR hardware leaks a constant (DC) component into both I and Q.
    # In the frequency domain this appears as a spike at exactly 0 Hz.
    # Subtracting the mean of the entire capture removes it cleanly.
    raw -= np.mean(raw)
    print(f"  Samples: {len(raw)}  ({len(raw)/SAMPLE_RATE:.2f} s)")

    # ── 2. Frequency shift + downsample ──────────────────────────────
    print(f"Downsampling {SAMPLE_RATE//1000} kHz → {BW//1000} kHz …")

    # Frequency shift: multiply every sample by exp(-j*2π*F_OFFSET*t).
    # This rotates the entire spectrum by -F_OFFSET Hz. Since the LoRa
    # signal is currently sitting at F_OFFSET = -200 kHz (because we
    # tuned 200 kHz too high), rotating by +200 kHz moves it to 0 Hz.
    t        = np.arange(len(raw)) / SAMPLE_RATE
    baseband = raw * np.exp(-1j * 2 * np.pi * F_OFFSET * t)

    # Rational downsample from SAMPLE_RATE to BW.
    # 2,400,000 / 125,000 = 19.2 = 96/5
    # resample_poly(signal, up, down): upsamples by 'up' then downsamples
    # by 'down', applying a proper low-pass anti-aliasing filter.
    # After this, one LoRa symbol = exactly N = 256 samples.
    lcm      = 5
    up       = lcm                          # ×5
    down     = int(SAMPLE_RATE / BW * lcm) # ÷96  (net effect: ÷19.2)
    chip_sig = resample_poly(baseband, up, down).astype(np.complex64)
    print(f"  Chip-rate samples: {len(chip_sig)}")

    # ── 3. Locate the burst ───────────────────────────────────────────
    # Compute mean signal power for each block of N samples.
    # One block = one symbol duration, so this gives us a power reading
    # for each symbol position in the capture.
    energies = np.array([
        np.mean(np.abs(chip_sig[i : i + N]) ** 2)
        for i in range(0, len(chip_sig) - N, N)
    ])

    # Plot energy over time so we can visually verify the burst location.
    # You should see flat noise with 2-3 sharp spikes where the LoRa
    # packets are. Each spike is one full LoRa frame (~25 symbols wide).
    plt.figure(figsize=(14, 3))
    plt.plot(energies)
    plt.title("Signal energy per symbol — find where the burst is")
    plt.xlabel("Symbol index")
    plt.ylabel("Mean power")
    plt.tight_layout()
    plt.show()

    print(f"Max energy at symbol: {np.argmax(energies)}")
    print(f"Noise floor (median): {np.median(energies):.4f}")
    print(f"Peak energy:          {np.max(energies):.4f}")

    # Find the rising edge of the first burst.
    # - median = noise floor (robust: unaffected by the few loud symbols)
    # - threshold = 10× noise floor (signal must be clearly above noise)
    # - walk back from energy peak until we drop below threshold
    noise_floor     = np.median(energies)
    burst_peak      = np.argmax(energies)
    burst_thresh    = noise_floor * 10

    burst_start_sym = burst_peak
    for i in range(burst_peak, 0, -1):
        if energies[i] < burst_thresh:
            burst_start_sym = i
            break

    start = burst_start_sym * N
    print(f"  Burst starts at symbol {burst_start_sym} (sample {start})")

    # ── 4. Walk through symbols ───────────────────────────────────────
    base_up = make_base_upchirp(SF, BW)

    print()
    print(f"{'Sym':>4}  {'Start':>8}  {'Peak bin':>10}  {'Peak/Mean':>10}  Notes")
    print("-" * 60)

    spectra    = []
    chip_values = []   # raw chip values saved for calibration step below

    for i in range(N_SYMBOLS):
        s0 = start + i * N
        s1 = s0 + N
        if s1 > len(chip_sig):
            print(f"{i:4}  (end of signal)")
            break

        spectrum  = dechirp(chip_sig[s0:s1], base_up)
        peak_bin  = int(np.argmax(spectrum))
        # Peak/Mean ratio: how concentrated is the energy in one bin?
        # Pure tone → all energy in one bin → PMR = N = 256 (theoretical max)
        # Pure noise → energy spread evenly → PMR ≈ 1.0
        # Real signals: PMR 20-50 for clean chirps, <5 for noise
        peak_mean = float(np.max(spectrum) / (np.mean(spectrum) + 1e-12))
        spectra.append(spectrum)
        chip_values.append(peak_bin)

        # Heuristic annotations based on symbol index and PMR
        if   peak_mean < 5.0:
            note = '← noise / no signal'
        elif i <= 1:
            note = '← possible pre-burst noise'
        elif 2 <= i <= 9 and peak_mean > 10:
            note = f'← preamble upchirp (bin {peak_bin} repeating)'
        elif 10 <= i <= 11:
            note = '← sync symbol'
        elif 12 <= i <= 14:
            note = '← downchirp (low PMR expected)'
        elif 15 <= i <= 20:
            note = '← header'
        else:
            note = '← payload'

        print(f"{i:4}  {s0:8}  {peak_bin:10}  {peak_mean:10.1f}  {note}")

    # ── 4b. Frequency calibration from preamble ───────────────────────
    # The preamble is 8 unmodulated base chirps — they all carry chip
    # value 0 by definition. Any consistent offset from bin 0 is the
    # RTL-SDR's frequency error (oscillator drift + temperature).
    #
    # We measure this offset by averaging the preamble bin values,
    # then subtract it from all chip values to correct for it.
    #
    # WHY CIRCULAR MEAN:
    # Bins wrap around at N. If the offset is near 0 (e.g. bins 253,
    # 254, 255, 0, 1, 2), a naive arithmetic mean gives ~128 — totally
    # wrong. Circular mean converts bins to angles on a unit circle,
    # averages the angles, then converts back. This handles wraparound
    # correctly. For example bins [253,254,255,0,1,2] → mean angle ≈ 0°
    # → offset bin 0, which is correct.
    #
    # PREAMBLE_START: adjust this to match where your table shows the
    # first clean high-PMR repeating symbol. Typically symbol 2 or 3
    # based on the burst detection landing slightly early.
    PREAMBLE_START = 2   # ← adjust if needed based on table output
    PREAMBLE_LEN   = 8

    preamble_bins = chip_values[PREAMBLE_START : PREAMBLE_START + PREAMBLE_LEN]

    # Circular mean of preamble bins
    angles   = [2 * np.pi * b / N for b in preamble_bins]
    mean_sin = np.mean([np.sin(a) for a in angles])
    mean_cos = np.mean([np.cos(a) for a in angles])
    freq_offset_bins = int(round(
        np.arctan2(mean_sin, mean_cos) * N / (2 * np.pi)
    )) % N

    freq_offset_hz  = freq_offset_bins * BW / N
    freq_offset_ppm = freq_offset_hz / CENTER_FREQ * 1e6

    print(f"\nFrequency calibration (from preamble):")
    print(f"  Preamble bins (raw)  : {preamble_bins}")
    print(f"  Measured offset      : {freq_offset_bins} bins")
    print(f"  Offset in Hz         : {freq_offset_hz:.1f} Hz")
    print(f"  RTL-SDR ppm error    : {freq_offset_ppm:.2f} ppm")

    # Apply correction: subtract offset from every chip value, wrap at N.
    # After correction, preamble bins should all read 0, and all other
    # chip values are shifted to their true positions.
    corrected = [(c - freq_offset_bins) % N for c in chip_values]

    print(f"\nCorrected chip values (offset subtracted):")
    print(f"{'Sym':>4}  {'Raw bin':>10}  {'Corrected':>10}  Notes")
    print("-" * 45)
    for i, (raw, cor) in enumerate(zip(chip_values, corrected)):
        if   i < PREAMBLE_START:
            note = '(pre-burst)'
        elif i < PREAMBLE_START + PREAMBLE_LEN:
            note = '← preamble (should be 0)'
        elif i < PREAMBLE_START + PREAMBLE_LEN + 2:
            note = '← sync symbol'
        elif i < PREAMBLE_START + PREAMBLE_LEN + 5:
            note = '← downchirp / freq-sync'
        else:
            note = ''
        print(f"{i:4}  {raw:10}  {cor:10}  {note}")
        
    # ── 5. Plots ──────────────────────────────────────────────────────
    if PLOT and spectra:

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