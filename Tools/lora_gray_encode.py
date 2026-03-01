#!/usr/bin/env python3
"""
lora_gray_encode.py — Stage 3 of the LoRa decode pipeline
=====================================================

Reads the symbol data produced by lora_get_symbol_value_FFT.py (JSON) and
recovers the original symbol values by applying Gray encoding to each
FFT bin index.

LoRa Gray coding (Robyns et al. 2018, §2.1, Eq. 2)
────────────────────────────────────────────────────

  ┌──────────────────────────────────────────────────────────────────┐
  │  NAMING CONVENTION — READ THIS FIRST                            │
  │                                                                 │
  │  "gray_encode" and "gray_decode" describe operations on the     │
  │  Gray code itself, NOT which end of the radio link uses them.   │
  │                                                                 │
  │    gray_encode(x) = x ^ (x >> 1)     binary  → Gray code        │
  │    gray_decode(g) = (inverse)         Gray code → binary        │
  │                                                                 │
  │  In LoRa the roles are COUNTER-INTUITIVE:                       │
  │                                                                 │
  │    TX (encoder.py) :  gray_DECODE(symbol) → chirp shift         │
  │         The transmitter converts symbol values FROM Gray-code   │
  │         space INTO natural/binary order to get the physical     │
  │         time-shift for the chirp.                               │
  │                                                                 │
  │    RX (this file)  :  gray_ENCODE(bin_index) → symbol value     │
  │         The receiver gets an FFT bin in natural order and       │
  │         converts it INTO Gray-code space to recover the         │
  │         original symbol value.                                  │
  │                                                                 │
  │  This matches gr-lora's demodulate():                           │
  │    const uint32_t word = bin_idx ^ (bin_idx >> 1u);  // encode! │
  │                                                                 │
  │  We call the output field "symbol_value" to avoid the           │
  │  encode-vs-decode naming trap entirely.                         │
  │                                                                 │
  │  Does this mean that the symbols are not gray-encoded when      │
  │  "on Air" since we did gray_decode before actual transmission?  │
  |  No! — the symbols are effectively Gray-encoded on the air.     |
  |  Here's why - TX computes: shift = Gray⁻¹(i) The physical chirp |
  |  shift equals that value.                                       |
  |  Because of the Gray mapping adjacent bins correspond to symbols| 
  |  differing by 1 bit only.                                       |
  |                                                                 |
  |  So although the transmitter applies Gray inverse, the resulting|
  |  RF symbol ordering is Gray-ordered.                            |
  |                                                                 |
  └─────────────────────────────────────────────────────────────────┘

RX formula:
    symbol_value = gray_encode(bin_index)  =  bin_index ^ (bin_index >> 1)

TX formula (in lora_encoder.py):
    chirp_shift  = gray_decode(symbol_value)

Input  : lora_symbols.json        (from Stage 2)
Output : lora_symbols_gray.json   (same structure, with added 'symbol_value' field)
"""

import argparse
import json
import sys
import os


# ═════════════════════════════════════════════════════════════════════════════
#  GRAY CODING
# ═════════════════════════════════════════════════════════════════════════════

def gray_encode(v: int) -> int:
    """binary → Gray code:  G(v) = v ^ (v >> 1).

    Despite the name, this is what the RX uses to recover symbol values:
        symbol_value = gray_encode(fft_bin_index)

    The TX applied gray_decode() before modulation (see lora_encoder.py),
    so the RX reverses that by applying gray_encode() here.
    """
    return v ^ (v >> 1)


def gray_decode(g: int) -> int:
    """Gray code → binary (inverse of gray_encode).

    Used by the TX (lora_encoder.py) to convert symbol values into chirp
    time-shift indices.  Not used in the RX path.
    """
    mask = g
    while mask:
        mask >>= 1
        g ^= mask
    return g


# ═════════════════════════════════════════════════════════════════════════════
#  MAIN
# ═════════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description="LoRa Stage 3: recover symbol values by Gray-encoding FFT bin indices."
    )
    parser.add_argument(
        'input', nargs='?', default='lora_symbols.json',
        help='Input JSON from Stage 2 (default: lora_symbols.json).'
    )
    parser.add_argument(
        '-o', '--output', default='lora_symbols_gray.json',
        help='Output JSON path (default: lora_symbols_gray.json).'
    )
    args = parser.parse_args()

    # ── Load Stage 2 output ───────────────────────────────────────────────
    if not os.path.exists(args.input):
        print(f"ERROR: {args.input} not found. Run Stage 2 first.")
        sys.exit(1)

    with open(args.input, 'r') as f:
        data = json.load(f)

    sf     = data['config']['sf']
    n_bins = data['config']['n_bins']   # 2^SF

    print(f"Recovering symbol values (gray_encode of bin index) from: {args.input}")
    print(f"  SF={sf}  N_bins={n_bins}")
    print()

    # ── Apply gray_encode(bin) to recover each symbol value ───────────────
    # Round corrected_bin to nearest integer first (it may have sub-bin
    # precision from parabolic interpolation).
    #
    # REDUCED-RATE (header):
    #   gr-lora demodulate() divides the bin index by 4 and reduces to
    #   (SF-2) bits BEFORE Gray encoding:
    #       bin_idx = lround(bin_idx / 4.0) % d_number_of_bins_hdr
    #       word    = bin_idx ^ (bin_idx >> 1)               // Gray encode
    #   where d_number_of_bins_hdr = 2^(SF-2) = n_bins / 4.
    #
    # NORMAL (payload):
    #   Full SF-bit bin index is Gray-encoded directly.
    #       word = bin_idx ^ (bin_idx >> 1)
    #
    # Gray coding does NOT commute with truncation:
    #   Gray(round(bin/4) % 64) ≠ Gray(bin) % 64
    # So the reduced-rate division MUST be applied before Gray encoding.

    n_bins_hdr = n_bins >> 2   # 2^(SF-2) = 64 for SF=8

    print(f"{'Sym':>4}  {'Role':>10}  {'Corr bin':>8}  {'Rounded':>7}  "
          f"{'Sym val':>8}  {'Binary':>10}")
    print("-" * 65)

    for sym in data['symbols']:
        corrected = sym['corrected_bin']
        rounded   = round(corrected) % n_bins     # wrap into [0, N-1]

        role = sym.get('role', '')

        if role == 'header':
            # Reduced-rate: divide by 4, reduce to (SF-2) bits, THEN Gray
            reduced_bin = round(rounded / 4.0) % n_bins_hdr
            sym_val     = gray_encode(reduced_bin)
        else:
            # Normal: Gray encode the full SF-bit bin directly
            sym_val     = gray_encode(rounded)

        # Store results
        sym['corrected_bin_rounded'] = rounded
        sym['symbol_value']          = sym_val

        # Format binary string for display (SF bits wide)
        bin_str = format(sym_val, f'0{sf}b')

        print(f"{sym['index']:4}  {sym['role']:>10}  {corrected:8.2f}  "
              f"{rounded:7}  {sym_val:8}  {bin_str:>10}")

    # ── Summary ───────────────────────────────────────────────────────────
    print()

    # Show just the header + payload symbol values
    header_syms  = [s for s in data['symbols'] if s['role'] == 'header']
    payload_syms = [s for s in data['symbols'] if s['role'] == 'payload']

    if header_syms:
        vals = [s['symbol_value'] for s in header_syms]
        print(f"  Header  symbols ({len(vals)}): {vals}")

    if payload_syms:
        vals = [s['symbol_value'] for s in payload_syms]
        print(f"  Payload symbols ({len(vals)}): {vals}")

    # ── Write output ──────────────────────────────────────────────────────
    with open(args.output, 'w') as f:
        json.dump(data, f, indent=2)

    print(f"\nSymbol value data saved to: {args.output}")


if __name__ == '__main__':
    main()
