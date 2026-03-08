#!/usr/bin/env python3
"""
lora_gray_encode.py — Stage 3 of the LoRa decode pipeline
=====================================================

Reads the symbol data produced by lora_get_symbol_value_FFT.py (JSON) and
recovers the original symbol values by applying Gray encoding to each
FFT bin index.

We use Gray coding so that adjacent symbols differ by only 1 bit, which reduces
bit errors (KEY IDEA) when the receiver slightly misidentifies the symbol
(compared to a binary mapping where adjacent symbols could differ by many bits).

Example of an error where the receiver detects the wrong symbol by 1 position:
    Without Gray code (binary mapping):
    011 → 100   (3 bit errors)
    With Gray code:
    011 → 010   (1 bit error)

LoRa Gray coding (Robyns et al. 2018, §2.1, Eq. 2)
────────────────────────────────────────────────────

  ┌──────────────────────────────────────────────────────────────────┐
  │  GRAY CODE MAPPING IN LoRa (CLEAR EXPLANATION)                   │
  │                                                                  │
  │  Mathematical operations:                                        │
  │    binary_to_gray(x)  = x ^ (x >> 1)    [binary → Gray code]     │
  │    gray_to_binary(g)  = inverse          [Gray code → binary]    │
  │                                                                  │
  │  These are INVERSE operations: if you apply both in sequence,    │
  │  you get back the original value.                                │
  │                                                                  │
  │  How LoRa uses them (counter-intuitive):                         │
  │                                                                  │
  │    TX (encoder.py) :  Uses gray_to_binary()                      │
  │         Input: BINARY symbol values (0, 1, 2, ..., 255)          │
  │         Computation: shift = gray_to_binary(symbol)              │
  │         Output: Chirp time-shift (used to modulate RF)           │
  │         Effect: Creates Gray-ordered chirp shifts on the RF      │
  │                                                                  │
  │    RX (this file)  :  Uses binary_to_gray()                      │
  │         Input: FFT bin index from demodulated signal             │
  │         Computation: symbol = binary_to_gray(bin_index)          │
  │         Output: BINARY symbol value (recovers original)          │
  │         Why: TX maps symbols to shifts using gray_to_binary().   │
  │              RX applies the inverse mapping binary_to_gray() to  │
  │              recover the original symbol. Thus:                  │
  │              binary_to_gray(gray_to_binary(s)) = s               │
  │                                                                  │
  │  Complete flow:                                                  │
  │    1. TX input: BINARY symbol (e.g., value = 5)                  │
  │    2. TX: shift = gray_to_binary(5)  [transforms to index]       │
  │    3. TX: Transmit chirp at that physical shift                  │
  │    4. RX: Demodulate signal → get FFT bin index                  │
  │    5. RX: symbol = binary_to_gray(bin)  [recovers value = 5]     │
  │                                                                  │
  │  This matches gr-lora's demodulate():                            │
  │    const uint32_t word = bin_idx ^ (bin_idx >> 1u);  // encode!  │
  │                                                                  │
  └──────────────────────────────────────────────────────────────────┘

RX formula:
    symbol_value = binary_to_gray(bin_index)  =  bin_index ^ (bin_index >> 1)

TX formula (in lora_encoder.py):
    chirp_shift  = gray_to_binary(symbol_value)

Input  : lora_symbols.json        (from Stage 2)
Output : lora_symbols_gray.json   (same structure, with added 'symbol_value' field)
"""

import argparse
import json
import sys
import os


# ═════════════════════════════════════════════════════════════════════════════
#  GRAY CODE MAPPING FUNCTIONS
# ═════════════════════════════════════════════════════════════════════════════

def binary_to_gray(v: int) -> int:
    """Binary value → Gray code.
    
    Mathematically: G(v) = v ^ (v >> 1) 
    (where ^ is bitwise XOR and >> is right shift)
    
    In LoRa RX: Used to recover the original symbol value from an FFT bin index.
    TX created a Gray-ordered mapping between symbol values and chirp shifts.
    RX applies this function (the inverse of gray_to_binary) to recover the
    original binary symbol from the detected bin index.
    
    Also known as: gray_encode (in traditional Gray code terminology).
    """
    return v ^ (v >> 1)


def gray_to_binary(g: int) -> int:
    """Gray code → Binary value (inverse of binary_to_gray).
    
    In LoRa TX: Used to convert a binary symbol into a chirp shift index.
    Despite the counter-intuitive naming, this creates a Gray-ordered mapping
    on the RF signal (adjacent symbols map to adjacent physical shifts).
    
    Also known as: gray_decode (in traditional Gray code terminology).
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
        description="LoRa Stage 3: recover symbol values by applying Gray mapping to FFT bin indices."
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

    print(f"Recovering symbol values (Gray mapping of bin index) from: {args.input}")
    print(f"  SF={sf}  N_bins={n_bins}")
    print()

    # ── Apply binary_to_gray(bin) to recover each symbol value ────────────
    # Round corrected_bin to nearest integer first (it may have sub-bin
    # precision from parabolic interpolation).
    #
    # REDUCED-RATE (header):
    #   gr-lora demodulate() divides the bin index by 4 and reduces to
    #   (SF-2) bits BEFORE Gray mapping:
    #       bin_idx = lround(bin_idx / 4.0) % d_number_of_bins_hdr
    #       word    = bin_idx ^ (bin_idx >> 1)               // binary_to_gray!
    #   where d_number_of_bins_hdr = 2^(SF-2) = n_bins / 4.
    #
    # NORMAL (payload):
    #   Full SF-bit bin index is mapped directly.
    #       word = bin_idx ^ (bin_idx >> 1)
    #
    # Gray mapping does NOT commute with truncation:
    #   Gray(round(bin/4) % 64) ≠ Gray(bin) % 64
    # So the reduced-rate division MUST be applied before Gray mapping.

    n_bins_hdr = n_bins >> 2   # 2^(SF-2) = 64 for SF=8

    print(f"{'Sym':>4}  {'Role':>10}  {'Corr bin':>8}  {'Rounded':>7}  "
          f"{'Sym val':>8}  {'Binary':>10}")
    print("-" * 65)

    for sym in data['symbols']:
        corrected = sym['corrected_bin']
        rounded   = round(corrected) % n_bins     # wrap into [0, N-1]

        role = sym.get('role', '')

        if role == 'header':
            # Reduced-rate: divide by 4, reduce to (SF-2) bits, THEN apply Gray mapping
            reduced_bin = round(rounded / 4.0) % n_bins_hdr
            sym_val     = binary_to_gray(reduced_bin)
        else:
            # Normal: Apply Gray mapping to the full SF-bit bin directly
            sym_val     = binary_to_gray(rounded)

        # Store results
        sym['corrected_bin_rounded'] = rounded
        sym['symbol_value']          = sym_val

        # Format binary string for display (SF-2 bits for header, SF bits for payload)
        width = sf - 2 if role == 'header' else sf
        bin_str = format(sym_val, f'0{width}b')

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
