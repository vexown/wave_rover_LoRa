#!/usr/bin/env python3
"""
lora_degray.py — Stage 3 of the LoRa decode pipeline
=====================================================

Reads the symbol data produced by lora_get_symbol_value_FFT.py (JSON) and
applies Gray decoding to each corrected chip value.

LoRa Gray coding (Robyns et al. 2018, §2.1, Eq. 2)
────────────────────────────────────────────────────
The transmitter applies Gray inverse (decoding) to the symbol value before
modulating it onto a chirp.  After dechirping + FFT at the receiver, the
resulting bin index is therefore Gray-encoded.

The naming here is confusing but think of it this way:
The symbol values are defined in Gray-code domain. 
The TX needs to leave that domain (decode) to get a physical shift. 
The RX needs to enter that domain (encode) to recover the symbol

To recover the original symbol value we apply standard Gray encoding to the
bin index (which is the same operation as Gray decoding of the value):

    symbol_value = gray_encode(bin_index)
    where gray_encode(x) = x ^ (x >> 1)

This is consistent with gr-lora's demodulate():
    const uint32_t word = bin_idx ^ (bin_idx >> 1u);

and with our own lora_encoder.py which uses gray_decode() on the TX side:
    shifts = [gray_decode(v) for v in chip_values]

Input  : lora_symbols.json        (from Stage 2)
Output : lora_symbols_gray.json   (same structure, with added 'gray_decoded' field)
"""

import argparse
import json
import sys
import os


# ═════════════════════════════════════════════════════════════════════════════
#  GRAY CODING
# ═════════════════════════════════════════════════════════════════════════════

def gray_encode(v: int) -> int:
    """Standard binary-reflected Gray encoding: G(v) = v ^ (v >> 1).

    At the receiver this is applied to the FFT peak bin index to undo
    the Gray-inverse that the transmitter applied before modulation.
    The name is confusing because the *encoder* (TX) uses gray_decode
    and the *decoder* (RX) uses gray_encode — but that is how LoRa works.
    """
    return v ^ (v >> 1)


def gray_decode(g: int) -> int:
    """Standard Gray decoding (inverse of gray_encode).

    Provided for completeness / testing.  Not used in the RX path —
    the RX path uses gray_encode on the bin index.
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
        description="LoRa Stage 3: Gray-decode corrected chip values."
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

    print(f"Gray decoding symbols from: {args.input}")
    print(f"  SF={sf}  N_bins={n_bins}")
    print()

    # ── Apply Gray encoding to each corrected bin → symbol value ──────────
    # Round corrected_bin to nearest integer first (it may have sub-bin
    # precision from parabolic interpolation).
    print(f"{'Sym':>4}  {'Role':>10}  {'Corr bin':>8}  {'Rounded':>7}  "
          f"{'Gray dec':>8}  {'Binary':>10}")
    print("-" * 65)

    for sym in data['symbols']:
        corrected = sym['corrected_bin']
        rounded   = round(corrected) % n_bins     # wrap into [0, N-1]
        gray_dec  = gray_encode(rounded)           # RX Gray "decode" = gray_encode()

        # Store results
        sym['corrected_bin_rounded'] = rounded
        sym['gray_decoded']          = gray_dec

        # Format binary string for display (SF bits wide)
        bin_str = format(gray_dec, f'0{sf}b')

        print(f"{sym['index']:4}  {sym['role']:>10}  {corrected:8.2f}  "
              f"{rounded:7}  {gray_dec:8}  {bin_str:>10}")

    # ── Summary ───────────────────────────────────────────────────────────
    print()

    # Show just the header + payload gray-decoded values
    header_syms  = [s for s in data['symbols'] if s['role'] == 'header']
    payload_syms = [s for s in data['symbols'] if s['role'] == 'payload']

    if header_syms:
        vals = [s['gray_decoded'] for s in header_syms]
        print(f"  Header  symbols ({len(vals)}): {vals}")

    if payload_syms:
        vals = [s['gray_decoded'] for s in payload_syms]
        print(f"  Payload symbols ({len(vals)}): {vals}")

    # ── Write output ──────────────────────────────────────────────────────
    with open(args.output, 'w') as f:
        json.dump(data, f, indent=2)

    print(f"\nGray-decoded symbol data saved to: {args.output}")


if __name__ == '__main__':
    main()
