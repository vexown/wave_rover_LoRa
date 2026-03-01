#!/usr/bin/env python3
"""
lora_deshuffle.py — LoRa Codeword Deshuffle (RX Stage 5)
=========================================================

Reverses the bit-level shuffling applied by the LoRa transmitter to each
codeword after interleaving.

gr-lora reference (decoder_impl.cc, deshuffle(), ~line 611):
─────────────────────────────────────────────────────────────
Each codeword has its bits permuted according to a fixed pattern:

    shuffle_pattern[] = {5, 0, 1, 2, 4, 3, 6, 7}

To deshuffle (recover original bit order):

    result_bit[j] = input_bit[ shuffle_pattern[j] ]

        j  :  0  1  2  3  4  5  6  7
        src:  5  0  1  2  4  3  6  7

    In other words:
        output bit 0 ← input bit 5
        output bit 1 ← input bit 0
        output bit 2 ← input bit 1
        output bit 3 ← input bit 2
        output bit 4 ← input bit 4
        output bit 5 ← input bit 3
        output bit 6 ← input bit 6
        output bit 7 ← input bit 7

Header special handling (decoder_impl.cc ~line 631):
────────────────────────────────────────────────────
  • Only the first 5 of 6 header codewords are deshuffled.
  • The 6th codeword is replaced with 0x00 in the header stream (it carries
    header-CRC info that gr-lora handles separately).
  • The original (un-deshuffled) 6th codeword is PREPENDED to the payload
    stream — it becomes the first element that payload deshuffle, dewhiten,
    and Hamming processes.  This is how gr-lora's state machine works
    (d_demodulated retains the 6th codeword after the header decode phase).

Pipeline position:
  … → Gray encode → Deinterleave → **[Deshuffle]** → Dewhiten
  → Hamming decode → Byte assembly → CRC check
"""

import json
import argparse
from typing import List


# ══════════════════════════════════════════════════════════════════════════════
#  SHUFFLE PATTERN — from gr-lora decoder_impl.cc
# ══════════════════════════════════════════════════════════════════════════════
#
#  The TX shuffles codeword bits with this pattern before interleaving.
#  The RX reverses it: result_bit[j] = input_bit[pattern[j]].
#
SHUFFLE_PATTERN = [5, 0, 1, 2, 4, 3, 6, 7]


# ══════════════════════════════════════════════════════════════════════════════
#  §1 — DESHUFFLE FUNCTIONS
# ══════════════════════════════════════════════════════════════════════════════

def deshuffle_byte(val: int) -> int:
    """
    Deshuffle one 8-bit codeword using the fixed LoRa shuffle pattern.

    Implements the inner loop of gr-lora deshuffle():
        result = 0;
        for (j = 0; j < 8; j++)
            result |= !!(input & (1 << shuffle_pattern[j])) << j;

    Parameters
    ----------
    val : int   8-bit codeword from the deinterleave stage.

    Returns
    -------
    int : deshuffled 8-bit codeword.
    """
    result = 0
    for j, src_bit in enumerate(SHUFFLE_PATTERN):
        if val & (1 << src_bit):
            result |= (1 << j)
    return result


def deshuffle_block(codewords: List[int], is_header: bool) -> dict:
    """
    Deshuffle a list of codewords and handle the header/payload boundary.

    For header (is_header=True):
      – Deshuffle codewords 0..4  (first 5 of 6)
      – Append 0x00 as the 6th value  (per gr-lora: header CRC placeholder)
      – Return the 6th codeword separately so it can be prepended to payload

    For payload (is_header=False):
      – Deshuffle ALL codewords
      – Return the full deshuffled list

    Returns
    -------
    dict with:
      'deshuffled' : list of deshuffled integers
      'spillover'  : (header only) the un-deshuffled 6th codeword, or None
    """
    if is_header:
        # Deshuffle only the first 5 codewords (indices 0–4)
        deshuffled = [deshuffle_byte(cw) for cw in codewords[:5]]
        # Append 0x00 in place of the 6th codeword
        deshuffled.append(0x00)
        # The 6th codeword (index 5) spills over to be prepended to payload
        spillover = codewords[5] if len(codewords) > 5 else None
        return {'deshuffled': deshuffled, 'spillover': spillover}
    else:
        deshuffled = [deshuffle_byte(cw) for cw in codewords]
        return {'deshuffled': deshuffled, 'spillover': None}


# ══════════════════════════════════════════════════════════════════════════════
#  §2 — MAIN
# ══════════════════════════════════════════════════════════════════════════════

def main():
    ap = argparse.ArgumentParser(
        description="LoRa RX deshuffle: reverse the bit permutation "
                    "applied to each codeword by the TX.")
    ap.add_argument('input', nargs='?', default='lora_deinterleaved.json',
                    help="Input JSON from deinterleave stage "
                         "(default: lora_deinterleaved.json)")
    ap.add_argument('-o', '--output', default='lora_deshuffled.json',
                    help="Output JSON path (default: lora_deshuffled.json)")
    args = ap.parse_args()

    # ── Load deinterleaved data ───────────────────────────────────────────
    with open(args.input, 'r') as f:
        data = json.load(f)

    deint = data['deinterleave']
    header_cw  = deint['header']['codewords_ints']
    payload_cw = deint['payload']['codewords_ints']

    # ── Deshuffle header ──────────────────────────────────────────────────
    #  Only the first 5 codewords are deshuffled; the 6th is replaced with
    #  0x00 and the original 6th spills over to the payload stream.
    hdr_result = deshuffle_block(header_cw, is_header=True)
    header_deshuffled = hdr_result['deshuffled']
    spillover_cw      = hdr_result['spillover']

    # ── Deshuffle payload ─────────────────────────────────────────────────
    #  Prepend the spillover codeword (6th header codeword, un-deshuffled)
    #  to the payload list, then deshuffle the whole combined list.
    #  This matches gr-lora, where d_demodulated retains the 6th header
    #  codeword and payload symbols are appended after it.
    payload_combined = []
    if spillover_cw is not None:
        payload_combined.append(spillover_cw)
    payload_combined.extend(payload_cw)

    pay_result = deshuffle_block(payload_combined, is_header=False)
    payload_deshuffled = pay_result['deshuffled']

    # ── Store results for the next stage ──────────────────────────────────
    data['deshuffled'] = {
        'header': {
            'codewords_ints': header_deshuffled,
            'note': 'First 5 codewords deshuffled, 6th replaced with 0x00'
        },
        'payload': {
            'codewords_ints': payload_deshuffled,
            'note': '6th header codeword prepended then all deshuffled'
        },
        'shuffle_pattern': SHUFFLE_PATTERN,
    }

    with open(args.output, 'w') as f:
        json.dump(data, f, indent=2)

    # ── Summary ───────────────────────────────────────────────────────────
    def fmt(vals):
        return ' '.join(f'0x{v:02X}' for v in vals)

    print(f"Shuffle pattern: {SHUFFLE_PATTERN}")
    print()
    print(f"Header  input  ({len(header_cw):2d} cw): {fmt(header_cw)}")
    print(f"Header  output ({len(header_deshuffled):2d} cw): {fmt(header_deshuffled)}")
    print(f"  (5 deshuffled + [0x00]; 6th codeword 0x{spillover_cw:02X} "
          f"→ prepended to payload)")
    print()
    print(f"Payload input  ({len(payload_cw):2d} cw): {fmt(payload_cw)}")
    print(f"  + spillover 0x{spillover_cw:02X} prepended → "
          f"{len(payload_combined)} cw total")
    print(f"Payload output ({len(payload_deshuffled):2d} cw): "
          f"{fmt(payload_deshuffled)}")
    print()
    print(f"Wrote deshuffled output to: {args.output}")


if __name__ == '__main__':
    main()
