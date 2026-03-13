#!/usr/bin/env python3
"""
lora_deshuffle.py — LoRa Codeword Deshuffle (RX Stage 5)
=========================================================

Reverses the bit-level shuffling applied by the LoRa transmitter to each
codeword after interleaving.

Why do we need this? (Intra-codeword protection)
────────────────────────────────────────────────
Interleaving (Stage 4) protects us if a whole physical symbol gets destroyed.
But what if a tiny bit of noise flips two adjacent bits inside a codeword?
The Hamming FEC struggles with adjacent burst errors. 
By "shuffling" the bit order before transmission, Semtech ensured that 
adjacent physical bits are not adjacent logical bits in the codeword.

The Shuffle Pattern (gr-lora reference, decoder_impl.cc, decode(), ~line 568):
──────────────────────────────────────────────────
Each codeword has its bits permuted according to a fixed hardware wire-routing:
    shuffle_pattern[] = {5, 0, 1, 2, 4, 3, 6, 7}

To deshuffle, we route the received bits back to their original positions:
    Output Bit  <-  Input Bit
        0       <-      5
        1       <-      0
        2       <-      1
        3       <-      2
        4       <-      4  (stays the same)
        5       <-      3
        6       <-      6  (stays the same)
        7       <-      7  (stays the same)

The "6th Codeword" Mystery Explained:
─────────────────────────────────────
Why does the header only use 5 codewords, and what is this 6th one doing?
  1. The LoRa explicit header contains exactly 20 bits of data.
  2. Because it uses CR=4, those 20 bits are encoded as five 4-bit nibbles.
  3. 5 nibbles + FEC = 5 codewords needed.
  4. HOWEVER, at SF8, the header matrix generates `SF - 2` = 6 codewords!

So the physical block has 6 slots, but the header only needs 5.
What does the LoRa TX put in that 6th slot? The first codeword of the
payload. Rather than waste a slot, the transmitter embeds the payload's
first codeword directly inside the header's 8-symbol block.

On the RX side, the deinterleaver faithfully recovers all 6 codewords from
those 8 header symbols. gr-lora then deshuffles only the first 5 (the real
header FEC data) and — crucially — does NOT erase the 6th:
    d_demodulated.erase(d_demodulated.begin(), d_demodulated.begin() + 5u)
That 6th entry stays at the front of d_demodulated. When the payload
decoder runs next, it ingests it first. Discarding it would silently drop
the first codeword of the payload, corrupting the entire payload decode.

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

    How the bit-shifting works:
    Imagine `val` is an 8-bit array. We loop through the SHUFFLE_PATTERN.
    On loop index `j=0`, we look at `SHUFFLE_PATTERN[0]`, which is `5`.
    We grab bit 5 from the input `val`, and drop it into bit 0 of `result`.

    Parameters
    ----------
    val : int   8-bit codeword (integer) from the deinterleave stage.

    Returns
    -------
    int : deshuffled 8-bit codeword.
    """
    result = 0
    for j, src_bit in enumerate(SHUFFLE_PATTERN):
        # If the specific source bit is a '1' in the input value...
        if val & (1 << src_bit):
            # ...then set the j-th bit in our result to '1'.
            result |= (1 << j)
    return result


def deshuffle_block(codewords: List[int], is_header: bool) -> dict:
    """
    Deshuffle a list of codewords and handle the strict header/payload boundary.

    For header (is_header=True):
      - We process exactly the 5 codewords the header needs.
      - We artificially inject 0x00 into the stream as a placeholder (gr-lora 
        does this to align its internal state machine for the header CRC).
      - We extract the 6th codeword (which is actually payload data) and return
        it as `spillover` so it isn't lost.

    For payload (is_header=False):
      - Deshuffle ALL codewords directly.
      - Return the full deshuffled list with no spillover.

    Returns
    -------
    dict with:
      'deshuffled' : list of deshuffled integers ready for the next stage.
      'spillover'  : (header only) the un-deshuffled 6th codeword, or None.
    """
    if is_header:
        # Deshuffle only the first 5 codewords (indices 0 to 4)
        deshuffled = [deshuffle_byte(cw) for cw in codewords[:5]]
        
        # Append 0x00 to maintain the expected 6-element array size 
        # required by later gr-lora state machine steps.
        deshuffled.append(0x00)
        
        # Save the 6th codeword (index 5) untouched. It belongs to the payload!
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
    # The header parsing is highly rigid. We only deshuffle the first 5 
    # codewords, replace the 6th with a dummy byte, and save the real 6th byte.
    hdr_result = deshuffle_block(header_cw, is_header=True)
    header_deshuffled = hdr_result['deshuffled']
    spillover_cw      = hdr_result['spillover']

    # ── Deshuffle payload ─────────────────────────────────────────────────
    # Before processing the payload block, we must glue that 6th header 
    # codeword onto the very front of the payload stream.
    payload_combined = []
    if spillover_cw is not None:
        payload_combined.append(spillover_cw)
    payload_combined.extend(payload_cw)

    # Now deshuffle the entire combined payload stream.
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