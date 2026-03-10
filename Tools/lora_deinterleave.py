#!/usr/bin/env python3
"""
lora_deinterleave.py — LoRa Diagonal Deinterleaver (RX Stage 4)
================================================================

Reverses the diagonal interleaving applied by the LoRa transmitter.

Background (Robyns et al. §2.2, gr-lora decoder_impl::deinterleave) + other complementary sources:
─────────────────────────────────────────────────────────────────────────────────────────────────

What is a Codeword?
───────────────────
In LoRa, we don't just transmit raw data. We take 4 bits of actual data (a nibble)
and add extra redundant bits for Forward Error Correction (FEC). 
This combination of (Data + Parity) is called a "Codeword."

The number of redundant bits added is determined by the Coding Rate (CR):
  • CR = 1 (4/5 rate) -> 4 data bits + 1 parity bit = 5-bit codeword
  • CR = 4 (4/8 rate) -> 4 data bits + 4 parity bits = 8-bit codeword

Therefore, every codeword is exactly (4 + CR) bits long.

Why Interleave? (The "Burst Error" Problem)
───────────────────────────────────────────
Radio frequency interference usually happens in "bursts" (e.g., a lightning strike 
or a brief chirp from another device). If we transmitted an entire codeword inside 
a single symbol, a burst error could destroy the whole symbol, wiping out that 
codeword completely. The FEC cannot fix a codeword if the whole thing is gone.

Solution: Diagonal Interleaving.
The TX takes several codewords and spreads their bits across multiple symbols. 
If a burst error destroys one physical symbol, it only destroys *one bit* from 
several different codewords. The FEC can easily repair a single missing bit per codeword!

How the Matrix Works (Robyns et al. §2.2):
──────────────────────────────────────────
The TX writes codewords into an SF x (4+CR) matrix column-by-column, then reads 
them out along diagonals to produce the physical chips (symbols).

At the RX, we receive these symbols (which are just integer values recovered 
from the FFT bins via Gray mapping). The deinterleaver reverses the diagonal 
permutation to extract the original SF codewords.

Header vs. Payload:
───────────────────
  • Header  : Always uses CR=4 (so codewords are 8 bits long).
              Because it uses Reduced-Rate (LDRO), the top 2 bits of the SF 
              are intentionally zeroed out. Thus, effective rows = SF - 2.
  • Payload : Uses the CR specified in the header. Full SF rows are used.

Input  : JSON from the Gray-encode stage (symbols with 'symbol_value' & 'role')
Output : JSON with 'codewords' (bit-lists) for header and payload blocks
"""

import json
import argparse
from typing import List


# ══════════════════════════════════════════════════════════════════════════════
#  §1 — BIT HELPERS
# ══════════════════════════════════════════════════════════════════════════════

def symbol_to_bits(val: int, sf: int) -> List[int]:
    """
    Convert a symbol integer to an SF-length bit list, LSB first.

    Why LSB first?
    In LoRa's physical layer, the interleaver matrix is built starting from 
    the Least Significant Bit (LSB). Bit 0 of the symbol corresponds to Row 0 
    of the interleaving matrix.

    Example (SF=8, val=0b10110001 or 177):
        → [1, 0, 0, 0, 1, 1, 0, 1]
           ^LSB                 ^MSB
    """
    return [(val >> i) & 1 for i in range(sf)]


def bits_to_int_le(bits: List[int]) -> int:
    """
    Convert an LSB-first bit list back to an integer.

    This takes the deinterleaved bits (which form a codeword) and squashes 
    them back into a standard integer so the Hamming decoder can process it.
    
    Example: bits_to_int_le([1, 0, 0, 0, 1, 1, 0, 1]) → 0xB1 (177)
    """
    v = 0
    for i, b in enumerate(bits):
        v |= (b & 1) << i
    return v


# ══════════════════════════════════════════════════════════════════════════════
#  §2 — DIAGONAL DEINTERLEAVER
# ══════════════════════════════════════════════════════════════════════════════
#
#  Notation:
#    • C = 4 + CR         (Columns) Number of symbols per interleave block.
#                         Also equals the bit-length of one codeword.
#    • R = sf_rows        (Rows) Number of codewords produced per block.
#                         (= SF for payload, = SF-2 for header reduced-rate).
#    • I[row][col]        The interleaved matrix exactly as received.
#    • D[bit][cw_idx]     The deinterleaved matrix (the recovered codewords).
#
#  The Math:
#  To undo the TX diagonal spreading, we use modulo arithmetic to trace 
#  the diagonal lines back through the matrix:
#      D[j][k] = I[(k - j) mod R][j]
#
#  The Academic Formula vs. The Code:
#  ──────────────────────────────────
#  In the Robyns et al. paper (§2.2), the deinterleaving process is defined 
#  with this specific mathematical formula:
#      D[j][k] = I[ (k + j + 1) mod R ][ j ]
#
#  However, the widely used open-source implementation (gr-lora) achieves 
#  this algebraically using bitwise left-rotations (rotl), which translates 
#  in Python/array indexing to:
#      D[j][k] = I[ (k - j) mod R ][ j ]
#
#  Why the difference? The academic paper defines the matrix using different 
#  indexing conventions and diagonal slopes. The code uses standard 0-based 
#  indexing (LSB = 0) and directly mirrors the CPU's bitwise rotation logic. 
#  Both methods extract the exact same diagonal bits!
#
#  This is algebraically equivalent to gr-lora's rotl-then-extract-columns
#  approach (decoder_impl.cc:543):
#      word = rotl(d_words[i], i, ppm)
#  where rotating symbol i left by i positions and reading column k gives
#  the same bit as reading row (k + i + 1) mod R of the un-rotated symbol.
#
#  Visualizing the Deinterleave (Conceptual):
#  If you imagine the received Matrix I, the bits of Codeword 0 are not 
#  in a straight line. They are scattered diagonally:
#    Sym 0: [Bit 0 of CW_0] ...
#    Sym 1: ... [Bit 1 of CW_0] ...
#    Sym 2: ... ... [Bit 2 of CW_0] ...
# ──────────────────────────────────────────────────────────────────────────────

def deinterleave_block(symbol_block: List[int],
                       sf_rows: int,
                       sf: int,
                       cr_plus4: int) -> List[List[int]]:
    """
    Deinterleave one block of symbols into a list of codewords.

    Parameters
    ----------
    symbol_block : list of `cr_plus4` symbol integers (each 0 to 2^SF - 1)
    sf_rows      : effective number of bit-rows to use per symbol
                   • SF     for normal payload
                   • SF - 2 for header (reduced-rate mode ignores the top 2 bits)
    sf           : spreading factor (needed to know the total bits per symbol)
    cr_plus4     : number of symbols per block = 4 + CR
                   (this also dictates the length of the resulting codewords)

    Returns
    -------
    List of `sf_rows` codewords. Each codeword is a list of `cr_plus4` bits (LSB first).
    """
    C = cr_plus4   # columns = symbols per block
    R = sf_rows    # rows    = codewords produced

    # ── 1. Build the received interleaved bit-matrix I[row][col] ──────────
    # We take our block of symbols and break them down into bits.
    #    Row i  = bit position i  (0 = LSB)
    #    Col j  = symbol index j  (0 to C-1)
    #
    # Note on Header: For header reduced-rate (R = SF-2), we only loop up to R.
    # This automatically discards the top 2 bits of the symbol, which were 
    # forced to zero by the TX anyway.
    I = [[0] * C for _ in range(R)]
    for j, sym in enumerate(symbol_block):
        bits = symbol_to_bits(sym, sf)        # Expand integer to LSB-first bits
        for i in range(R):
            I[i][j] = bits[i]

    # ── 2. Reverse the diagonal permutation ───────────────────────────────
    # Here we trace the diagonals to reconstruct the codewords.
    #  j = bit position in the codeword (0 to C-1)
    #  k = codeword index (0 to R-1)
    #
    # The formula `(k - j) mod R` mathematically handles the "wrap-around" 
    # when a diagonal line hits the bottom of the matrix and continues 
    # from the top.
    #
    #  Derivation from gr-lora's rotl-then-extract-columns approach:
    #    deinterleaved[x] bit i = rotl(symbol[i], i, R) 
    #                     bit x = symbol[i] bit ((x - i) mod R)
    D = [[0] * R for _ in range(C)]
    for j in range(C):
        for k in range(R):
            i_idx = (k - j) % R
            D[j][k] = I[i_idx][j]

    # ── 3. Assemble codewords ─────────────────────────────────────────────
    # Now we just read the reconstructed matrix D column-by-column to get 
    # our clean codewords.
    # Bit index j corresponds to the j-th bit of the (4+CR)-bit codeword.
    codewords = []
    for k in range(R):
        cw_bits = [D[j][k] for j in range(C)]
        codewords.append(cw_bits)

    return codewords


# ══════════════════════════════════════════════════════════════════════════════
#  §3 — HEADER PARSING (placeholder)
# ══════════════════════════════════════════════════════════════════════════════
#
#  The LoRa explicit header contains 20 data bits (5 nibbles) which encode:
#    • Payload length (8 bits)
#    • Coding rate    (3 bits)  — tells us the CR for the payload
#    • CRC present    (1 bit)
#    • Header checksum (8 bits, across the header itself)
#
#  These 20 bits are protected by CR=4 Hamming(8,4) and the header-specific
#  whitening sequence (gr::lora::prng_header in utilities.h).
#
#  Full header decode requires:
#    1. Hamming(8,4) decode → 5 nibbles → 20 data bits
#
#  This is not yet implemented; we pass raw codeword ints through so the
#  next pipeline stage can apply Hamming decoding.
# ──────────────────────────────────────────────────────────────────────────────

def parse_header_from_codewords(cw_ints: List[int]) -> dict:
    """
    Placeholder for header field extraction.

    After Hamming decode, the 5 resulting nibbles encode the header fields.
    Until that stage is implemented, we just forward the raw codeword
    integers.

    See gr-lora decoder_impl.cc DECODE_HEADER state (~line 830) for the
    reference implementation that extracts payload_length, CR, and CRC flag.
    """
    return {"raw_header_codewords": cw_ints}


# ══════════════════════════════════════════════════════════════════════════════
#  §4 — MAIN: LOAD SYMBOLS → DEINTERLEAVE → WRITE JSON
# ══════════════════════════════════════════════════════════════════════════════

def main():
    ap = argparse.ArgumentParser(
        description="LoRa RX Stage 4: Deinterleaver. Reverses diagonal spreading "
                    "to recover error-corrected codewords from physical symbols.")
    ap.add_argument('input', nargs='?', default='lora_symbols_gray.json',
                    help="Input JSON from Stage 3 (default: lora_symbols_gray.json)")
    ap.add_argument('-o', '--output', default='lora_deinterleaved.json',
                    help="Output JSON path (default: lora_deinterleaved.json)")
    args = ap.parse_args()

    # ── Load Stage 3 data ─────────────────────────────────────────────────
    with open(args.input, 'r') as f:
        data = json.load(f)

    sf = data['config']['sf']

    # ── Define Header Matrix Rules ────────────────────────────────────────
    # The header is the most critical part of the packet, so it always uses 
    # the maximum protection settings regardless of user configuration:
    #    • CR = 4          -> 4 + 4 = 8 symbols per interleave block
    #    • Reduced-rate    -> effective bit-rows = SF - 2
    #  (see gr-lora decoder_impl.cc: d_phdr.cr is set to 4 for the header,
    #   and deinterleave is called with ppm = SF − 2 during DECODE_HEADER)
    header_cr         = 4
    header_block_cols = 4 + header_cr       # 8 symbols per header block
    header_rows       = sf - 2              # Discard the 2 zeroed MSBs (reduced-rate)

    # Separate symbols based on the role assigned in earlier stages
    header_symbols  = [s['symbol_value']
                       for s in data['symbols']
                       if s.get('role') == 'header']
    payload_symbols = [s['symbol_value']
                       for s in data['symbols']
                       if s.get('role') == 'payload']

    result = {
        "config":  data['config'],
        "header":  {},
        "payload": {},
    }

    # ── Process Header Block(s) ───────────────────────────────────────────
    # We slice the header symbols into chunks of 8 (header_block_cols).
    # Each chunk of 8 symbols yields (SF - 2) individual codewords.
    #
    #  gr-lora reference (decoder_impl.cc, DECODE_HEADER state ~line 830):
    #    deinterleave(sf - 2u)  // ppm = SF - 2 for header reduced-rate
    header_codewords_bits: List[List[int]] = []
    for blk_start in range(0, len(header_symbols), header_block_cols):
        block = header_symbols[blk_start : blk_start + header_block_cols]
        if len(block) < header_block_cols:
            break   # End of data, incomplete block
        cw_bits = deinterleave_block(block, header_rows, sf, header_block_cols)
        header_codewords_bits.extend(cw_bits)

    # Convert the bit-lists back to integers for the next stage
    header_cw_ints = [bits_to_int_le(b) for b in header_codewords_bits]
    result['header']['codewords_bits'] = header_codewords_bits
    result['header']['codewords_ints'] = header_cw_ints

    # NOTE: to fully decode the header you still need:
    #   Hamming(8,4) decode  (decoder_impl.cc:582)
    # Once decoded, the header tells us payload length, payload CR, and
    # whether a payload CRC is present.

    # ── Determine Payload Matrix Rules ────────────────────────────────────
    # In a real receiver, the payload_cr is extracted from the decoded header.
    # Since we haven't decoded the header yet, we check the config dict. If 
    # it's missing, we default to CR=4 as a safe fallback.
    payload_cr = data['config'].get('cr')
    if payload_cr is None:
        payload_cr = header_cr
    
    payload_block_cols = 4 + payload_cr   # Size of chunk depends on CR
    payload_rows       = sf               # Payload uses the full SF rows

    # ── Process Payload Block(s) ──────────────────────────────────────────
    # We slice the payload symbols into chunks of (4 + CR).
    # Each chunk yields exactly SF individual codewords.
    #
    #  gr-lora reference (decoder_impl.cc, DECODE_PAYLOAD state ~line 860):
    #    deinterleave(sf)  // ppm = SF for payload
    payload_codewords_bits: List[List[int]] = []
    for blk_start in range(0, len(payload_symbols), payload_block_cols):
        block = payload_symbols[blk_start : blk_start + payload_block_cols]
        if len(block) < payload_block_cols:
            break
        cw_bits = deinterleave_block(block, payload_rows, sf, payload_block_cols)
        payload_codewords_bits.extend(cw_bits)

    payload_cw_ints = [bits_to_int_le(b) for b in payload_codewords_bits]
    result['payload']['codewords_bits'] = payload_codewords_bits
    result['payload']['codewords_ints'] = payload_cw_ints

    # ── Write Output & Print Summary ──────────────────────────────────────
    data['deinterleave'] = result
    with open(args.output, 'w') as f:
        json.dump(data, f, indent=2)

    print(f"SF = {sf}, header CR = {header_cr}, payload CR = {payload_cr}")
    print(f"Header  : {len(header_symbols):3d} symbols → "
          f"{len(header_codewords_bits):3d} codewords  (block size {header_block_cols})")
    print(f"Payload : {len(payload_symbols):3d} symbols → "
          f"{len(payload_codewords_bits):3d} codewords  (block size {payload_block_cols})")
    print(f"Wrote deinterleaved output to: {args.output}")

if __name__ == '__main__':
    main()