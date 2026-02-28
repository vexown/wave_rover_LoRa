#!/usr/bin/env python3
"""
lora_deinterleave.py — LoRa Diagonal Deinterleaver (RX Stage 4)
================================================================

Reverses the diagonal interleaving applied by the LoRa transmitter.

Background (Robyns et al. §2.2, gr-lora decoder_impl::deinterleave):
─────────────────────────────────────────────────────────────────────
The TX interleaver takes SF codewords of (4+CR) bits each and writes them
into an SF × (4+CR) matrix column-by-column, then reads them out along
*diagonals* to produce (4+CR) chip values of SF bits each.

At the RX we receive (4+CR) chip values (symbol values after Gray encoding
of FFT bins).  The deinterleaver reverses the diagonal permutation to
recover the original SF codewords.

In gr-lora (decoder_impl.cc ~line 543) the C++ deinterleaver does:
    for (i = 0; i < bits_per_word; i++) {
        word = rotl(d_words[i], i, ppm);       // rotate left by i
        for (j = (1 << offset_start); j; j >>= 1, x--)
            words_deinterleaved[x] |= !!(word & j) << i;
    }
This extracts columns from the rotated matrix — equivalent to the diagonal
read expressed algebraically below.

Header vs. Payload:
───────────────────
  • Header  : always CR=4 (8 columns), reduced-rate → effective rows = SF−2
              (the two MSBs of each symbol are discarded / zero in reduced-rate)
  • Payload : CR from header, full SF rows

Input  : JSON from the Gray-encode stage (symbols with 'symbol_value' & 'role')
Output : JSON with 'codewords' (bit-lists) for header and payload blocks

Pipeline position:
  I/Q → preamble detect → time sync → freq sync → dechirp+FFT
  → Gray decode → **[Deinterleave]** → [HEADER: Hamming decode]
  → [PAYLOAD: dewhiten → Hamming decode] → byte assembly → CRC check
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

    Example (SF=8, val=0b10110001):
        → [1, 0, 0, 0, 1, 1, 0, 1]
           ^LSB                 ^MSB
    """
    return [(val >> i) & 1 for i in range(sf)]


def bits_to_int_le(bits: List[int]) -> int:
    """
    Convert an LSB-first bit list back to an integer.

    Inverse of symbol_to_bits():
        bits_to_int_le([1, 0, 0, 0, 1, 1, 0, 1]) → 0xB1
    """
    v = 0
    for i, b in enumerate(bits):
        v |= (b & 1) << i
    return v


# ══════════════════════════════════════════════════════════════════════════════
#  §2 — DIAGONAL DEINTERLEAVER
# ══════════════════════════════════════════════════════════════════════════════
#
#  Notation (following Robyns et al. §2.2):
#    • C = 4 + CR        number of symbols (columns) per interleave block
#    • R = sf_rows        number of codewords (rows) produced per block
#                         (= SF for payload, = SF−2 for header reduced-rate)
#    • I[i][j]            the interleaved matrix as received:
#                           i ∈ [0, R)  = bit position within each symbol
#                           j ∈ [0, C)  = symbol index within the block
#    • D[j][k]            the deinterleaved matrix:
#                           j ∈ [0, C)  = bit position within a codeword
#                           k ∈ [0, R)  = codeword index
#
#  The TX interleaver wrote codewords along diagonals of I.
#  To undo this the RX applies:
#
#      D[j][k] = I[ (k + j + 1) mod R ][ j ]
#
#  This is algebraically equivalent to gr-lora's rotl-then-extract-columns
#  approach (decoder_impl.cc:543):
#      word = rotl(d_words[i], i, ppm)
#  where rotating symbol i left by i positions and reading column k gives
#  the same bit as reading row (k + i + 1) mod R of the un-rotated symbol.
# ──────────────────────────────────────────────────────────────────────────────

def deinterleave_block(symbol_block: List[int],
                       sf_rows: int,
                       sf: int,
                       cr_plus4: int) -> List[List[int]]:
    """
    Deinterleave one block of symbols into codewords.

    Parameters
    ----------
    symbol_block : list of `cr_plus4` symbol values (each 0 .. 2^SF − 1)
    sf_rows      : effective number of bit-rows to use per symbol
                   • SF     for normal payload
                   • SF − 2 for header (reduced-rate mode)
    sf           : spreading factor (needed for symbol_to_bits width)
    cr_plus4     : number of symbols per block = 4 + CR
                   (also the number of bits in each output codeword)

    Returns
    -------
    List of `sf_rows` codewords, each a list of `cr_plus4` bits (LSB first).
    """
    C = cr_plus4   # columns = symbols per block
    R = sf_rows    # rows    = codewords produced

    # ── Build the interleaved bit-matrix I[row][col] ──────────────────────
    #    Row i  = bit position i  (0 = LSB)
    #    Col j  = symbol index j  (0 .. C−1)
    #
    #  For header reduced-rate (R = SF−2) we use only the bottom R bits of
    #  each symbol; the top 2 bits were zeroed by the TX reduced-rate encoding.
    I = [[0] * C for _ in range(R)]
    for j, sym in enumerate(symbol_block):
        bits = symbol_to_bits(sym, sf)        # LSB-first, length SF
        for i in range(R):
            I[i][j] = bits[i]

    # ── Reverse the diagonal permutation ──────────────────────────────────
    #    D[j][k] = I[ (k + j + 1) mod R ][ j ]
    #
    #  j = bit position in the codeword   (0 .. C−1)
    #  k = codeword index                 (0 .. R−1)
    D = [[0] * R for _ in range(C)]
    for j in range(C):
        for k in range(R):
            i_idx = (k + (j + 1)) % R
            D[j][k] = I[i_idx][j]

    # ── Assemble codewords ────────────────────────────────────────────────
    #  Codeword k = [ D[0][k], D[1][k], …, D[C−1][k] ]
    #  Bit index j corresponds to the j-th bit of the (4+CR)-bit codeword,
    #  with j = 0 being the LSB.
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
#    • Coding rate    (3 bits)  — the CR used for the *payload*
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
    # ── CLI arguments ─────────────────────────────────────────────────────
    ap = argparse.ArgumentParser(
        description="LoRa RX deinterleaver: reverse the diagonal interleave "
                    "to recover (4+CR)-bit codewords from Gray-encoded symbols.")
    ap.add_argument('input', nargs='?', default='lora_symbols_gray.json',
                    help="Input JSON from the Gray-encode stage "
                         "(default: lora_symbols_gray.json)")
    ap.add_argument('-o', '--output', default='lora_deinterleaved.json',
                    help="Output JSON path (default: lora_deinterleaved.json)")
    args = ap.parse_args()

    # ── Load symbol data ──────────────────────────────────────────────────
    with open(args.input, 'r') as f:
        data = json.load(f)

    sf = data['config']['sf']

    # ── Header parameters ─────────────────────────────────────────────────
    #  The header is *always* transmitted with:
    #    • CR = 4          → 4 + 4 = 8 symbols per interleave block
    #    • Reduced-rate    → effective bit-rows = SF − 2
    #  (see gr-lora decoder_impl.cc: d_phdr.cr is set to 4 for the header,
    #   and deinterleave is called with ppm = SF − 2 during DECODE_HEADER)
    header_cr         = 4
    header_block_cols = 4 + header_cr       # 8 symbols per header block
    header_rows       = sf - 2              # reduced-rate: discard 2 MSBs

    # ── Separate symbols by role ──────────────────────────────────────────
    #  The Gray-encode stage tags each symbol with a 'role':
    #    'header'  — first block(s) after preamble
    #    'payload' — remaining data symbols
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

    # ── Deinterleave header ───────────────────────────────────────────────
    #  Process header symbols in blocks of 8 (= 4 + CR_header).
    #  Each block yields (SF − 2) codewords of 8 bits each.
    #
    #  gr-lora reference (decoder_impl.cc, DECODE_HEADER state ~line 830):
    #    deinterleave(sf - 2u)  // ppm = SF - 2 for header reduced-rate
    header_codewords_bits: List[List[int]] = []
    for blk_start in range(0, len(header_symbols), header_block_cols):
        block = header_symbols[blk_start : blk_start + header_block_cols]
        if len(block) < header_block_cols:
            break   # incomplete block — not enough symbols
        cw_bits = deinterleave_block(block, header_rows, sf, header_block_cols)
        header_codewords_bits.extend(cw_bits)

    header_cw_ints = [bits_to_int_le(b) for b in header_codewords_bits]

    result['header']['codewords_bits'] = header_codewords_bits
    result['header']['codewords_ints'] = header_cw_ints

    # NOTE: to fully decode the header you still need:
    #   Hamming(8,4) decode  (decoder_impl.cc:582)
    # Once decoded, the header tells us payload length, payload CR, and
    # whether a payload CRC is present.

    # ── Determine payload CR ──────────────────────────────────────────────
    #  Ideally this comes from the decoded header.  If the capture metadata
    #  already contains 'cr', use it; otherwise fall back to CR=4 until
    #  header decoding is implemented.
    payload_cr = data['config'].get('cr')
    if payload_cr is None:
        payload_cr = header_cr    # conservative fallback
    payload_block_cols = 4 + payload_cr   # symbols per payload interleave block
    payload_rows       = sf               # full SF rows for payload

    # ── Deinterleave payload ──────────────────────────────────────────────
    #  Process payload symbols in blocks of (4 + CR_payload).
    #  Each block yields SF codewords of (4 + CR_payload) bits each.
    #
    #  gr-lora reference (decoder_impl.cc, DECODE_PAYLOAD state ~line 860):
    #    deinterleave(sf)  // ppm = SF for payload
    payload_codewords_bits: List[List[int]] = []
    for blk_start in range(0, len(payload_symbols), payload_block_cols):
        block = payload_symbols[blk_start : blk_start + payload_block_cols]
        if len(block) < payload_block_cols:
            break   # incomplete block
        cw_bits = deinterleave_block(block, payload_rows, sf, payload_block_cols)
        payload_codewords_bits.extend(cw_bits)

    payload_cw_ints = [bits_to_int_le(b) for b in payload_codewords_bits]

    result['payload']['codewords_bits'] = payload_codewords_bits
    result['payload']['codewords_ints'] = payload_cw_ints

    # ── Write output ──────────────────────────────────────────────────────
    data['deinterleave'] = result
    with open(args.output, 'w') as f:
        json.dump(data, f, indent=2)

    # ── Summary ───────────────────────────────────────────────────────────
    print(f"SF = {sf}, header CR = {header_cr}, payload CR = {payload_cr}")
    print(f"Header  : {len(header_symbols):3d} symbols → "
          f"{len(header_codewords_bits):3d} codewords  (block size {header_block_cols})")
    print(f"Payload : {len(payload_symbols):3d} symbols → "
          f"{len(payload_codewords_bits):3d} codewords  (block size {payload_block_cols})")
    print(f"Wrote deinterleaved output to: {args.output}")


if __name__ == '__main__':
    main()