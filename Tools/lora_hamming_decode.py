#!/usr/bin/env python3
"""
lora_hamming_decode.py — LoRa Hamming(8,4) Decode + Byte Assembly (RX Stage 7)
================================================================================

Decodes Hamming(8,4)-protected codewords into data nibbles, then packs them
into bytes and parses the header fields.

This is the FINAL stage of the LoRa decode pipeline. After all the RF-level
processing (dechirp → gray → deinterleave → deshuffle → dewhiten), we now
have clean 8-bit codewords that need to be decoded into actual user data.

═══════════════════════════════════════════════════════════════════════════════
  WHAT IS HAMMING CODE AND WHY DOES LoRa USE IT?
═══════════════════════════════════════════════════════════════════════════════

Hamming codes are a family of Forward Error Correction (FEC) codes invented
by Richard Hamming in 1950. They add redundant "parity" bits to the data so
the receiver can DETECT and CORRECT bit errors without retransmission.

LoRa uses Hamming(8,4) for CR=4, meaning:
  • 4 data bits   (the actual nibble we want)
  • 4 parity bits (redundancy for error protection)
  • 8 bits total  (one codeword)

This gives LoRa the ability to:
  • Detect up to 2 bit errors per codeword
  • Correct 1 bit error per codeword

The trade-off is bandwidth: we transmit 8 bits to carry only 4 bits of data
(50% overhead). Lower CR values (1, 2, 3) use less redundancy (5, 6, 7 bits
per codeword) for better throughput, but weaker error correction.

═══════════════════════════════════════════════════════════════════════════════
  HAMMING(8,4) CODEWORD LAYOUT
═══════════════════════════════════════════════════════════════════════════════

The 8-bit codeword has this fixed bit layout:

    Bit position:   7    6    5    4    3    2    1    0
    Role:          p4   p3   d3   p2   d2   d1   d0   p1

  • p1..p4 are parity bits  (positions 0, 4, 6, 7)
  • d0..d3 are data bits    (positions 1, 2, 3, 5)

Why are data bits at positions {1, 2, 3, 5} and not {0, 1, 2, 3}?
  In classic Hamming codes, parity bits sit at power-of-2 positions
  (1, 2, 4). Position 0 is used for an overall parity check (SECDED).
  But LoRa's layout is slightly different — it follows the specific
  convention from standard Hamming(8,4) tables (see the generator
  matrix in the liquid-dsp source).

Each parity bit covers a specific set of data bits:

    p1 (bit 0) checks d1, d2, d3   (bits 2, 3, 5)
    p2 (bit 4) checks d0, d1, d2   (bits 1, 2, 3)
    p3 (bit 6) checks d0, d1, d3   (bits 1, 2, 5)
    p4 (bit 7) checks d0, d2, d3   (bits 1, 3, 5)

    Notice how each data bit is covered by a UNIQUE combination of parity
    bits. This is what allows the syndrome to pinpoint exactly which bit
    was corrupted.

gr-lora reference:   utilities.h hamming_decode_soft_byte() ~line 288
═══════════════════════════════════════════════════════════════════════════════

═══════════════════════════════════════════════════════════════════════════════
  SYNDROME-BASED ERROR CORRECTION
═══════════════════════════════════════════════════════════════════════════════

How error correction works (conceptual):
  1. Receive the 8-bit codeword
  2. Recompute what each parity bit SHOULD be from the data bits
  3. Compare recomputed vs. received parity → the 4-bit "syndrome"
  4. If syndrome = 0, no errors. Otherwise, the syndrome identifies
     which bit was flipped.

A 4-bit syndrome is computed from (p1≠p1c, p2≠p2c, p3≠p3c, p4≠p4c).
If the syndrome is non-zero, a lookup table identifies the errored bit:

    Syndrome →  0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15
    Flip bit →  -  0  4  -  6  -  -  2  7  -  -  3  -  5  1  -

    (-) means the syndrome maps to bit 0 in the LUT (Look-Up Table); for syndromes that
    don't correspond to a valid single-bit error this may miscorrect,
    but Hamming(8,4) can only correct single-bit errors anyway.

In practice, neither gr-lora nor we use the syndrome approach directly.
Instead, liquid-dsp precomputes a 256-entry lookup table that maps EVERY
possible 8-bit input directly to the corrected 4-bit nibble. This is
faster (one array lookup vs. computing 4 parities + a conditional flip)
and avoids the syndrome logic entirely.

After correction, the data nibble is extracted from bit positions {1,2,3,5}:
    nibble = pack_nibble(bit1, bit2, bit3, bit5)

═══════════════════════════════════════════════════════════════════════════════

═══════════════════════════════════════════════════════════════════════════════
  NIBBLE-TO-BYTE PACKING
═══════════════════════════════════════════════════════════════════════════════

Each codeword produces one 4-bit nibble. To reconstruct a full byte, we
need TWO codewords (two nibbles = 8 bits = 1 byte).

Codewords are processed in pairs to produce one byte each:

    Header  : byte = (d1 << 4) | d2     d1 from even codeword, d2 from odd
    Payload : byte = (d2 << 4) | d1     note: opposite order!

Why are the nibble orders different for header vs. payload?
  This is a quirk of gr-lora's implementation. The non-deprecated code
  path uses liquid-dsp's fec_decode() which outputs nibbles in one order,
  then explicitly calls swap_nibbles() for the payload to match what the
  LoRa modem actually does. The header does NOT get the swap.
  The deprecated hamming_decode_soft() did it inline (decoder_impl.cc ~line 680)
  with different shift/mask logic for header vs. payload, achieving the
  same result.

═══════════════════════════════════════════════════════════════════════════════

═══════════════════════════════════════════════════════════════════════════════
  HEADER STRUCTURE (loraphy_header_t)
═══════════════════════════════════════════════════════════════════════════════

The LoRa explicit-mode header is always 3 bytes (decoded from 6 codewords).
It tells the receiver everything it needs to decode the payload:

  byte 0:  length           (8 bits)   — payload length in bytes
  byte 1:  crc_msn          (bits 0-3) — header CRC most-significant nibble
           has_mac_crc      (bit  4)   — 1 if payload CRC is present
           cr               (bits 5-7) — coding rate for payload (1-4)
  byte 2:  crc_lsn          (bits 0-3) — header CRC least-significant nibble
           reserved         (bits 4-7) — unused

  The header CRC (8 bits = crc_msn || crc_lsn) is computed over byte 0 and
  the upper nibble of byte 1. It lets the receiver verify the header before
  attempting to decode the payload.

  Note: gr-lora replaces the 6th header codeword with 0 (see Stage 5
  deshuffle notes), so crc_lsn = 0 in the decoded output. The full header
  CRC is only partially recoverable from our pipeline.

  Reference: gr-lora/include/lora/loraphy.h

═══════════════════════════════════════════════════════════════════════════════

═══════════════════════════════════════════════════════════════════════════════
  MOCK EXAMPLE: Hamming Decode + Byte Assembly (Pen-and-Paper)
═══════════════════════════════════════════════════════════════════════════════

Suppose we have 2 dewhitened header codewords: [0x87, 0x99]

STEP 1: Hamming decode each codeword via LUT:

  Codeword 0x87 = 1000_0111 (binary)
    LUT[0x87] = 0x3 → nibble d1 = 3 (binary: 0011)
    Verify: ENC_LUT[3] = 0x87 ✓ (no correction needed)

  Codeword 0x99 = 1001_1001 (binary)
    LUT[0x99] = 0x4 → nibble d2 = 4 (binary: 0100)
    Verify: ENC_LUT[4] = 0x99 ✓ (no correction needed)

STEP 2: Pack nibbles into a byte (header ordering):
    byte = (d1 << 4) | d2 = (0x3 << 4) | 0x4 = 0x34

STEP 3 (error correction example):
  What if noise flipped bit 3 of codeword 0x87?
    Received: 0x8F = 1000_1111
    LUT[0x8F] = 0x3 → still decodes to nibble 3!
    But ENC_LUT[3] = 0x87 ≠ 0x8F → corrected flag = True
    The LUT absorbed the error and still produced the right nibble.

═══════════════════════════════════════════════════════════════════════════════

"""

import json
import argparse
import math
from typing import List, Tuple


# ══════════════════════════════════════════════════════════════════════════════
#  §1 — HAMMING(8,4) DECODER
# ══════════════════════════════════════════════════════════════════════════════
#
#  IMPORTANT: gr-lora uses liquid-dsp's fec_hamming84_decode(), NOT the
#  deprecated hamming_decode_soft_byte() from utilities.h.  The liquid-dsp
#  implementation uses a 256-entry lookup table (hamming84_dec_gentab[])
#  that maps each possible 8-bit received codeword directly to the
#  corrected 4-bit nibble.
#
#  How the LUT works:
#    Index = the raw 8-bit codeword you received (0x00 to 0xFF)
#    Value = the corrected 4-bit data nibble (0x0 to 0xF)
#
#  Example:
#    LUT[0xD2] = 0x1  → codeword 0xD2 decodes to nibble 1
#    LUT[0xD3] = 0x1  → codeword 0xD3 also decodes to nibble 1
#                        (0xD3 has a 1-bit error vs. the valid 0xD2;
#                         the LUT silently corrects it)
#
#  The two implementations (LUT vs. syndrome) agree for valid and single-error
#  codewords but may diverge for multi-bit errors. We use the LUT to match
#  gr-lora's actual behavior.
#
#  Source: liquid-dsp/src/fec/src/fec_hamming84.c
#

# Decoder LUT: hamming84_dec_gentab[received_codeword] → corrected nibble
# 256 entries covering every possible 8-bit received value.
HAMMING84_DEC_LUT = [
    0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x3, 0x3,
    0x0, 0x0, 0x5, 0x5, 0xe, 0xe, 0x7, 0x7,
    0x0, 0x0, 0x9, 0x9, 0x2, 0x2, 0x7, 0x7,
    0x4, 0x4, 0x7, 0x7, 0x7, 0x7, 0x7, 0x7,
    0x0, 0x0, 0x9, 0x9, 0xe, 0xe, 0xb, 0xb,
    0xe, 0xe, 0xd, 0xd, 0xe, 0xe, 0xe, 0xe,
    0x9, 0x9, 0x9, 0x9, 0xa, 0xa, 0x9, 0x9,
    0xc, 0xc, 0x9, 0x9, 0xe, 0xe, 0x7, 0x7,
    0x0, 0x0, 0x5, 0x5, 0x2, 0x2, 0xb, 0xb,
    0x5, 0x5, 0x5, 0x5, 0x6, 0x6, 0x5, 0x5,
    0x2, 0x2, 0x1, 0x1, 0x2, 0x2, 0x2, 0x2,
    0xc, 0xc, 0x5, 0x5, 0x2, 0x2, 0x7, 0x7,
    0x8, 0x8, 0xb, 0xb, 0xb, 0xb, 0xb, 0xb,
    0xc, 0xc, 0x5, 0x5, 0xe, 0xe, 0xb, 0xb,
    0xc, 0xc, 0x9, 0x9, 0x2, 0x2, 0xb, 0xb,
    0xc, 0xc, 0xc, 0xc, 0xc, 0xc, 0xf, 0xf,
    0x0, 0x0, 0x3, 0x3, 0x3, 0x3, 0x3, 0x3,
    0x4, 0x4, 0xd, 0xd, 0x6, 0x6, 0x3, 0x3,
    0x4, 0x4, 0x1, 0x1, 0xa, 0xa, 0x3, 0x3,
    0x4, 0x4, 0x4, 0x4, 0x4, 0x4, 0x7, 0x7,
    0x8, 0x8, 0xd, 0xd, 0xa, 0xa, 0x3, 0x3,
    0xd, 0xd, 0xd, 0xd, 0xe, 0xe, 0xd, 0xd,
    0xa, 0xa, 0x9, 0x9, 0xa, 0xa, 0xa, 0xa,
    0x4, 0x4, 0xd, 0xd, 0xa, 0xa, 0xf, 0xf,
    0x8, 0x8, 0x1, 0x1, 0x6, 0x6, 0x3, 0x3,
    0x6, 0x6, 0x5, 0x5, 0x6, 0x6, 0x6, 0x6,
    0x1, 0x1, 0x1, 0x1, 0x2, 0x2, 0x1, 0x1,
    0x4, 0x4, 0x1, 0x1, 0x6, 0x6, 0xf, 0xf,
    0x8, 0x8, 0x8, 0x8, 0x8, 0x8, 0xb, 0xb,
    0x8, 0x8, 0xd, 0xd, 0x6, 0x6, 0xf, 0xf,
    0x8, 0x8, 0x1, 0x1, 0xa, 0xa, 0xf, 0xf,
    0xc, 0xc, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf,
]

# Encoder LUT (for debug / verification):
#   hamming84_enc_gentab[nibble] → 8-bit codeword
#
# This is the INVERSE of the decode table: given a 4-bit nibble, it produces
# the valid 8-bit Hamming(8,4) codeword. We use it to detect whether error
# correction was applied: if ENC_LUT[decoded_nibble] != received_codeword,
# then the codeword had an error that the LUT corrected.
#
# There are only 16 valid codewords (one per nibble 0-F). All other 240
# possible 8-bit values are "invalid" and will be corrected to the nearest
# valid codeword by the decoder LUT.
#
# Fun fact: these are the same values used in the dewhitening PRNG table!
# (See Stage 6 notes about why the PRNG entries look like Hamming codewords.)
HAMMING84_ENC_LUT = [
    0x00, 0xd2, 0x55, 0x87, 0x99, 0x4b, 0xcc, 0x1e,
    0xe1, 0x33, 0xb4, 0x66, 0x78, 0xaa, 0x2d, 0xff,
]


def hamming84_decode(v: int) -> Tuple[int, bool]:
    """
    Decode one Hamming(8,4) codeword using liquid-dsp's LUT.

    This matches the actual fec_decode path used by gr-lora:
        s = hamming84_dec_gentab[v]

    The decode is a single array lookup — O(1), no math required.
    We then check if correction was applied by re-encoding the nibble
    and comparing against the original input:
        If ENC_LUT[nibble] == v  → codeword was valid, no correction
        If ENC_LUT[nibble] != v  → codeword had an error (now corrected)

    Parameters
    ----------
    v : int   8-bit codeword (dewhitened, from Stage 6).

    Returns
    -------
    (nibble, corrected) : tuple
        nibble    — decoded 4-bit data value (0–15)
        corrected — True if the codeword was not a perfect match
                    (i.e., the re-encoded nibble differs from the input,
                     meaning the LUT applied error correction)
    """
    nibble = HAMMING84_DEC_LUT[v & 0xFF]
    # Re-encode the decoded nibble and compare to the received codeword.
    # If they differ, the LUT corrected an error.
    corrected = (HAMMING84_ENC_LUT[nibble] != (v & 0xFF))
    return nibble, corrected


# ══════════════════════════════════════════════════════════════════════════════
#  §2 — NIBBLE-TO-BYTE PACKING
# ══════════════════════════════════════════════════════════════════════════════
#
#  After Hamming decoding, each codeword yields a 4-bit nibble. We need
#  to combine pairs of nibbles back into bytes.
#
#  Visual example (header, 6 codewords → 3 bytes):
#
#    CW[0] → nibble A    CW[1] → nibble B    → byte 0 = (A << 4) | B
#    CW[2] → nibble C    CW[3] → nibble D    → byte 1 = (C << 4) | D
#    CW[4] → nibble E    CW[5] → nibble F    → byte 2 = (E << 4) | F
#
#  For payload, the nibble order within each pair is SWAPPED:
#    CW[0] → nibble A    CW[1] → nibble B    → byte 0 = (B << 4) | A
#
#  This swap comes from how gr-lora routes the data through liquid-dsp's
#  fec_decode(), which outputs nibbles in a different order than the
#  deprecated hamming_decode_soft_byte(). gr-lora compensates by calling
#  swap_nibbles() on the payload (but NOT the header).
#

def decode_codewords_to_bytes(codewords: List[int],
                              is_header: bool) -> Tuple[List[int], List[dict]]:
    """
    Hamming-decode a list of codewords and pack nibbles into bytes.

    Processes codewords in pairs:
      d1 = hamming_decode(codeword[i])        — even-index codeword
      d2 = hamming_decode(codeword[i+1])      — odd-index  codeword

      Header  : byte = (d1 << 4) | d2
      Payload : byte = (d2 << 4) | d1     ← nibbles swapped!

    This replicates gr-lora's hamming_decode_soft() (decoder_impl.cc ~line 680).

    Parameters
    ----------
    codewords : list of int
        Dewhitened 8-bit codeword values.
    is_header : bool
        True for header (different nibble ordering).

    Returns
    -------
    (bytes_out, detail) : tuple
        bytes_out — list of decoded bytes
        detail    — list of dicts with per-pair decode info for debug output
    """
    decoded_bytes = []
    detail = []

    for i in range(0, len(codewords), 2):
        # Decode even-index codeword → first nibble
        d1, c1 = hamming84_decode(codewords[i])

        # Decode odd-index codeword → second nibble
        if i + 1 < len(codewords):
            d2, c2 = hamming84_decode(codewords[i + 1])
        else:
            d2, c2 = 0, False   # odd count: pad with zero nibble

        # Pack the two nibbles into one byte.
        # Header and payload use opposite nibble ordering.
        if is_header:
            byte_val = (d1 << 4) | d2
        else:
            byte_val = (d2 << 4) | d1

        decoded_bytes.append(byte_val)
        detail.append({
            'pair': [i, i + 1 if i + 1 < len(codewords) else None],
            'codewords': [codewords[i],
                          codewords[i + 1] if i + 1 < len(codewords) else 0],
            'nibbles': [d1, d2],
            'corrected': [c1, c2],
            'byte': byte_val,
        })

    return decoded_bytes, detail


# ══════════════════════════════════════════════════════════════════════════════
#  §3 — HEADER FIELD EXTRACTION
# ══════════════════════════════════════════════════════════════════════════════
#
#  The header is the "table of contents" for the LoRa packet. It MUST be
#  decoded first (and correctly!) because it tells us:
#    • How many payload bytes to expect (length)
#    • What coding rate the payload uses (cr)
#    • Whether the payload has a CRC appended (has_mac_crc)
#
#  Without these, we wouldn't know when the payload ends or how to decode it.
#  This is why the header always uses CR=4 (maximum protection) regardless
#  of what the payload uses — losing the header means losing the packet.
#
#  Visual byte layout (6 codewords → 3 bytes):
#
#    byte 0: [  L7   L6   L5   L4   L3   L2   L1   L0  ]  ← length
#    byte 1: [ CR2  CR1  CR0  CRC  C3   C2   C1   C0   ]  ← cr, has_mac_crc, crc_msn
#    byte 2: [ R3   R2   R1   R0   C3   C2   C1   C0   ]  ← reserved, crc_lsn
#

def parse_header(header_bytes: List[int]) -> dict:
    """
    Extract LoRa PHY header fields from the 3-byte decoded header.

    Layout (loraphy_header_t, packed struct — little-endian bit order):

        byte 0:  length                    (8 bits)
        byte 1:  crc_msn[3:0]             (bits 0–3) — header CRC MSN
                 has_mac_crc              (bit 4)    — 1 = payload CRC present
                 cr[2:0]                  (bits 5–7) — payload coding rate
        byte 2:  crc_lsn[3:0]             (bits 0–3) — header CRC LSN
                 reserved[3:0]            (bits 4–7) — unused

    The header CRC (8 bits total, split across two nibbles) is a checksum
    over byte 0 and the upper nibble of byte 1. The receiver uses it to
    verify header integrity before committing to decode the payload.

    Reference: gr-lora/include/lora/loraphy.h
    """
    if len(header_bytes) < 3:
        return {'error': f'Expected 3 header bytes, got {len(header_bytes)}'}

    b0 = header_bytes[0]
    b1 = header_bytes[1]
    b2 = header_bytes[2]

    # Byte 0: entire byte is the payload length
    length      = b0

    # Byte 1: packed fields (extracted with bit masks)
    crc_msn     = b1 & 0x0F          # lower 4 bits: header CRC MSN
    has_mac_crc = (b1 >> 4) & 0x01   # bit 4: payload CRC flag
    cr          = (b1 >> 5) & 0x07   # bits 5-7: coding rate (1-4)

    # Byte 2: packed fields
    crc_lsn     = b2 & 0x0F          # lower 4 bits: header CRC LSN
    reserved    = (b2 >> 4) & 0x0F   # upper 4 bits: unused

    # Reconstruct the full 8-bit header CRC from its two nibbles.
    # Note: crc_lsn comes from the 6th header codeword, which gr-lora
    # replaced with 0x00 in Stage 5 (deshuffle), so it will always be 0.
    # The full CRC is therefore only partially recoverable.
    header_crc  = (crc_msn << 4) | crc_lsn

    return {
        'raw_bytes': [f'0x{b:02X}' for b in header_bytes],
        'length':      length,
        'cr':          cr,
        'has_mac_crc': has_mac_crc,
        'crc_msn':     crc_msn,
        'crc_lsn':     crc_lsn,
        'header_crc':  f'0x{header_crc:02X}',
        'reserved':    reserved,
    }


# ══════════════════════════════════════════════════════════════════════════════
#  §4 — MAIN
# ══════════════════════════════════════════════════════════════════════════════

def main():
    ap = argparse.ArgumentParser(
        description="LoRa RX Hamming(8,4) decode + byte assembly: "
                    "decode codewords, pack nibbles into bytes, "
                    "and parse the header fields.")
    ap.add_argument('input', nargs='?', default='lora_dewhitened.json',
                    help="Input JSON from dewhiten stage "
                         "(default: lora_dewhitened.json)")
    ap.add_argument('-o', '--output', default='lora_decoded.json',
                    help="Output JSON path (default: lora_decoded.json)")
    args = ap.parse_args()

    # ── Load dewhitened data ──────────────────────────────────────────────
    with open(args.input, 'r') as f:
        data = json.load(f)

    dw = data['dewhitened']
    header_cw  = dw['header']['codewords_ints']
    payload_cw = dw['payload']['codewords_ints']

    # ── Hamming decode + byte assembly — header ───────────────────────────
    #  6 codewords → 3 bytes (nibble pairs packed as (d1<<4)|d2).
    #  The header always uses CR=4 with 8-bit codewords, so the Hamming(8,4)
    #  LUT applies directly. The header is decoded FIRST because we need
    #  its fields (payload length, CR, CRC flag) to properly decode the
    #  payload that follows.
    header_bytes, header_detail = decode_codewords_to_bytes(
        header_cw, is_header=True)

    # ── Parse header fields ───────────────────────────────────────────────
    #  Extract length, CR, has_mac_crc, and header CRC from the 3 decoded
    #  bytes. In a production decoder, we'd verify the header CRC here
    #  before proceeding. If it fails, the packet is discarded.
    header_fields = parse_header(header_bytes)

    # ── Hamming decode + byte assembly — payload ──────────────────────────
    #  N codewords → ceil(N/2) bytes (nibble pairs packed as (d2<<4)|d1).
    #  Note the SWAPPED nibble order compared to header — this is the
    #  swap_nibbles() quirk described in the module docstring.
    payload_bytes, payload_detail = decode_codewords_to_bytes(
        payload_cw, is_header=False)

    # ── Extract payload data ──────────────────────────────────────────────
    #  The header's 'length' field tells us how many payload bytes are real
    #  data. If has_mac_crc is set, there are 2 additional bytes at the end
    #  containing a CRC-16 over the payload data.
    #
    #  Layout of decoded payload bytes:
    #    [  data (length bytes)  |  CRC-16 (2 bytes, if has_mac_crc)  ]
    payload_length = header_fields.get('length', len(payload_bytes))
    has_mac_crc    = header_fields.get('has_mac_crc', 0)
    mac_crc_size   = 2 if has_mac_crc else 0

    total_payload_bytes = payload_length + mac_crc_size

    payload_data = payload_bytes[:payload_length]
    payload_crc  = payload_bytes[payload_length:total_payload_bytes]

    # ── Store full results ────────────────────────────────────────────────
    data['decoded'] = {
        'header': {
            'bytes':   [f'0x{b:02X}' for b in header_bytes],
            'fields':  header_fields,
            'detail':  header_detail,
        },
        'payload': {
            'all_bytes':     [f'0x{b:02X}' for b in payload_bytes],
            'data':          [f'0x{b:02X}' for b in payload_data],
            'data_hex':      ''.join(f'{b:02X}' for b in payload_data),
            'data_ascii':    ''.join(chr(b) if 32 <= b < 127 else '.'
                                     for b in payload_data),
            'mac_crc':       [f'0x{b:02X}' for b in payload_crc] if payload_crc else None,
            'detail':        payload_detail,
        },
    }

    with open(args.output, 'w') as f:
        json.dump(data, f, indent=2)

    # ── Summary ───────────────────────────────────────────────────────────
    def fmt_hex(vals):
        return ' '.join(f'0x{v:02X}' for v in vals)

    print("═══════════════ HEADER ═══════════════")
    print(f"  Codewords ({len(header_cw)}): "
          f"{fmt_hex(header_cw)}")
    print(f"  Decoded bytes ({len(header_bytes)}): "
          f"{fmt_hex(header_bytes)}")
    print()

    hf = header_fields
    print(f"  Payload length : {hf.get('length', '?')} byte(s)")
    print(f"  Coding rate    : {hf.get('cr', '?')}")
    print(f"  Has MAC CRC    : {hf.get('has_mac_crc', '?')}")
    print(f"  Header CRC     : {hf.get('header_crc', '?')}")
    print(f"  Reserved       : {hf.get('reserved', '?')}")

    # Show per-pair details
    for d in header_detail:
        cws = d['codewords']
        nib = d['nibbles']
        cor = d['corrected']
        flag = ' ← corrected' if any(cor) else ''
        print(f"    pair {d['pair']}: "
              f"0x{cws[0]:02X},0x{cws[1]:02X} → "
              f"nibbles {nib[0]:X},{nib[1]:X} → 0x{d['byte']:02X}{flag}")

    print()
    print("═══════════════ PAYLOAD ══════════════")
    print(f"  Codewords ({len(payload_cw)}): "
          f"{fmt_hex(payload_cw)}")
    print(f"  Decoded bytes ({len(payload_bytes)}): "
          f"{fmt_hex(payload_bytes)}")
    print()
    print(f"  Data ({payload_length} byte(s)):  "
          f"{fmt_hex(payload_data)}  "
          f"  ASCII: \"{data['decoded']['payload']['data_ascii']}\"")

    if payload_crc:
        print(f"  MAC CRC:          {fmt_hex(payload_crc)}")

    for d in payload_detail:
        cws = d['codewords']
        nib = d['nibbles']
        cor = d['corrected']
        flag = ' ← corrected' if any(cor) else ''
        print(f"    pair {d['pair']}: "
              f"0x{cws[0]:02X},0x{cws[1]:02X} → "
              f"nibbles {nib[0]:X},{nib[1]:X} → 0x{d['byte']:02X}{flag}")

    print()
    print(f"Wrote decoded output to: {args.output}")


if __name__ == '__main__':
    main()
