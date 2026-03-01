#!/usr/bin/env python3
"""
lora_hamming_decode.py — LoRa Hamming(8,4) Decode + Byte Assembly (RX Stage 7)
================================================================================

Decodes Hamming(8,4)-protected codewords into data nibbles, then packs them
into bytes and parses the header fields.

═══════════════════════════════════════════════════════════════════════════════
  HAMMING(8,4) CODEWORD LAYOUT
═══════════════════════════════════════════════════════════════════════════════

The 8-bit codeword has this fixed bit layout:

    Bit position:   7    6    5    4    3    2    1    0
    Role:          p4   p3   d3   p2   d2   d1   d0   p1

  • p1..p4 are parity bits
  • d0..d3 are data bits  (these form the 4-bit nibble we want to extract)

Each parity bit covers a specific set of data bits:

    p1 (bit 0) checks d1, d2, d3   (bits 2, 3, 5)
    p2 (bit 4) checks d0, d1, d2   (bits 1, 2, 3)
    p3 (bit 6) checks d0, d1, d3   (bits 1, 2, 5)
    p4 (bit 7) checks d0, d2, d3   (bits 1, 3, 5)

gr-lora reference:   utilities.h hamming_decode_soft_byte() ~line 288
═══════════════════════════════════════════════════════════════════════════════

═══════════════════════════════════════════════════════════════════════════════
  SYNDROME-BASED ERROR CORRECTION
═══════════════════════════════════════════════════════════════════════════════

A 4-bit syndrome is computed from (p1≠p1c, p2≠p2c, p3≠p3c, p4≠p4c).
If the syndrome is non-zero, a lookup table identifies the errored bit:

    Syndrome →  0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15
    Flip bit →  -  0  4  -  6  -  -  2  7  -  -  3  -  5  1  -

    (-) means the syndrome maps to bit 0 in the LUT; for syndromes that
    don't correspond to a valid single-bit error this may miscorrect,
    but Hamming(8,4) can only correct single-bit errors anyway.

After correction, the data nibble is extracted from bit positions {1,2,3,5}:
    nibble = pack_nibble(bit1, bit2, bit3, bit5)

═══════════════════════════════════════════════════════════════════════════════

═══════════════════════════════════════════════════════════════════════════════
  NIBBLE-TO-BYTE PACKING
═══════════════════════════════════════════════════════════════════════════════

Codewords are processed in pairs to produce one byte each:

    Header  : byte = (d1 << 4) | d2     d1 from even codeword, d2 from odd
    Payload : byte = (d2 << 4) | d1     note: opposite order!

This matches gr-lora's hamming_decode_soft() (deprecated but equivalent):
    decoder_impl.cc ~line 680

The non-deprecated path uses fec_decode() + swap_nibbles() for payload,
which achieves the same result.

═══════════════════════════════════════════════════════════════════════════════

═══════════════════════════════════════════════════════════════════════════════
  HEADER STRUCTURE (loraphy_header_t)
═══════════════════════════════════════════════════════════════════════════════

  byte 0:  length           (8 bits)   — payload length in bytes
  byte 1:  crc_msn          (bits 0–3) — header CRC most-significant nibble
           has_mac_crc      (bit  4)   — 1 if payload CRC is present
           cr               (bits 5–7) — coding rate for payload (1–4)
  byte 2:  crc_lsn          (bits 0–3) — header CRC least-significant nibble
           reserved         (bits 4–7) — unused

  Note: gr-lora replaces the 6th header codeword with 0, so crc_lsn = 0
  in the decoded output.  The full header CRC is only partially recoverable.

  Reference: gr-lora/include/lora/loraphy.h

═══════════════════════════════════════════════════════════════════════════════

Pipeline position:
  … → Deshuffle → Dewhiten → **[Hamming Decode + Byte Assembly]**
  → (CRC check)
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
#  The two implementations agree for valid and single-error codewords but
#  may diverge for multi-bit errors.  We use the LUT to match gr-lora.
#
#  Source: liquid-dsp/src/fec/src/fec_hamming84.c
#
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
HAMMING84_ENC_LUT = [
    0x00, 0xd2, 0x55, 0x87, 0x99, 0x4b, 0xcc, 0x1e,
    0xe1, 0x33, 0xb4, 0x66, 0x78, 0xaa, 0x2d, 0xff,
]


def hamming84_decode(v: int) -> Tuple[int, bool]:
    """
    Decode one Hamming(8,4) codeword using liquid-dsp's LUT.

    This matches the actual fec_decode path used by gr-lora:
        s = hamming84_dec_gentab[v]

    Parameters
    ----------
    v : int   8-bit codeword.

    Returns
    -------
    (nibble, corrected) : tuple
        nibble    — decoded 4-bit data value (0–15)
        corrected — True if the codeword was not a perfect match
                    (i.e., the re-encoded nibble differs from the input)
    """
    nibble = HAMMING84_DEC_LUT[v & 0xFF]
    # Check if error correction was applied
    corrected = (HAMMING84_ENC_LUT[nibble] != (v & 0xFF))
    return nibble, corrected


# ══════════════════════════════════════════════════════════════════════════════
#  §2 — NIBBLE-TO-BYTE PACKING
# ══════════════════════════════════════════════════════════════════════════════

def decode_codewords_to_bytes(codewords: List[int],
                              is_header: bool) -> Tuple[List[int], List[dict]]:
    """
    Hamming-decode a list of codewords and pack nibbles into bytes.

    Processes codewords in pairs:
      d1 = hamming_decode(codeword[i])        — even-index codeword
      d2 = hamming_decode(codeword[i+1])      — odd-index  codeword

      Header  : byte = (d1 << 4) | d2
      Payload : byte = (d2 << 4) | d1

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
        d1, c1 = hamming84_decode(codewords[i])

        if i + 1 < len(codewords):
            d2, c2 = hamming84_decode(codewords[i + 1])
        else:
            d2, c2 = 0, False   # odd count: pad with zero nibble

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

def parse_header(header_bytes: List[int]) -> dict:
    """
    Extract LoRa PHY header fields from the 3-byte decoded header.

    Layout (loraphy_header_t, packed struct — little-endian bit order):

        byte 0:  length                    (8 bits)
        byte 1:  crc_msn[3:0]             (bits 0–3)
                 has_mac_crc              (bit 4)
                 cr[2:0]                  (bits 5–7)
        byte 2:  crc_lsn[3:0]             (bits 0–3)
                 reserved[3:0]            (bits 4–7)

    Reference: gr-lora/include/lora/loraphy.h
    """
    if len(header_bytes) < 3:
        return {'error': f'Expected 3 header bytes, got {len(header_bytes)}'}

    b0 = header_bytes[0]
    b1 = header_bytes[1]
    b2 = header_bytes[2]

    length      = b0
    crc_msn     = b1 & 0x0F
    has_mac_crc = (b1 >> 4) & 0x01
    cr          = (b1 >> 5) & 0x07
    crc_lsn     = b2 & 0x0F
    reserved    = (b2 >> 4) & 0x0F

    # Reconstruct header CRC (8-bit, from two nibbles)
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
    #  6 codewords → 3 bytes (nibble pairs packed as (d1<<4)|d2)
    header_bytes, header_detail = decode_codewords_to_bytes(
        header_cw, is_header=True)

    # ── Parse header fields ───────────────────────────────────────────────
    header_fields = parse_header(header_bytes)

    # ── Hamming decode + byte assembly — payload ──────────────────────────
    #  N codewords → ceil(N/2) bytes (nibble pairs packed as (d2<<4)|d1)
    payload_bytes, payload_detail = decode_codewords_to_bytes(
        payload_cw, is_header=False)

    # ── Extract payload data ──────────────────────────────────────────────
    #  The header tells us how many payload bytes are real data vs CRC.
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
