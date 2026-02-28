#!/usr/bin/env bash
# ══════════════════════════════════════════════════════════════════════════════
#  lora_decoder.sh — End-to-end LoRa decode pipeline
# ══════════════════════════════════════════════════════════════════════════════
#
#  Stage 1: Capture IQ samples from RTL-SDR            → last_capture.npy
#  Stage 2: Dechirp & extract symbol values            → lora_symbols.json
#  Stage 3: Recover symbol values (Gray encode bins)   → lora_symbols_gray.json
#  Stage 4: Deinterleave symbols into codewords        → lora_deinterleaved.json
#  Stage 5: (future) HEADER: Hamming decode
#  Stage 6: (future) PAYLOAD: dewhiten → Hamming decode
#  Stage 7: (future) Byte assembly + CRC check
#
# ── KNOWN LIMITATIONS ─────────────────────────────────────────────────────────
#  This pipeline is currently hardcoded for:
#    • SF = 8
#    • CR = 4  (coding rate 4/8)
#    • 1-byte payload
#    • Explicit header mode
#  Several values are not yet dynamic:
#    • N_SYMBOLS (Stage 2) assumes ~32 symbols total (2 pre + 8 preamble +
#      2 sync + 3 downchirp + 8 header + 8 payload = 31).
#      Longer payloads or different SF/CR will need a larger N_SYMBOLS.
#    • Stage 2 symbol role annotations use hardcoded index ranges.
#    • Stage 4 falls back to CR=4 if the header hasn't been decoded yet.
#
# ── Usage ─────────────────────────────────────────────────────────────────────
#   bash lora_decoder.sh              # full pipeline: capture + symbol extraction
#   bash lora_decoder.sh --skip-capture   # reuse existing last_capture.npy
#
# ══════════════════════════════════════════════════════════════════════════════
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ── Parse arguments ───────────────────────────────────────────────────────────
SKIP_CAPTURE=false
for arg in "$@"; do
    case "$arg" in
        --skip-capture) SKIP_CAPTURE=true ;;
        -h|--help)
            echo "Usage: $0 [--skip-capture]"
            echo "  --skip-capture   Skip RTL-SDR capture, reuse existing last_capture.npy"
            exit 0
            ;;
        *)
            echo "Unknown argument: $arg"
            exit 1
            ;;
    esac
done

# ── Stage 1: Capture ─────────────────────────────────────────────────────────
if [ "$SKIP_CAPTURE" = false ]; then
    echo "════════════════════════════════════════════════════════════════"
    echo "  Stage 1: Capturing IQ samples from RTL-SDR"
    echo "════════════════════════════════════════════════════════════════"
    bash "${SCRIPT_DIR}/capture_rtl_sdr_v4.sh"
    echo
else
    echo "════════════════════════════════════════════════════════════════"
    echo "  Stage 1: SKIPPED (--skip-capture) — reusing last_capture.npy"
    echo "════════════════════════════════════════════════════════════════"
    if [ ! -f "last_capture.npy" ]; then
        echo "ERROR: last_capture.npy not found. Run without --skip-capture first."
        exit 1
    fi
    echo
fi

# ── Stage 2: Symbol extraction ───────────────────────────────────────────────
echo "════════════════════════════════════════════════════════════════"
echo "  Stage 2: Dechirping & extracting symbol values"
echo "════════════════════════════════════════════════════════════════"
python3 "${SCRIPT_DIR}/lora_get_symbol_value_FFT.py" --no-plot --save-symbols lora_symbols.json

if [ ! -f "lora_symbols.json" ]; then
    echo "ERROR: Symbol extraction failed — lora_symbols.json not created."
    exit 1
fi

echo

# ── Stage 3: Recover symbol values (Gray encode bin indices) ──────────────────
echo "════════════════════════════════════════════════════════════════"
echo "  Stage 3: Recovering symbol values (gray_encode of FFT bins)"
echo "════════════════════════════════════════════════════════════════"
python3 "${SCRIPT_DIR}/lora_gray_encode.py" lora_symbols.json -o lora_symbols_gray.json

if [ ! -f "lora_symbols_gray.json" ]; then
    echo "ERROR: Symbol value recovery failed — lora_symbols_gray.json not created."
    exit 1
fi

echo

# ── Stage 4: Deinterleave symbols into codewords ─────────────────────────────
echo "════════════════════════════════════════════════════════════════"
echo "  Stage 4: Deinterleaving symbols → codewords"
echo "════════════════════════════════════════════════════════════════"
python3 "${SCRIPT_DIR}/lora_deinterleave.py" lora_symbols_gray.json -o lora_deinterleaved.json

if [ ! -f "lora_deinterleaved.json" ]; then
    echo "ERROR: Deinterleave failed — lora_deinterleaved.json not created."
    exit 1
fi

echo

# ── Stage 5: (placeholder) Hamming decode header, dewhiten+Hamming payload, bytes, CRC ─
echo "════════════════════════════════════════════════════════════════"
echo "  Stage 5+: Remaining decode (not yet implemented)"
echo "════════════════════════════════════════════════════════════════"
echo "  TODO: HEADER: Hamming decode"
echo "        PAYLOAD: dewhiten → Hamming decode"
echo "        Byte assembly"
echo "        CRC check"
echo
echo "Pipeline complete."