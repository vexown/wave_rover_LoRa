#!/usr/bin/env bash
# ══════════════════════════════════════════════════════════════════════════════
#  lora_decoder.sh — End-to-end LoRa decode pipeline
# ══════════════════════════════════════════════════════════════════════════════
#
#  Stage 1: Capture IQ samples from RTL-SDR   → last_capture.npy
#  Stage 2: Dechirp & extract symbol values   → lora_symbols.json
#  Stage 3: Gray decode chip values            → lora_symbols_gray.json
#  Stage 4: (future) Deinterleave, decode FEC, recover payload bytes
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
echo "Symbol data saved to: lora_symbols.json"
echo

# ── Stage 3: Gray decoding ────────────────────────────────────────────────────
echo "════════════════════════════════════════════════════════════════"
echo "  Stage 3: Gray decoding corrected chip values"
echo "════════════════════════════════════════════════════════════════"
python3 "${SCRIPT_DIR}/lora_degray.py" lora_symbols.json -o lora_symbols_gray.json

if [ ! -f "lora_symbols_gray.json" ]; then
    echo "ERROR: Gray decoding failed — lora_symbols_gray.json not created."
    exit 1
fi

echo
echo "Gray-decoded data saved to: lora_symbols_gray.json"
echo

# ── Stage 4: (placeholder) Deinterleave, decode FEC, recover payload bytes ────
echo "════════════════════════════════════════════════════════════════"
echo "  Stage 4: Decode payload (not yet implemented)"
echo "════════════════════════════════════════════════════════════════"
echo "  TODO: deinterleave, header decoding (special case), payload dewhitening, hamming decoding, byte reconstruction"
echo
echo "Pipeline complete."