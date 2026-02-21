#!/usr/bin/env bash
set -e

FREQ=869725000
RATE=2400000
SECONDS=10
SAMPLES=$((RATE * SECONDS))

OUTPUT_IQ="last_capture.iq"
OUTPUT_NPY="last_capture.npy"

echo "Recording ${SECONDS}s (${SAMPLES} samples) at ${FREQ} Hz → ${OUTPUT_IQ}"

# -n ensures rtl_sdr exits cleanly after SAMPLES
rtl_sdr -f ${FREQ} -s ${RATE} -g 0 -n ${SAMPLES} "${OUTPUT_IQ}"

echo "Capture finished. Converting ${OUTPUT_IQ} → ${OUTPUT_NPY} ..."

python3 << EOF
import numpy as np

iq_file = "${OUTPUT_IQ}"
out_file = "${OUTPUT_NPY}"

print(f"Loading {iq_file} ...")
raw = np.fromfile(iq_file, dtype=np.uint8)

# RTL-SDR IQ format: unsigned 8-bit interleaved I/Q
iq = (raw.astype(np.float32) - 127.5) / 127.5
iq = iq[0::2] + 1j * iq[1::2]

print(f"Saving {out_file} ({len(iq)} complex samples)")
np.save(out_file, iq)
EOF

echo
echo "Capture and conversion complete."
echo "Next step:"
echo "  python3 lora_symbol_observer.py"
echo
echo "Ensure MODE = 'file' and IQ_FILE = '${OUTPUT_NPY}' in the script."