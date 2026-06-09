#!/bin/sh

set -eu

mkdir -p ~/pifm_logs

for i in $(seq -w 1 100); do
    echo "=== run $i ==="
    sudo timeout -s INT 60s ./pi_fm_rds -freq 105.4 -debug -audio pulses.wav \
        > ~/pifm_logs/run_${i}.log 2>&1
    sleep 2
done