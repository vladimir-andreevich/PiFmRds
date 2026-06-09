#!/bin/sh

set -eu

sudo true

sudo timeout -s INT 60s taskset -c 2 \
    ./pi_fm_rds -freq 105.4 -audio pulses.wav -debug \
    2> test_taskset_only.log
