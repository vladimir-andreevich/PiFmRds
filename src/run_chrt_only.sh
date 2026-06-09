#!/bin/sh

set -eu

sudo true

sudo timeout -s INT 60s chrt -f 80 \
    ./pi_fm_rds -freq 105.4 -audio pulses.wav -debug \
    2> test_chrt_only.log
