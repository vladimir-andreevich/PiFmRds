#!/bin/sh

set -eu

sudo true

taskset -c 3 yes >/dev/null &
load_pid=$!

cleanup() {
    kill "$load_pid" 2>/dev/null || true
    wait "$load_pid" 2>/dev/null || true
}

trap cleanup EXIT INT TERM

sudo timeout -s INT 30s chrt -f 80 taskset -c 2 \
    ./pi_fm_rds -freq 105.4 -audio pulses.wav -debug \
    2> dma_current_ssh_pinned_load.log
