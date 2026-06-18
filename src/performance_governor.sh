#!/bin/sh

set -eu

sudo true

backup_dir="./governor_backup"
mkdir -p "$backup_dir"

restore_governors() {
    for g in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
        cpu="$(basename "$(dirname "$(dirname "$g")")")"
        if [ -f "$backup_dir/${cpu}.txt" ]; then
            sudo sh -c "cat '$backup_dir/${cpu}.txt' > '$g'"
        fi
    done
}

trap restore_governors EXIT INT TERM

for governor in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
    cpu="$(basename "$(dirname "$(dirname "$governor")")")"
    cat "$governor" > "$backup_dir/${cpu}.txt"
    echo performance | sudo tee "$governor" >/dev/null
done

sudo timeout -s INT 60s ./pi_fm_rds -freq 105.4 -audio pulses.wav -debug \
    2> test_performance_governor.log
