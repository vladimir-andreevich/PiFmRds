mkdir -p ~/pifm_logs

for i in $(seq -w 1 100); do
    echo "=== run $i ==="
    sudo timeout -s INT 9s ./pi_fm_rds -freq 103.0 -debug -audio sound.wav \
        > ~/pifm_logs/run_${i}.log 2>&1
    sleep 0.5
done