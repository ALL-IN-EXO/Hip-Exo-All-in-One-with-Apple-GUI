#!/bin/bash
# RPi RL controller launcher
# Usage:
#   ./run.sh                # interactive menu to choose nn
#   ./run.sh dnn            # run directly with given nn
#   ./run.sh lstm_leg_dcp
#   ./run.sh lstm_pd
#   ./run.sh pf_imu
#   ./run.sh myoassist_1966080
#   ./run.sh myoassist_2293760

NN_CHOICES=("dnn" "lstm" "lstm_leg_dcp" "lstm_pd" "pf_imu" "myoassist_1966080" "myoassist_2293760")

pick_nn() {
    echo "请选择神经网络类型 (Select neural network type):"
    local i=1
    for nn in "${NN_CHOICES[@]}"; do
        echo "  $i) $nn"
        i=$((i + 1))
    done
    read -rp "输入编号 [1-${#NN_CHOICES[@]}] (默认 1): " choice
    choice=${choice:-1}
    if ! [[ "$choice" =~ ^[0-9]+$ ]] || (( choice < 1 || choice > ${#NN_CHOICES[@]} )); then
        echo "无效选择，使用默认 dnn"
        NN="dnn"
    else
        NN="${NN_CHOICES[$((choice - 1))]}"
    fi
}

if [ -n "$1" ]; then
    NN="$1"
    # validate
    valid=0
    for nn in "${NN_CHOICES[@]}"; do
        if [ "$nn" = "$NN" ]; then valid=1; break; fi
    done
    if [ "$valid" -ne 1 ]; then
        echo "未知 nn 类型: $NN"
        echo "可选: ${NN_CHOICES[*]}"
        exit 1
    fi
else
    pick_nn
fi

echo "启动 RL controller，nn = $NN"
source ~/venvs/pytorch-env/bin/activate
cd "$(dirname "$0")"
python RL_controller_torch.py --nn "$NN"
