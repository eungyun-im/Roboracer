#!/bin/bash
# 공식 시뮬레이터 빌드 다운로드 -> sim/<이름>/autodrive_simulator
# usage: fetch_sims.sh [practice compete explore icra26 icra25 cdctf25]   (기본: practice compete)
cd "$(dirname "$(readlink -f "$0")")/.."
B=https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Sim-Racing/releases/download
declare -A URL=( [practice]="$B/2026-iros/autodrive_simulator_practice_linux.zip" [compete]="$B/2026-iros/autodrive_simulator_compete_linux.zip"
  [explore]="$B/2026-iros/autodrive_simulator_explore_linux.zip" [icra26]="$B/2026-icra/autodrive_simulator_compete_linux.zip"
  [icra25]="$B/2025-icra/autodrive_simulator_compete_linux.zip" [cdctf25]="$B/2025-cdc-tf/autodrive_simulator_compete_linux.zip" )
declare -A DIR=( [practice]=practice [compete]=c_2026-iros [explore]=explore [icra26]=c_2026-icra [icra25]=c_2025-icra [cdctf25]=c_2025-cdc-tf )
NAMES=${@:-practice compete}
mkdir -p downloads sim
for n in $NAMES; do
  [ -z "${URL[$n]}" ] && { echo "unknown: $n"; continue; }
  d=sim/${DIR[$n]}; [ -d "$d/autodrive_simulator" ] && { echo "$n: 이미 있음 ($d)"; continue; }
  z=downloads/sim_${n}.zip; echo "$n: 다운로드 ${URL[$n]}"
  curl -L --fail -o "$z" "${URL[$n]}" || { echo "  실패"; continue; }
  mkdir -p "$d" && unzip -q -o "$z" -d "$d"
  [ -d "$d/autodrive_simulator" ] || { sub=$(find "$d" -maxdepth 2 -name "AutoDRIVE Simulator.x86_64" | head -1); [ -n "$sub" ] && mv "$(dirname "$sub")" "$d/autodrive_simulator"; }
  chmod +x "$d/autodrive_simulator/AutoDRIVE Simulator.x86_64" 2>/dev/null; echo "  -> $d/autodrive_simulator"
done
