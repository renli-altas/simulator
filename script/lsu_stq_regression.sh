#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_DIR="$ROOT_DIR/build/lsu_stq_regression"
SIM="$ROOT_DIR/build/simulator"

mkdir -p "$LOG_DIR"

declare -a CASE_NAMES=()
declare -a CASE_STATUS=()

run_case() {
  local name="$1"
  shift
  local log="$LOG_DIR/${name}.log"
  echo "[RUN] $name"
  if "$@" >"$log" 2>&1; then
    echo "[PASS] $name"
    CASE_NAMES+=("$name")
    CASE_STATUS+=("PASS")
  else
    echo "[FAIL] $name"
    tail -n 80 "$log" || true
    CASE_NAMES+=("$name")
    CASE_STATUS+=("FAIL")
  fi
}

echo "[INFO] Building simulator"
make -C "$ROOT_DIR" -j4 >/dev/null

run_case baremetal_load_store \
  make -C "$ROOT_DIR/baremetal/test" load-store

run_case baremetal_unalign \
  make -C "$ROOT_DIR/baremetal/test" unalign

run_case uart_mmio_test \
  make -C "$ROOT_DIR/baremetal/uart-mmio-test"

run_case vm_pagefault_test \
  make -C "$ROOT_DIR/baremetal/vm-pagefault-test"

run_case rv32ui_ld_st \
  make -C "$ROOT_DIR/baremetal/riscv-tests/isa" rv32ui/ld_st

run_case rv32ui_sb \
  make -C "$ROOT_DIR/baremetal/riscv-tests/isa" rv32ui/sb

run_case rv32ui_sh \
  make -C "$ROOT_DIR/baremetal/riscv-tests/isa" rv32ui/sh

run_case rv32ua_lrsc \
  make -C "$ROOT_DIR/baremetal/riscv-tests/isa" rv32ua/lrsc

echo
echo "== LSU/STQ Regression Summary =="
failed=0
for i in "${!CASE_NAMES[@]}"; do
  printf "  %-24s %s\n" "${CASE_NAMES[$i]}" "${CASE_STATUS[$i]}"
  if [[ "${CASE_STATUS[$i]}" != "PASS" ]]; then
    failed=1
  fi
done

if [[ "$failed" -ne 0 ]]; then
  echo "[RESULT] FAIL"
  exit 1
fi

echo "[RESULT] PASS"
