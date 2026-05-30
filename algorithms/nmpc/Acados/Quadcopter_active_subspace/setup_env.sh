#!/usr/bin/env bash
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
export ACADOS_SOURCE_DIR="${repo_root}/third_party/acados"
export LD_LIBRARY_PATH="${ACADOS_SOURCE_DIR}/lib:${LD_LIBRARY_PATH}"
