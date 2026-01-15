#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ROOT_DIR=$(cd -- "${SCRIPT_DIR}" && pwd)
OJ_TOOL=${OJ_TOOL:-oj}
CXX_LIST=${CXX_LIST:-"g++ clang++"}
CXXFLAGS_DEFAULT=${CXXFLAGS:-"-std=c++20 -O2 -Wall -Wextra"}

if ! command -v "${OJ_TOOL}" >/dev/null 2>&1; then
  echo "error: online-judge-tools (oj) is not installed" >&2
  echo "install via: pip install -U online-judge-tools" >&2
  exit 1
fi

run_one() {
  local test_file=$1
  local url cache_dir compiler out bin
  url=$(grep -o '^# *define\s\+PROBLEM\s\+\(https\?://[^\s"]\+\)' "${test_file}" | sed 's/.*PROBLEM\s\+//')
  cache_dir="${ROOT_DIR}/test/$(echo -n "${url:-${test_file}}" | md5sum | cut -d' ' -f1)"
  mkdir -p "${cache_dir}"

  for compiler in ${CXX_LIST}; do
    if ! command -v "${compiler}" >/dev/null 2>&1; then
      echo "skip: ${compiler} not found" >&2
      continue
    fi
    out="${cache_dir}/${compiler}.out"
    echo "[compile] ${compiler} ${test_file}"
    "${compiler}" ${CXXFLAGS_DEFAULT} -I "${ROOT_DIR}" -o "${out}" "${test_file}" >/dev/null
    if [[ -n "${url}" ]]; then
      if [[ ! -d "${cache_dir}/test" ]]; then
        echo "[download] ${url}"
        "${OJ_TOOL}" download --system "${url}" -d "${cache_dir}/test"
      fi
      echo "[run] ${compiler} ${test_file}"
      "${OJ_TOOL}" test --tle 10 --c "${out}" -d "${cache_dir}/test"
    else
      echo "[run] ${compiler} ${test_file} (no PROBLEM directive)"
      "${out}"
    fi
  done
}

if [[ $# -eq 0 ]]; then
  mapfile -t targets < <(find "${ROOT_DIR}" -name '*.test.cpp' -print | sort)
else
  mapfile -t targets < <(printf '%s\n' "$@")
fi

if [[ ${#targets[@]} -eq 0 ]]; then
  echo "no .test.cpp files found" >&2
  exit 0
fi

for file in "${targets[@]}"; do
  run_one "${file}"
  echo
done
