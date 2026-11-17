#!/usr/bin/env bash
# build_curobo.sh — cuRobo v0.7.7 을 Isaac Sim 4.5 내장 Python 에 editable 설치한다.
#
#   사용법:
#     build_curobo.sh --check     : 툴체인 확인만 (nvcc / gcc-11 / Isaac torch CUDA·ABI). 아무것도 설치하지 않음
#     build_curobo.sh --dry-run   : 위 확인 + pip install --dry-run (설치될 패키지 목록만 출력)
#     build_curobo.sh --install   : 위 확인 + 실제 설치 (cuRobo 문서의 Isaac Sim 방식)
#   선택 환경 변수:
#     WARP_PIN=1.5.0 (또는 "warp-lang==1.5.0") : warp-lang 버전을 고정해서 설치 (비우면 pip 기본 선택)
#     CONSTRAINTS=<파일>                         : pip -c 제약 파일 (예: env_check/step2/isaac_pin_constraints.txt)
#
#   원칙: 환경 변수는 이 스크립트 안에서만 지정한다. ~/.bashrc, 시스템 CUDA, ~/.local 은 건드리지 않는다.
#   설치 대상: ~/effmass_mpc/isaacsim/kit/python/lib/python3.10/site-packages (Isaac 내장 Python)
set -uo pipefail

MODE="${1:---check}"
ROOT="$HOME/effmass_mpc"
ISAAC="$ROOT/isaacsim"
CUDA118="$ROOT/cuda-11.8"
CUROBO="$ROOT/curobo"
OUT="$ROOT/env_check/step2"
STAMP="$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUT"

[[ -x "$CUDA118/bin/nvcc" ]] || { echo "[build_curobo] $CUDA118/bin/nvcc 가 없습니다. CUDA 11.8 로컬 설치를 먼저 하세요." >&2; exit 2; }
[[ -x "$ISAAC/python.sh" ]] || { echo "[build_curobo] $ISAAC/python.sh 가 없습니다." >&2; exit 2; }
[[ -d "$CUROBO/.git" ]]     || { echo "[build_curobo] $CUROBO 가 없습니다." >&2; exit 2; }

# ---- 깨끗한 환경 (run_isaac.sh 와 같은 원칙 + CUDA 11.8 빌드 변수) ----
CLEAN_ENV=(
  env -i
  HOME="$HOME" USER="${USER:-$(id -un)}" LOGNAME="${LOGNAME:-$(id -un)}"
  SHELL=/bin/bash TERM="${TERM:-xterm-256color}" LANG="${LANG:-C.UTF-8}"
  PYTHONNOUSERSITE=1
  PYTHONUNBUFFERED=1
  CUDA_HOME="$CUDA118"
  PATH="$CUDA118/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
  LD_LIBRARY_PATH="$CUDA118/lib64"
  CC=/usr/bin/gcc-11
  CXX=/usr/bin/g++-11
  TORCH_CUDA_ARCH_LIST="7.5"
)
PY=("${CLEAN_ENV[@]}" "$ISAAC/python.sh")

echo "================ [build_curobo] 툴체인 확인 ($STAMP) ================"
echo "--- 빌드 환경 변수"
"${CLEAN_ENV[@]}" bash -c 'for v in CUDA_HOME PATH LD_LIBRARY_PATH CC CXX TORCH_CUDA_ARCH_LIST PYTHONNOUSERSITE PYTHONPATH ROS_DISTRO AMENT_PREFIX_PATH; do echo "$v=${!v-<unset>}"; done'
echo "--- which nvcc / nvcc --version"
"${CLEAN_ENV[@]}" bash -c 'which nvcc; nvcc --version | tail -2'
echo "--- gcc-11 / g++-11"
/usr/bin/gcc-11 --version | head -1; /usr/bin/g++-11 --version | head -1
echo "--- Isaac 내장 torch"
"${PY[@]}" -c "
import torch, sys
from torch.utils import cpp_extension as ce
print('python          :', sys.version.split()[0], sys.executable)
print('torch           :', torch.__version__, torch.__file__)
print('torch.version.cuda:', torch.version.cuda)
print('CXX11_ABI       :', torch._C._GLIBCXX_USE_CXX11_ABI)
print('cpp_extension CUDA_HOME:', ce.CUDA_HOME)
print('arch flags      :', ce._get_cuda_arch_flags())
" 2>&1 | grep -v "^$"
NVCC_VER="$("${CLEAN_ENV[@]}" nvcc --version | sed -n 's/.*release \([0-9.]*\),.*/\1/p')"
TORCH_CUDA="$("${PY[@]}" -c 'import torch; print(torch.version.cuda)' 2>/dev/null | tail -1)"
echo "--- 일치 여부: nvcc=$NVCC_VER  torch.version.cuda=$TORCH_CUDA"
if [[ "$NVCC_VER" != "$TORCH_CUDA" ]]; then
  echo "[build_curobo] 불일치 — 빌드하지 않고 종료" >&2; exit 3
fi
echo "일치"

[[ "$MODE" == "--check" ]] && exit 0

cd "$CUROBO"
echo "--- cuRobo commit: $(git rev-parse HEAD)  ($(git describe --tags 2>/dev/null))"
EXTRA=()   # cuRobo 설치 줄에 붙는 인자 (warp 고정 + 제약)
CARGS=()   # 모든 pip 설치 줄에 붙는 제약 인자
if [[ -n "${WARP_PIN:-}" ]]; then
  [[ "$WARP_PIN" == *"=="* ]] || WARP_PIN="warp-lang==$WARP_PIN"   # WARP_PIN=1.5.0 형식도 허용
  EXTRA+=("$WARP_PIN"); echo "--- warp 고정: $WARP_PIN"
fi

if [[ -n "${CONSTRAINTS:-}" ]]; then
  [[ -f "$CONSTRAINTS" ]] || { echo "[build_curobo] 제약 파일 없음: $CONSTRAINTS" >&2; exit 5; }
  CARGS+=(-c "$CONSTRAINTS"); EXTRA+=(-c "$CONSTRAINTS"); echo "--- 제약 파일: $CONSTRAINTS"; grep -v '^#' "$CONSTRAINTS" | sed 's/^/      /'
fi

if [[ "$MODE" == "--dry-run" ]]; then
  echo "================ pip install --dry-run ================"
  "${PY[@]}" -m pip install --dry-run tomli wheel ninja "${CARGS[@]}" 2>&1 | grep -E "^Would install|ERROR" 
  "${PY[@]}" -m pip install --dry-run -e ".[isaacsim]" --no-build-isolation "${EXTRA[@]}" 2>&1 | grep -vE "^\s*$" | tail -15
  exit 0
fi

if [[ "$MODE" == "--install" ]]; then
  "${PY[@]}" -m pip list --format=freeze 2>/dev/null > "$OUT/pip_before_install_$STAMP.txt"
  echo "================ 1) pip install tomli wheel ninja (cuRobo 문서 단계) ================"
  "${PY[@]}" -m pip install tomli wheel ninja "${CARGS[@]}" || exit 4
  echo "================ 2) pip install -e .[isaacsim] --no-build-isolation ================"
  T0=$(date +%s)
  "${PY[@]}" -m pip install -v -e ".[isaacsim]" --no-build-isolation "${EXTRA[@]}"
  RC=$?
  echo "--- pip 종료 코드 $RC, 소요 $(( $(date +%s) - T0 )) s"
  "${PY[@]}" -m pip list --format=freeze 2>/dev/null > "$OUT/pip_after_install_$STAMP.txt"
  echo "--- pip list 차이 (< 설치 전, > 설치 후)"
  diff "$OUT/pip_before_install_$STAMP.txt" "$OUT/pip_after_install_$STAMP.txt"

  # ---- chardet 정리 (2026-11-15 사용자 결정, step2_report.md 2.13~2.14절) ----
  # trimesh[easy] 가 끌어오는 chardet(7.x)은 Isaac 내장 requests 2.32.3 의 지원 범위(<6) 밖이라
  # 모든 Isaac 실행에서 RequestsDependencyWarning 을 내고, requests 의 문자 인코딩 판별을
  # charset_normalizer 3.3.2 → chardet 으로 바꾼다. cuRobo 는 chardet 을 쓰지 않는다.
  # 설치 전에는 없었고 이번 설치로 새로 생긴 경우에만 제거하고 기록한다.
  if ! grep -qiE '^chardet==' "$OUT/pip_before_install_$STAMP.txt" && grep -qiE '^chardet==' "$OUT/pip_after_install_$STAMP.txt"; then
    CHV="$(grep -iE '^chardet==' "$OUT/pip_after_install_$STAMP.txt")"
    echo "================ 3) 새로 설치된 $CHV 제거 ================"
    "${PY[@]}" -m pip uninstall -y chardet
    "${PY[@]}" -m pip list --format=freeze 2>/dev/null > "$OUT/pip_after_chardet_removal_$STAMP.txt"
    echo "$(date '+%F %T') removed $CHV (newly installed by cuRobo deps); diff below" >> "$OUT/chardet_removal_record.txt"
    diff "$OUT/pip_after_install_$STAMP.txt" "$OUT/pip_after_chardet_removal_$STAMP.txt" | tee -a "$OUT/chardet_removal_record.txt"
  else
    echo "--- chardet: 새로 설치되지 않았거나 설치 전부터 있었음 → 손대지 않음"
  fi
  exit $RC
fi

echo "알 수 없는 모드: $MODE" >&2; exit 1
