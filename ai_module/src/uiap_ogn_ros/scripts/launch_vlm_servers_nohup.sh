#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_UIAP_ROOT="$(cd "${SCRIPT_DIR}/../../../../uiap-ogn" 2>/dev/null || pwd)"
UIAP_ROOT="${UIAP_ROOT:-${DEFAULT_UIAP_ROOT}}"

if [ -z "${VLFM_PYTHON}" ]; then
  if command -v conda >/dev/null 2>&1 && conda env list | grep -q "uiap-ogn"; then
    # shellcheck disable=SC1091
    source "$(conda info --base)/etc/profile.d/conda.sh"
    conda activate uiap-ogn
    export VLFM_PYTHON="$(which python)"
  else
    export VLFM_PYTHON="$(which python)"
  fi
fi

export MOBILE_SAM_CHECKPOINT="${MOBILE_SAM_CHECKPOINT:-${UIAP_ROOT}/data/mobile_sam.pt}"
export GROUNDING_DINO_CONFIG="${GROUNDING_DINO_CONFIG:-${UIAP_ROOT}/GroundingDINO/groundingdino/config/GroundingDINO_SwinT_OGC.py}"
export GROUNDING_DINO_WEIGHTS="${GROUNDING_DINO_WEIGHTS:-${UIAP_ROOT}/data/groundingdino_swint_ogc.pth}"
export CLASSES_PATH="${CLASSES_PATH:-${UIAP_ROOT}/vlfm/vlm/classes.txt}"
export GROUNDING_DINO_PORT="${GROUNDING_DINO_PORT:-12181}"
export BLIP2ITM_PORT="${BLIP2ITM_PORT:-12182}"
export SAM_PORT="${SAM_PORT:-12183}"
export YOLOV7_PORT="${YOLOV7_PORT:-12184}"

LOG_DIR="${LOG_DIR:-/tmp/uiap_ogn_vlm}"
mkdir -p "${LOG_DIR}"

echo "[uiap_ogn_ros] Starting VLM servers from ${UIAP_ROOT}"

function start_server() {
  local name=$1
  local module=$2
  local port=$3
  local log="${LOG_DIR}/${name}.log"
  nohup "${VLFM_PYTHON}" -m "${module}" --port "${port}" >"${log}" 2>&1 &
  local pid=$!
  echo "  - ${name} (pid ${pid}) -> ${log}"
}

start_server grounding_dino vlfm.vlm.grounding_dino "${GROUNDING_DINO_PORT}"
start_server blip2_itm vlfm.vlm.blip2itm "${BLIP2ITM_PORT}"
start_server mobile_sam vlfm.vlm.sam "${SAM_PORT}"
start_server yolov7 vlfm.vlm.yolov7 "${YOLOV7_PORT}"

echo "[uiap_ogn_ros] Servers launched. Logs at ${LOG_DIR}"
