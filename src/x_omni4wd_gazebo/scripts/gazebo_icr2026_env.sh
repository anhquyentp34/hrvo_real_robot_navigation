#!/usr/bin/env bash
# Chạy Gazebo với ICR2026_env.world hoặc ICR2026_env2.world dùng cùng
# GAZEBO_MODEL_PATH / RESOURCE / PLUGIN như roslaunch
# x_omni4wd_simulation ICR2026_simulation.launch — để picture (model://…)
# và actor skin/plugin khớp vị trí & mesh với bản mô phỏng ROS.
#
# Cách dùng: source ~/…/devel/setup.bash  rồi:
#   rosrun x_omni4wd_gazebo gazebo_icr2026_env.sh
#   rosrun x_omni4wd_gazebo gazebo_icr2026_env.sh ICR2026_env2.world
# hoặc truyền world tuyệt đối + đối số cho gazebo:
#   …/gazebo_icr2026_env.sh /abs/path/to/custom.world --verbose

set -euo pipefail

# Khai báo mặc định để /usr/share/gazebo/setup.sh không lỗi khi shell đang bật `set -u`.
: "${GAZEBO_MODEL_PATH:=}"
: "${GAZEBO_RESOURCE_PATH:=}"
: "${GAZEBO_PLUGIN_PATH:=}"

# Nạp biến mặc định của Gazebo (media/shaders/models hệ thống) nếu có.
if [[ -f "/usr/share/gazebo/setup.sh" ]]; then
  # shellcheck source=/usr/share/gazebo/setup.sh
  source "/usr/share/gazebo/setup.sh"
elif [[ -f "/usr/share/gazebo-11/setup.sh" ]]; then
  # shellcheck source=/usr/share/gazebo-11/setup.sh
  source "/usr/share/gazebo-11/setup.sh"
fi

if ! command -v rospack >/dev/null 2>&1; then
  echo "Chưa có rospack: hãy source workspace (devel/setup.bash)." >&2
  exit 1
fi

OMNI="$(rospack find x_omni4wd_gazebo)"
ACTOR_SKIN="$(rospack find gazebo_ros_actor_plugin)"
COLL="$(rospack find actor_collisions_plugin)"

# Khớp ICR2026_simulation.launch (env)
export GAZEBO_MODEL_PATH="${OMNI}/models:${ACTOR_SKIN}/config/skins:${GAZEBO_MODEL_PATH:-}"
export GAZEBO_RESOURCE_PATH="${OMNI}/worlds:${GAZEBO_RESOURCE_PATH:-}"
export GAZEBO_PLUGIN_PATH="${GAZEBO_PLUGIN_PATH:-}:${ACTOR_SKIN}/../../devel/lib:${COLL}/../../devel/lib"

WORLD_ARG="${1:-ICR2026_env.world}"
if [[ "${WORLD_ARG}" == *.world ]] && [[ "${WORLD_ARG}" != /* ]]; then
  WORLD="${OMNI}/worlds/${WORLD_ARG}"
else
  WORLD="${WORLD_ARG}"
fi

if [[ ! -f "${WORLD}" ]]; then
  echo "Không tìm thấy world: ${WORLD}" >&2
  exit 1
fi

if [[ $# -gt 0 ]]; then
  shift
fi

exec gazebo "${WORLD}" "$@"
