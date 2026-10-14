#!/usr/bin/env bash
# MoveIt 2 一键安装（官方二进制）。
#
# 按 Ubuntu 版本选择 ROS 2 发行版并安装 MoveIt 2、RViz、Panda 示例和
# ros2_control（示例用 mock 硬件执行轨迹）。中间件使用官方推荐的 CycloneDDS。
#
# 用法:
#   ./scripts/install_moveit.sh
#   ./scripts/install_moveit.sh --distro jazzy
#   ROS_DISTRO=humble ./scripts/install_moveit.sh
#
# 对应关系:
#   Ubuntu 22.04 → humble
#   Ubuntu 24.04 → jazzy（也可 --distro kilted）
#   Ubuntu 26.04 → lyrical（也可 --distro rolling）

set -euo pipefail

BEGIN_MARK="# >>> moveit2 >>>"
END_MARK="# <<< moveit2 <<<"

log() {
  printf '\n==> %s\n' "$*"
}

die() {
  printf '错误: %s\n' "$*" >&2
  exit 1
}

retry() {
  local attempt=1
  local max=3
  until "$@"; do
    if [[ "$attempt" -ge "$max" ]]; then
      return 1
    fi
    printf '命令失败，%s/%s 次后重试...\n' "$attempt" "$max" >&2
    sleep $((attempt * 2))
    attempt=$((attempt + 1))
  done
}

usage() {
  cat <<'EOF'
用法: scripts/install_moveit.sh [--distro <name>]

安装 ROS 2 与 MoveIt 2 官方二进制包。未指定发行版时按 Ubuntu 版本选择:
  22.04 → humble
  24.04 → jazzy
  26.04 → lyrical

24.04 可额外指定 kilted，26.04 可额外指定 rolling。
环境变量 ROS_DISTRO 与 --distro 作用相同，命令行优先。
EOF
}

load_os_release() {
  [[ -r /etc/os-release ]] || die "找不到 /etc/os-release"
  # shellcheck disable=SC1091
  . /etc/os-release
  [[ "${ID:-}" == "ubuntu" ]] || die "此脚本只支持 Ubuntu，当前系统是 ${ID:-unknown}"
  [[ -n "${VERSION_ID:-}" ]] || die "无法读取 Ubuntu 版本"
  UBUNTU_VERSION="$VERSION_ID"
  UBUNTU_CODENAME="${UBUNTU_CODENAME:-${VERSION_CODENAME:-}}"
  [[ -n "$UBUNTU_CODENAME" ]] || die "无法读取 Ubuntu 代号"
}

default_distro() {
  case "$1" in
    22.04) printf 'humble\n' ;;
    24.04) printf 'jazzy\n' ;;
    26.04) printf 'lyrical\n' ;;
    *) die "Ubuntu $1 没有对应的 MoveIt 2 二进制包。支持 22.04、24.04、26.04" ;;
  esac
}

distro_matches_ubuntu() {
  case "$1:$2" in
    22.04:humble | 24.04:jazzy | 24.04:kilted | 26.04:lyrical | 26.04:rolling) return 0 ;;
    *) return 1 ;;
  esac
}

apt_get() {
  sudo DEBIAN_FRONTEND=noninteractive apt-get \
    -o Dpkg::Options::=--force-confdef \
    -o Dpkg::Options::=--force-confold \
    "$@"
}

ensure_utf8() {
  if locale | grep -qi 'utf-8'; then
    return 0
  fi
  log "生成 en_US.UTF-8"
  apt_get update
  apt_get install -y locales
  sudo locale-gen en_US en_US.UTF-8
  sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
  export LANG=en_US.UTF-8
  export LC_ALL=en_US.UTF-8
}

ensure_ubuntu_suites() {
  local sources=/etc/apt/sources.list.d/ubuntu.sources
  local codename="$1"
  [[ -f "$sources" ]] || return 0
  if grep -Eq "^Suites:.*[[:space:]]${codename}-updates([[:space:]]|$)" "$sources" \
    && grep -Eq "^Suites:.*[[:space:]]${codename}-backports([[:space:]]|$)" "$sources"; then
    return 0
  fi
  log "补全 Ubuntu 软件源的 updates 与 backports"
  sudo python3 - "$sources" "$codename" <<'PY'
import sys
path, codename = sys.argv[1], sys.argv[2]
lines = open(path, encoding="utf-8").read().splitlines(keepends=True)
changed = False
out = []
for line in lines:
    body = line.strip()
    if body.startswith("Suites:") and f"{codename}-security" not in body.split():
        suites = body.split()[1:]
        if codename in suites:
            for extra in (f"{codename}-updates", f"{codename}-backports"):
                if extra not in suites:
                    suites.append(extra)
                    changed = True
            line = "Suites: " + " ".join(suites) + "\n"
    out.append(line)
if changed:
    open(path, "w", encoding="utf-8").writelines(out)
PY
  sudo apt-get clean
}

ros_apt_source_version() {
  local version
  version=$(curl -fsSL "https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest" \
    | awk -F'"' '/tag_name/ { print $4; exit }') || true
  if [[ -z "$version" ]]; then
    version=$(curl -fsSL -o /dev/null -w '%{url_effective}' \
      "https://github.com/ros-infrastructure/ros-apt-source/releases/latest" \
      | awk -F/ '{ print $NF }')
  fi
  [[ -n "$version" && "$version" != "latest" ]] || die "无法获取 ros2-apt-source 版本"
  printf '%s\n' "$version"
}

setup_ros_apt() {
  local version deb
  version=$(ros_apt_source_version)
  deb="/tmp/ros2-apt-source_${version}.${UBUNTU_CODENAME}_all.deb"
  log "配置 ROS 2 软件源（ros2-apt-source ${version}）"
  retry curl -fL -o "$deb" \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${version}/ros2-apt-source_${version}.${UBUNTU_CODENAME}_all.deb"
  sudo dpkg -i "$deb"
  rm -f "$deb"
}

require_packages() {
  local missing=()
  local pkg
  for pkg in "$@"; do
    if ! apt-cache show "$pkg" >/dev/null 2>&1; then
      missing+=("$pkg")
    fi
  done
  if [[ "${#missing[@]}" -gt 0 ]]; then
    die "软件源里没有这些包: ${missing[*]}"
  fi
}

upsert_bashrc() {
  local bashrc="$HOME/.bashrc"
  local distro="$1"
  local tmp
  touch "$bashrc"
  tmp=$(mktemp)
  if grep -Fxq "$BEGIN_MARK" "$bashrc"; then
    awk -v begin="$BEGIN_MARK" -v end="$END_MARK" -v distro="$distro" '
      $0 == begin {
        print begin
        print "source /opt/ros/" distro "/setup.bash"
        print "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
        print end
        skip = 1
        next
      }
      skip && $0 == end { skip = 0; next }
      !skip { print }
    ' "$bashrc" >"$tmp"
  else
    cat "$bashrc" >"$tmp"
    printf '\n%s\nsource /opt/ros/%s/setup.bash\nexport RMW_IMPLEMENTATION=rmw_cyclonedds_cpp\n%s\n' \
      "$BEGIN_MARK" "$distro" "$END_MARK" >>"$tmp"
  fi
  mv "$tmp" "$bashrc"
}

verify_install() {
  local distro="$1"
  bash -c "
    set -euo pipefail
    set +u
    source '/opt/ros/${distro}/setup.bash'
    set -u
    for pkg in moveit_ros_move_group moveit_setup_assistant moveit_resources_panda_moveit_config controller_manager rmw_cyclonedds_cpp; do
      ros2 pkg prefix \"\$pkg\" >/dev/null
      printf '已安装 %s\n' \"\$pkg\"
    done
  "
}

main() {
  local requested="${ROS_DISTRO:-}"
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --distro)
        [[ $# -ge 2 ]] || die "--distro 需要发行版名称"
        requested="$2"
        shift 2
        ;;
      -h | --help)
        usage
        exit 0
        ;;
      *)
        die "未知参数: $1（可用 --help 查看用法）"
        ;;
    esac
  done

  [[ "$(id -u)" -ne 0 ]] || die "请用普通用户运行，脚本会在需要时调用 sudo"
  command -v sudo >/dev/null 2>&1 || die "需要 sudo"

  load_os_release
  local distro
  if [[ -n "$requested" ]]; then
    distro="$requested"
    distro_matches_ubuntu "$UBUNTU_VERSION" "$distro" \
      || die "Ubuntu ${UBUNTU_VERSION} 不能安装 ROS 2 ${distro}"
  else
    distro=$(default_distro "$UBUNTU_VERSION")
  fi

  if ! sudo -n true 2>/dev/null; then
    sudo -v
  fi

  log "Ubuntu ${UBUNTU_VERSION}，安装 ROS 2 ${distro} 与 MoveIt 2"
  ensure_utf8
  ensure_ubuntu_suites "$UBUNTU_CODENAME"
  setup_ros_apt

  log "更新软件包索引"
  retry apt_get update

  local packages=(
    ros-dev-tools
    "ros-${distro}-desktop"
    "ros-${distro}-moveit"
    "ros-${distro}-moveit-resources"
    "ros-${distro}-rmw-cyclonedds-cpp"
    "ros-${distro}-ros2-control"
    "ros-${distro}-ros2-controllers"
  )
  require_packages "${packages[@]}"

  log "升级已安装的软件包"
  apt_get upgrade -y

  log "安装 ROS 2 ${distro}、MoveIt 2、ros2_control 与 CycloneDDS"
  apt_get install -y "${packages[@]}"

  log "初始化 rosdep"
  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    sudo rosdep init
  fi
  retry rosdep update

  log "把 ROS 2 与 CycloneDDS 写入 ${HOME}/.bashrc"
  upsert_bashrc "$distro"

  log "校验 MoveIt 2 包"
  verify_install "$distro"

  cat <<EOF

安装完成。新开一个终端，或执行:
  source ~/.bashrc

启动 Panda MoveIt 示例:
  ros2 launch moveit_resources_panda_moveit_config demo.launch.py
EOF
}

main "$@"
