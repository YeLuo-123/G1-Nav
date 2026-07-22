#!/usr/bin/env bash
set -euo pipefail

# Deploy workspace to remote robot and optionally build + launch
# Usage: scripts/deploy_to_fangqi.sh <ssh_target> [--remote-ws PATH] [--ros2 DISTRO] [--launch LAUNCH]

SSH_TARGET=""
REMOTE_WS="~/g1nav_ws"
ROS2_DISTRO=""
LAUNCH_FILE=""

print_usage() {
  cat <<EOF
Usage: $0 <ssh_target> [--remote-ws PATH] [--ros2 DISTRO] [--launch LAUNCH]

Examples:
  $0 dev@10.11.32.162 --remote-ws ~/g1nav_ws --ros2 humble --launch bringup_launcher.launch.py

Notes:
  - Ensure you can SSH to the robot (password or key) before running.
  - This script syncs Git-tracked files only, then runs a remote build.
EOF
}

if [ "$#" -lt 1 ]; then
  print_usage
  exit 1
fi

SSH_TARGET="$1"
shift

while [ "$#" -gt 0 ]; do
  case "$1" in
    --remote-ws)
      REMOTE_WS="$2"; shift 2;;
    --ros2)
      ROS2_DISTRO="$2"; shift 2;;
    --launch)
      LAUNCH_FILE="$2"; shift 2;;
    -h|--help)
      print_usage; exit 0;;
    *)
      echo "Unknown arg: $1"; print_usage; exit 1;;
  esac
done

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"

echo "Deploying repository from $REPO_ROOT to $SSH_TARGET:$REMOTE_WS"

# Prepare remote workspace directories
ssh "$SSH_TARGET" "bash -lc 'mkdir -p $REMOTE_WS/src/g1nav'"

# Stream only the committed project tree. Local data, dependencies and build
# products ignored by Git can never be copied accidentally.
git -C "$REPO_ROOT" archive --format=tar HEAD | \
  ssh "$SSH_TARGET" "tar -xf - -C $REMOTE_WS/src/g1nav"

echo "Files copied. Starting remote build (this may take several minutes)..."

# Build on remote: source ROS if ROS2_DISTRO provided, run colcon build
REMOTE_BUILD_CMD=""
if [ -n "$ROS2_DISTRO" ]; then
  REMOTE_BUILD_CMD="source /opt/ros/$ROS2_DISTRO/setup.bash && cd $REMOTE_WS && colcon build --symlink-install"
else
  REMOTE_BUILD_CMD="cd $REMOTE_WS && colcon build --symlink-install"
fi

ssh -t "$SSH_TARGET" "bash -lc '$REMOTE_BUILD_CMD'"

if [ -n "$LAUNCH_FILE" ]; then
  echo "Launching $LAUNCH_FILE on remote device..."
  REMOTE_LAUNCH_CMD=""
  if [ -n \"$ROS2_DISTRO\" ]; then
    REMOTE_LAUNCH_CMD="source /opt/ros/$ROS2_DISTRO/setup.bash && source $REMOTE_WS/install/setup.bash && ros2 launch $LAUNCH_FILE"
  else
    REMOTE_LAUNCH_CMD="source $REMOTE_WS/install/setup.bash && ros2 launch $LAUNCH_FILE"
  fi
  ssh -t "$SSH_TARGET" "bash -lc '$REMOTE_LAUNCH_CMD'"
fi

echo "Deployment finished. Check remote logs or SSH into the robot for runtime debugging."
