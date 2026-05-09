#!/usr/bin/env bash
# =============================================================================
# install/lib/udev.sh coverage
# =============================================================================

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck source=lib/framework.sh
source "$SCRIPT_DIR/lib/framework.sh"
# shellcheck source=lib/mocks.sh
source "$SCRIPT_DIR/lib/mocks.sh"

setup_sandbox
install_all_mocks

section "install_udev_rules works with set -u"

repo="$SANDBOX/repo"
sandbox_repo "$repo"

# shellcheck source=/dev/null
source "$repo/install/lib/common.sh"
# shellcheck source=/dev/null
source "$repo/install/lib/i18n.sh"
# shellcheck source=/dev/null
source "$repo/install/lib/config.sh"
# shellcheck source=/dev/null
source "$repo/install/lib/platform.sh"
# shellcheck source=/dev/null
source "$repo/install/lib/docker.sh"
# shellcheck source=/dev/null
source "$repo/install/lib/backend_choice.sh"
# shellcheck source=/dev/null
source "$repo/install/lib/range.sh"
# shellcheck source=/dev/null
source "$repo/install/lib/env.sh"
# shellcheck source=/dev/null
source "$repo/install/lib/udev.sh"

load_locale
reapply_test_assertions

export UDEV_RULES_FILE="$SANDBOX/99-mowgli.rules"
export SUDO=""

export HARDWARE_BACKEND="mowgli"
export GNSS_BACKEND="unicore"
unset GPS_CONNECTION GPS_PROTOCOL GPS_PORT GPS_BY_ID GPS_UART_DEVICE GPS_BAUD 2>/dev/null || true
export LIDAR_ENABLED="false"
export TFLUNA_FRONT_ENABLED="false"
export TFLUNA_EDGE_ENABLED="false"

mkdir -p "$SANDBOX/dev"
mkdir -p "$SANDBOX/serial-by-id"
touch "$SANDBOX/dev/ttyACM0"
ln -s "$SANDBOX/dev/ttyACM0" "$SANDBOX/serial-by-id/usb-Mowgli_STM32-if00"
export SERIAL_BY_ID_DIR="$SANDBOX/serial-by-id"

# Reproduce the web composer Unicore path:
# --backend=mowgli --gnss=unicore --lidar=ldlidar-uart --tfluna=none
# No --gps flag is emitted, so setup_env fills the runtime GPS defaults
# before udev verification runs.
setup_env >/dev/null 2>&1
touch "$GPS_UART_DEVICE"

set -u
if output="$(install_udev_rules 2>&1)"; then
  pass "install_udev_rules handles unicore web preset under set -u"
else
  fail "install_udev_rules handles unicore web preset under set -u" "$output"
fi
set +u

assert_file_exists "udev rules file written" "$UDEV_RULES_FILE"
rules="$(cat "$UDEV_RULES_FILE")"
assert_contains "Mowgli by-id rule generated" 'SYMLINK+="mowgli"' "$rules"
assert_contains "Mowgli by-id rule uses resolved kernel" 'KERNEL=="ttyACM0"' "$rules"
assert_contains "gps uart rule generated" 'SYMLINK+="gps"' "$rules"
assert_eq "GNSS backend remains unicore" "unicore" "$GNSS_BACKEND"

test_summary
