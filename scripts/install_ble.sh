#!/bin/bash

# Script to install ble.sh (Bash Line Editor)
# This script should be run as the target user (not root)

set -euo pipefail

echo "Installing ble.sh..."

# Set ZDOTDIR if not already set (adapts to ZSH configuration)
export ZDOTDIR="${ZDOTDIR:-${HOME}/.config/zsh}"

# Clone and install ble.sh
git clone --recursive --depth 1 --shallow-submodules https://github.com/akinomyoga/ble.sh.git "${HOME}"/ble.sh.d
make -j -C "${HOME}"/ble.sh.d install PREFIX="${HOME}"/.local

# Configure ble.sh in bashrc
sed -i "1isource \${HOME}/.local/share/blesh/ble.sh --noattach" "${HOME}"/.bashrc
echo "[[ \${BLE_VERSION-} ]] && ble-attach" >> "${HOME}"/.bashrc

# Create and configure .blerc
touch "${HOME}"/.blerc
echo 'source /etc/profile.d/bash_completion.sh' >> "${HOME}"/.blerc

echo "ble.sh installation complete!"
