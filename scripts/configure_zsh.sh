#!/bin/bash

# Configure ZSH with custom configuration
set -euo pipefail

USERNAME=${1:-ubuntu}
HOME_DIR=${2:-/home/ubuntu}

echo "Configuring ZSH for user: $USERNAME"
echo "Home directory: $HOME_DIR"

# Set the ZDOTDIR variable
export ZDOTDIR="${HOME_DIR}/.config/zsh"
export ZSH_CUSTOM="${ZDOTDIR}/custom"

# Be sure that the ~/.cache folder exists and has the right permissions
if [ ! -d "${HOME_DIR}/.cache" ]; then
    mkdir -p "${HOME_DIR}/.cache"
# if the folder exists, check that the permissions are correct
# if the folder belongs to root, change the ownership. It must be done with sudo
elif [ "$(stat -c %U ${HOME_DIR}/.cache)" = "root" ]; then
    sudo chown -R "$USERNAME":"$USERNAME" "${HOME_DIR}/.cache"
fi

# link fd to fdfind for compatibility
if ! command -v fd >/dev/null 2>&1; then
    # be sure that the ~/.local/bin folder exists
    if [ ! -d "${HOME_DIR}/.local/bin" ]; then
        mkdir -p "${HOME_DIR}/.local/bin"
    fi
    # create the symlink
    ln -s "$(which fdfind)" "${HOME_DIR}/.local/bin/fd"
fi

# Clone repositories using HTTPS (works in Docker build context)
echo "Cloning ZSH configuration..."
git clone --recursive https://github.com/effibot/zdotdir.git "$ZDOTDIR" || true
git clone --recursive https://github.com/effibot/zsh_custom.git "$ZSH_CUSTOM" || true

# Create or update the root .zshenv file to use ZDOTDIR
cat << 'EOF' > "${HOME_DIR}/.zshenv"
export ZDOTDIR=~/.config/zsh
[[ -f $ZDOTDIR/.zshenv ]] && . $ZDOTDIR/.zshenv
EOF

echo "ZSH configuration will be complete at the first start."