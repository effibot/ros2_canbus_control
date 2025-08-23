#!/bin/bash

# Script to install zoxide (smarter cd command)
# This script should be run as the target user (not root)

set -euo pipefail

echo "Installing zoxide..."

# Set ZDOTDIR if not already set (adapts to ZSH configuration)
export ZDOTDIR="${ZDOTDIR:-${HOME}/.config/zsh}"

# Install zoxide
curl -sSfL https://raw.githubusercontent.com/ajeetdsouza/zoxide/main/install.sh | sh

# Configure zoxide in bashrc
echo "" >> "${HOME}/.bashrc"
echo "# zoxide setup" >> "${HOME}/.bashrc"
echo "eval \"\$(zoxide init bash --cmd cd)\"" >> "${HOME}/.bashrc"

# Configure zoxide in zshrc (using ZDOTDIR)
#mkdir -p "${ZDOTDIR}"
#echo "" >> "${ZDOTDIR}/.zshrc"
#echo "# zoxide setup" >> "${ZDOTDIR}/.zshrc"
#echo "eval \"\$(zoxide init zsh --cmd cd)\"" >> "${ZDOTDIR}/.zshrc"

echo "zoxide installation complete!"
