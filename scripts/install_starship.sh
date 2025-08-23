#!/bin/bash

# Script to install starship (modern prompt)
# This script should be run as the target user (not root)

set -euo pipefail

echo "Installing starship..."

# Set ZDOTDIR if not already set (adapts to ZSH configuration)
export ZDOTDIR="${ZDOTDIR:-${HOME}/.config/zsh}"

# Install starship
curl -sS https://starship.rs/install.sh | sh -s -- -y

# Configure starship in bashrc
echo "" >> "${HOME}/.bashrc"
echo "# starship setup" >> "${HOME}/.bashrc"
echo "eval \"\$(starship init bash)\"" >> "${HOME}/.bashrc"

# Configure starship in zshrc (using ZDOTDIR)
#mkdir -p "${ZDOTDIR}"
#echo "" >> "${ZDOTDIR}/.zshrc"
#echo "# starship setup" >> "${ZDOTDIR}/.zshrc"
#echo "eval \"\$(starship init zsh)\"" >> "${ZDOTDIR}/.zshrc"

# Create configuration directory and file
mkdir -p "${HOME}/.config"
touch "${HOME}/.config/starship.toml"

# Configure starship theme
starship preset gruvbox-rainbow >> "${HOME}/.config/starship.toml"

echo "starship installation complete!"
