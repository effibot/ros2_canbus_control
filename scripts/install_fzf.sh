#!/bin/bash

# Script to install fzf (fuzzy finder) for bash and zsh
# This script should be run as a regular user, not root
set -euo pipefail

echo "Installing fzf (fuzzy finder)..."

# Set ZDOTDIR if not already set (adapts to ZSH configuration)
export ZDOTDIR="${ZDOTDIR:-${HOME}/.config/zsh}"

# Clone fzf repository
git clone --depth 1 https://github.com/junegunn/fzf.git "${HOME}/.fzf"

# Install fzf with auto-completion and key bindings enabled
"${HOME}/.fzf/install" --all --no-bash --no-zsh

# Add fzf to PATH and configuration in .bashrc
echo "" >> "${HOME}/.bashrc"
echo "# fzf setup" >> "${HOME}/.bashrc"
echo "export PATH=\"\${HOME}/.fzf/bin:\${PATH}\"" >> "${HOME}/.bashrc"
echo "eval \"\$(fzf --bash)\"" >> "${HOME}/.bashrc"

# Add fzf to PATH and configuration in zshrc (using ZDOTDIR)
#mkdir -p "${ZDOTDIR}"
#echo "" >> "${ZDOTDIR}/.zshrc"
#echo "# fzf setup" >> "${ZDOTDIR}/.zshrc"
#echo "export PATH=\"\${HOME}/.fzf/bin:\${PATH}\"" >> "${ZDOTDIR}/.zshrc"
#echo "eval \"\$(fzf --zsh)\"" >> "${ZDOTDIR}/.zshrc"

# Make sure the binary is executable
chmod +x "${HOME}/.fzf/bin/fzf"

echo "fzf installation complete!"
echo "fzf binary location: ${HOME}/.fzf/bin/fzf"