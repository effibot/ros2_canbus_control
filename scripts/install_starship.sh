#!/bin/bash

# Script to install starship (modern prompt)
# This script should be run as the target user (not root)

set -euo pipefail

echo "Installing starship..."

# Install starship
curl -sS https://starship.rs/install.sh | sh -s -- -y -b "${HOME}"/.local/bin

# Configure starship in bashrc
echo "eval \"\$(starship init bash)\"" >> "${HOME}"/.bashrc

# Create configuration directory and file
mkdir -p "${HOME}"/.config
touch "${HOME}"/.config/starship.toml

# Configure starship theme
starship preset gruvbox-rainbow >> "${HOME}"/.config/starship.toml

echo "starship installation complete!"
