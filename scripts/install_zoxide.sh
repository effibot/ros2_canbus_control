#!/bin/bash

# Script to install zoxide (smarter cd command)
# This script should be run as the target user (not root)

set -euo pipefail

echo "Installing zoxide..."

# Install zoxide
curl -sSfL https://raw.githubusercontent.com/ajeetdsouza/zoxide/main/install.sh | sh

# Configure zoxide in bashrc
echo "eval \"\$(zoxide init bash --cmd cd)\"" >> "${HOME}"/.bashrc

echo "zoxide installation complete!"
