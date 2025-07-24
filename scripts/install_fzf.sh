#! /bin/bash

# Script to install fzf (fuzzy finder) for bash
# This script should be run as a regular user, not root
set -euo pipefail
echo "Installing fzf (fuzzy finder) for bash..."
git clone --depth 1 https://github.com/junegunn/fzf.git ${HOME}/.fzf && ${HOME}/.fzf/install 
echo "fzf installation complete!"
# Add fzf to the bash configuration
echo 'eval "$(fzf --bash)"' >> ${HOME}/.bashrc
echo "Done."