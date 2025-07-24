#!/bin/bash

# Script to configure integrations between all terminal enhancement tools
# This script should be run after installing all components
# Run as the target user (not root)

set -euo pipefail

echo "Configuring terminal tool integrations..."

# Configure GPG for container environment
echo "Configuring GPG for container environment..."
mkdir -p ~/.gnupg && chmod 700 ~/.gnupg
echo "pinentry-mode loopback" >> ~/.gnupg/gpg.conf
echo "allow-loopback-pinentry" >> ~/.gnupg/gpg-agent.conf
# Reload GPG agent to apply new configuration
gpg-connect-agent reloadagent /bye > /dev/null 2>&1 || true

sed -i 's/#force_color_prompt=yes/force_color_prompt=yes/' "${HOME}"/.bashrc

{
    echo "export GPG_TTY=\$(tty)"
    echo "if [ -z \"\${TMUX:-}\" ] && [ \"\${TERM_PROGRAM:-}\" != \"vscode\" ] && [ -z \"\${SESSION_MANAGER:-}\" ]; then tmux attach -t default || tmux new -s default; fi"
    echo "if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]; then source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash; fi"
    echo "if [ -f /usr/share/colcon_cd/function/colcon_cd.sh ]; then source /usr/share/colcon_cd/function/colcon_cd.sh; export _colcon_cd_root=/opt/ros/\${ROS_DISTRO}; fi"
} >> "${HOME}"/.bashrc

# Configure ble.sh integrations
echo "Configuring ble.sh integrations..."
# Ensure .blerc exists
touch "${HOME}"/.blerc

# Configure ble.sh integrations with fzf and zoxide
{
    echo 'ble-import -d integration/fzf-completion'
    echo 'ble-import -d integration/fzf-key-bindings'
    echo 'ble-import integration/zoxide'
} >> "${HOME}"/.blerc

echo "All integrations configuration complete!"
