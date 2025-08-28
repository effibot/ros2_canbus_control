#! /bin/bash

# be sure that wget has wgetrc under .config/wget
if [ ! -f $HOME/.config/wget/wgetrc ]; then
	mkdir -p $HOME/.config/wget
	touch $HOME/.config/wget/wgetrc
fi

# Check if wget is available, install if not
if ! type -p wget >/dev/null; then
	sudo apt update
	sudo apt install wget -y
fi

# Create keyrings directory
sudo mkdir -p -m 755 /etc/apt/keyrings

# Download GitHub CLI keyring
out=$(mktemp)
wget -nv -O"$out" https://cli.github.com/packages/githubcli-archive-keyring.gpg

# Install the keyring
cat "$out" | sudo tee /etc/apt/keyrings/githubcli-archive-keyring.gpg > /dev/null
sudo chmod go+r /etc/apt/keyrings/githubcli-archive-keyring.gpg

# Create sources.list.d directory
sudo mkdir -p -m 755 /etc/apt/sources.list.d

# Add GitHub CLI repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" | sudo tee /etc/apt/sources.list.d/github-cli.list > /dev/null

# Update package list and install GitHub CLI
sudo apt update
sudo apt install gh -y

# Clean up temporary file
rm -f "$out"