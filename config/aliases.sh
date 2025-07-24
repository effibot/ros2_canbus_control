#!/bin/bash

# Shell aliases configuration
# Source this file in your .bashrc to enable aliases

# eza aliases (modern ls replacement)
alias ls='eza --icons --group-directories-first --color=always --follow-symlinks'
alias ll='ll -l'
alias la='ls -la'
alias lt='ls --tree'
alias lta='ls --tree -a'
alias ltr='ls --tree --level=2'

# Additional useful aliases
alias grep='grep --color=auto'
alias fgrep='fgrep --color=auto'
alias egrep='egrep --color=auto'
alias clc='clear'
alias fzf='fzf --preview "batcat --color=always --line-range :500 {}" --bind "enter:become(${EDITOR} {})"'
alias cat='batcat --style=plain --color=always'

# Directory navigation aliases
alias ..='cd ..'
alias ...='cd ../..'
alias ....='cd ../../..'

# Git aliases (if git is available)
if command -v git >/dev/null 2>&1; then
    alias gs='git status'
    alias ga='git add'
    alias gc='git commit'
    alias gp='git push'
    alias gl='git log --oneline'
    alias gd='git diff'
fi

# ROS 2 aliases
if [ -n "${ROS_DISTRO:-}" ]; then
    alias cb='colcon build'
    alias cbt='colcon build --cmake-target'
    alias cbp='colcon build --packages-select'
    alias cbs='colcon build --symlink-install'
    alias ct='colcon test'
    alias ctp='colcon test --packages-select'
    alias rosdep_install='rosdep install --from-paths src --ignore-src -r -y'
fi
