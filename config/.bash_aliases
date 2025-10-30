#! /bin/bash

alias clc='clear';
alias ..='cd ..';
alias ...='cd ../..';
alias ....='cd ../../..';

# include custom functions if available
# shellcheck source=/home/ros/.funcrc
# shellcheck disable=SC1091
[ -f ~/.funcrc ] && . ~/.funcrc

# ROS workspace alias
alias cw='cd "$ROS_WORKSPACE"';
alias rc='ros_clean';
alias rs='ros_source';
alias rb='ros_build';
alias ru='ros_update';
alias rsrch='ros_search';