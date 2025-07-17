#!/bin/bash

SESSION="agilicious-onboard"

# Start tmux session, detached
tmux new-session -d -s $SESSION -n main

# Pane 0
tmux send-keys -t $SESSION:0.0 'cd ~/catkin_ws' C-m

# Split horizontally: Pane 0 (left), Pane 1 (right)
tmux split-window -h -t $SESSION:0.0
tmux send-keys -t $SESSION:0.1 'htop'

# Split Pane 0 vertically (left half becomes top-left and bottom-left)
tmux split-window -v -t $SESSION:0.0
tmux send-keys -t $SESSION:0.0 'nvim'

# Split Pane 1 vertically (right half becomes top-right and bottom-right)
tmux split-window -v -t $SESSION:0.1
tmux send-keys -t $SESSION:0.1 'tail -f /var/log/syslog'

# Attach session
tmux attach -t $SESSION