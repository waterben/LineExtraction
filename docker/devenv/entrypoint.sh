#!/usr/bin/env bash

echo "Initialize command line history..."
# Create cmd_history directory and setup bash history persistence
mkdir -p ~/.cmd_history
# Fix ownership if directory was created as root (common with docker volumes)
USER_ID=$(id -u)
GROUP_ID=$(id -g)
sudo chown -R "${USER_ID}:${GROUP_ID}" ~/.cmd_history 2>/dev/null || true
touch ~/.cmd_history/.bash_history
echo "export HISTFILE=~/.cmd_history/.bash_history" >> ~/.bashrc


echo "Activate git pre-commit for commits..."
pre-commit install -f

# Use config from repo
exec "$@"
