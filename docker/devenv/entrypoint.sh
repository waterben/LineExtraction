#!/usr/bin/env bash

echo "Initialize development environment..."

# Create cmd_history directory and setup bash history persistence
mkdir -p ~/.cmd_history
# Fix ownership if directory was created as root (common with docker volumes)
USER_ID=$(id -u)
GROUP_ID=$(id -g)
sudo chown -R "${USER_ID}:${GROUP_ID}" ~/.cmd_history 2>/dev/null || true
touch ~/.cmd_history/.bash_history

# Source project environment in bashrc (if not already configured)
if ! grep -q "source.*/workspace/.project_env" ~/.bashrc 2>/dev/null; then
    echo "# Source project environment" >> ~/.bashrc
    echo 'if [[ -f "/workspace/.project_env" ]]; then' >> ~/.bashrc
    echo '    source "/workspace/.project_env"' >> ~/.bashrc
    echo 'fi' >> ~/.bashrc
fi

echo "Activate git pre-commit for commits..."
pre-commit install -f

# Use config from repo
exec "$@"
