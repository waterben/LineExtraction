#!/usr/bin/env bash

echo "LineExtraction entrypoint!"

mkdir -p ~/.cache
mkdir -p ~/.ccache
mkdir -p ~/.ssh

echo "Installing pre-commit!"
# Activate git pre-commit for git commits
pre-commit install -f


# Avoid extension reinstalls on container rebuild
# https://code.visualstudio.com/docs/remote/containers-advanced#_avoiding-extension-reinstalls-on-container-rebuild
mkdir -p \
  ~/.vscode-server/extensions \
  ~/.vscode-server-insiders/extensions \

# Persist bash history between runs
# https://code.visualstudio.com/remote/advancedcontainers/persist-bash-history
mkdir -p ~/commandhistory
# Fix ownership if the volume was created with wrong permissions
sudo chown -R $(id -u):$(id -g) ~/commandhistory
touch ~/commandhistory/.bash_history

echo "export HISTFILE=~/commandhistory/.bash_history" >> ~/.bashrc

. /usr/local/bin/devenv_entrypoint_custom.sh

exec "$@"
