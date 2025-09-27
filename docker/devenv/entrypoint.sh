#!/usr/bin/env bash

echo "LineExtraction entrypoint!"

mkdir -p ~/.cache
mkdir -p ~/.ccache
mkdir -p ~/.ssh

# Persist bash history between runs
# https://code.visualstudio.com/remote/advancedcontainers/persist-bash-history
mkdir -p ~/commandhistory

# Fix ownership of created directories if it was created with wrong permissions
USER_ID=$(id -u)
GROUP_ID=$(id -g)
if [[ -d ~/.cache ]]; then
    sudo chown -R "${USER_ID}:${GROUP_ID}" ~/.ccache
fi

if [[ -d ~/.ccache ]]; then
    sudo chown -R "${USER_ID}:${GROUP_ID}" ~/.ccache
fi

if [[ -d ~/.ssh ]]; then
    sudo chown -R "${USER_ID}:${GROUP_ID}" ~/.ccache
fi

if [[ -d ~/commandhistory ]]; then
    sudo chown -R "${USER_ID}:${GROUP_ID}" ~/commandhistory
fi

touch ~/commandhistory/.bash_history
echo "export HISTFILE=~/commandhistory/.bash_history" >> ~/.bashrc

echo "Installing pre-commit!"
# Activate git pre-commit for git commits
pre-commit install -f


# Avoid extension reinstalls on container rebuild
# https://code.visualstudio.com/docs/remote/containers-advanced#_avoiding-extension-reinstalls-on-container-rebuild
mkdir -p \
  ~/.vscode-server/extensions \
  ~/.vscode-server-insiders/extensions \

. /usr/local/bin/devenv_entrypoint_custom.sh

exec "$@"
