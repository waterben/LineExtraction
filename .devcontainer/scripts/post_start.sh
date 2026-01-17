#!/usr/bin/env bash
set -euo pipefail
shopt -s nullglob globstar

# This script is executed each time the container is successfully started,
# in order to ensure the environment is properly set up in the container for development

echo "Initialize development folders..."
# Make sure required folders exist (including cmd_history for consistency)
mkdir -p ~/.cache ~/.ccache ~/.ssh ~/.cmd_history ~/.vscode-server ~/.vscode-server-insiders

# Fix ownership of created directories if they were created with wrong permissions
USER_ID=$(id -u)
GROUP_ID=$(id -g)
sudo chown -R "${USER_ID}:${GROUP_ID}" ~/.cache ~/.ccache ~/.ssh ~/.cmd_history ~/.vscode-server ~/.vscode-server-insiders 2>/dev/null || true

touch ~/.cmd_history/.bash_history

# Enable history search using page-up and page-down
echo '$include /etc/inputrc' >  ~/.inputrc
echo '# Map "page up" and "page down" to search the history' >> ~/.inputrc
echo '"\e[5~": history-search-backward' >> ~/.inputrc
echo '"\e[6~": history-search-forward ' >> ~/.inputrc

# Create a VSCode profile with git support for PS1 using prompt string provided by git
echo '# Use local copy of the PS1 support scripts provided by git' > ~/.vscode_profile
echo "source /usr/local/bin/git-prompt.sh" >> ~/.vscode_profile
# - Color codes shell https://gkarthiks.github.io/quick-commands-cheat-sheet/bash_command.html
echo '# Enable color codes' >> ~/.vscode_profile
echo 'export COLOR_RED="\[\033[1;31m\]"' >> ~/.vscode_profile
echo 'export COLOR_GREEN="\[\033[1;32m\]"' >> ~/.vscode_profile
echo 'export COLOR_BLUE="\[\033[1;34m\]"' >> ~/.vscode_profile
echo 'export COLOR_END="\[\033[0m\]"' >> ~/.vscode_profile
# - Git prompt https://raw.github.com/git/git/master/contrib/completion/git-prompt.sh
# - Git prompt options https://mjswensen.com/blog/git-status-prompt-options/
# - PS1 https://www.cyberciti.biz/faq/bash-shell-change-the-color-of-my-shell-prompt-under-linux-or-unix/
echo '# Change the prompt accordingly' >> ~/.vscode_profile
echo 'export GIT_PS1_SHOWDIRTYSTATE=1' >> ~/.vscode_profile
echo 'export GIT_PS1_SHOWSTASHSTATE=0' >> ~/.vscode_profile
echo 'export GIT_PS1_SHOWUNTRACKEDFILES=0' >> ~/.vscode_profile
echo 'export GIT_PS1_SHOWUPSTREAM="auto"' >> ~/.vscode_profile
echo 'export PS1="${COLOR_GREEN}\u@\h${COLOR_END}:${COLOR_BLUE}\w${COLOR_RED}''\$(__git_ps1)''${COLOR_END}\n> "' >> ~/.vscode_profile
echo '# ccache uses default ~/.ccache directory (mounted as volume)' >> ~/.vscode_profile
echo '# Set the bash history file to be persistent on volume and ensure it is activated before prompt' >> ~/.vscode_profile
echo 'export PROMPT_COMMAND="history -a" && export HISTFILE="${HOME}/.cmd_history/.bash_history"' >>  ~/.vscode_profile

# Source project-specific environment if .project_env exists in workspace
echo '# Source project-specific environment (.project_env) if it exists' >> ~/.vscode_profile
echo 'if [[ -f "${PWD}/.project_env" ]]; then' >> ~/.vscode_profile
echo '    source "${PWD}/.project_env"' >> ~/.vscode_profile
echo 'fi' >> ~/.vscode_profile

# Enable environment in bashrc (only if not already present)
if ! grep -q "source ~/.vscode_profile" ~/.bashrc; then
    echo '# BEGIN - Appended via VSCode postStartCommand' >> ~/.bashrc
    echo 'source ~/.vscode_profile' >> ~/.bashrc
    echo '# END - Appended via VSCode postStartCommand' >> ~/.bashrc
fi

pre-commit install --config .pre-commit-config.yaml
