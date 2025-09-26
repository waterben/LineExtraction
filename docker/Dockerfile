
ARG VARIANT="ubuntu:24.04"

# Base image with all our dependencies needed for CI.
FROM ${VARIANT} AS base


ENV BAZELISK_BASE_URL="https://github.com/bazelbuild/bazel/releases/download"
# Re-propagate ARG-value in new build stage
# See: https://docs.docker.com/engine/reference/builder/#understand-how-arg-and-from-interact
ARG VARIANT
COPY base /tmp/base
COPY --chmod=755 scripts/install_apt_packages_from_list scripts/uvx-wrapper /usr/local/bin
# Copy tool installation scripts from repo
COPY --from=repo_root_context --chmod=755 tools/scripts /tmp/tools/scripts
RUN <<EOF
  set -eux
  apt-get update
  install_apt_packages_from_list --file /tmp/base/common_packages.txt
  # cp /tmp/base/certs/*.crt /usr/local/share/ca-certificates
  locale-gen en_US en_US.UTF-8
  update-ca-certificates
  rm -rf /var/lib/apt/lists/* /tmp/base
EOF
ENV REQUESTS_CA_BUNDLE=/etc/ssl/certs/ca-certificates.crt

# Install system packages first
RUN echo "Installing system packages..." && /tmp/tools/scripts/install-system-packages.sh

# Install UV (separated to identify failures)
RUN ["/bin/bash", "-c", "set -eux && echo 'Installing UV...' && source /tmp/tools/scripts/install-uv.sh && install_uv /usr/local/bin false"]

# Install Bazel (separated to identify failures)
RUN ["/bin/bash", "-c", "echo 'Installing Bazel...' && source /tmp/tools/scripts/install-bazel.sh && install_bazel /usr/local/bin"]

# Install clangd (separated to identify failures)
RUN ["/bin/bash", "-c", "echo 'Installing clangd...' && source /tmp/tools/scripts/install-clangd.sh && install_clangd /usr/local/bin"]

# Install additional tools that aren't in the modular scripts yet
RUN <<EOF
  set -eux
  # Source tool versions for consistency
  source /tmp/tools/scripts/tool-versions.sh
  
  echo "Installing buildifier..."
  curl -Ls https://github.com/bazelbuild/buildtools/releases/download/v${BUILDIFIER_VERSION}/buildifier-linux-amd64 -o /usr/local/bin/buildifier
  chmod 755 /usr/local/bin/buildifier
  
  echo "Installing GitHub CLI..."
  curl -Ls https://github.com/cli/cli/releases/download/v${GH_VERSION}/gh_${GH_VERSION}_linux_amd64.tar.gz | tar xfz - --strip-components=1 -C /usr/local
  
  echo "Installing sonar-scanner..."
  curl -Ls https://binaries.sonarsource.com/Distribution/sonar-scanner-cli/sonar-scanner-cli-4.6.2.2472-linux.zip | bsdtar xf - --strip-components=1 -C /usr/local
  chmod 755 /usr/local/bin/sonar*
  
  echo "Installing actionlint..."
  curl -Ls https://github.com/rhysd/actionlint/releases/download/v${ACTIONLINT_VERSION}/actionlint_${ACTIONLINT_VERSION}_linux_amd64.tar.gz | tar xfz - -C /usr/local/bin
  chmod 755 /usr/local/bin/actionlint
  
  echo "Installing yq..."
  curl -Ls https://github.com/mikefarah/yq/releases/download/v${YQ_VERSION}/yq_linux_amd64 -o /usr/local/bin/yq
  chmod 755 /usr/local/bin/yq
  
  echo "Installing ruff..."
  curl -Ls https://github.com/astral-sh/ruff/releases/download/${RUFF_VERSION}/ruff-x86_64-unknown-linux-musl.tar.gz | tar xfz - --strip-components=1 -C /usr/local/bin
  chmod 755 /usr/local/bin/ruff
  
  # Clean up installation scripts
  rm -rf /tmp/tools
EOF

# Verify all installations
RUN <<EOF
  set -eux
  echo "Verifying installations..."
  yq --version
  buildifier --version
  bazel --version
  actionlint --version
  uv --version
  gh --version
EOF

ENV UV_PYTHON_INSTALL_DIR="/opt/python"
# ENV PYTHON_3_8_INTERPRETER_PATH="/opt/python/cpython-3.8.5-linux-x86_64-gnu/bin/python"
ENV PYTHON_3_10_INTERPRETER_PATH="/opt/python/cpython-3.10.11-linux-x86_64-gnu/bin/python"
# ENV PYTHON_3_8_VENV_PATH="/opt/venv/deps/python3.8"
ENV PYTHON_3_10_VENV_PATH="/opt/venv/deps/python3.10"
COPY --from=repo_root_context pyproject.toml uv.lock /tmp/
RUN <<EOF
  set -eux
  mkdir -p /opt/python
  mkdir -p /opt/venv
  # uv python install 3.8.5
  uv python install 3.10.11
  uv python install ${PYTHON_DEV_VERSION}

  # creating virtual environments
  # uv venv --python ${PYTHON_3_8_INTERPRETER_PATH} ${PYTHON_3_8_VENV_PATH} --seed
  uv venv --python ${PYTHON_3_10_INTERPRETER_PATH} ${PYTHON_3_10_VENV_PATH} --seed

  # installing dependencies into virtual environments
  # cd /tmp/ && . ${PYTHON_3_8_VENV_PATH}/bin/activate && uv sync --active --locked && deactivate
  cd /tmp/ && . ${PYTHON_3_10_VENV_PATH}/bin/activate && uv sync --active --locked && deactivate


  for cmd in pre-commit rstcheck pymarkdownlnt yamllint docutils; do
    ln -s /usr/local/bin/uvx-wrapper /usr/local/bin/$cmd
  done
  rm /tmp/pyproject.toml /tmp/uv.lock

  # Set permissions (skip setfacl if not available)
  chmod -R 755 /opt/venv || true
  # setfacl --recursive -m u::rwx,g::rwx,o::rwx /opt/venv || true
  # setfacl --recursive --default -m u::rwx,g::rwx,o::rwx /opt/venv || true

  # check if the virtual environments are having packages installed
  # . ${PYTHON_3_8_VENV_PATH}/bin/activate && pip list | wc -l | awk '$1 < 20 { exit 1 }'
  . ${PYTHON_3_10_VENV_PATH}/bin/activate && pip list && echo "Package count: $(pip list | wc -l)"
EOF

# ENV PATH="/opt/venv/deps/python3.8/bin:$PATH"
ENV PATH="/opt/venv/deps/python3.10/bin:$PATH"
CMD ["/bin/bash"]

FROM base AS cicd
RUN : \
  && useradd -c "" --no-log-init -u 1001 -m github-action-runner \
  && echo github-action-runner ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/github-action-runner
USER github-action-runner
RUN <<EOF
  set -eux
  pre-commit --help
  rstcheck --help
  pymarkdownlnt --help
  yamllint --help
  docutils --help
EOF

# Devenv image with user information added to the system.
FROM base AS devenv
ARG USER=dockeruser
# 1000/998: default ids. Should be set by the caller and need to match ids of host user.
ARG UID=1000
ARG GID=1000
ARG DOCKER_GID=998

# Check if a user with this UID exists, and delete them before readding them later
RUN getent passwd ${UID} && userdel --remove $(getent passwd ${UID} | cut -d: -f1) || true

COPY devenv/common_packages.txt /tmp/
COPY devenv/entrypoint.sh /devenv_entrypoint.sh
RUN : \
  && echo "LASTLOG_UID_MAX    0" >> /etc/login.defs \
  && rm -f /var/log/faillog /var/log/lastlog \
  && apt-get update -qq \
  && DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends -y $(cat /tmp/common_packages.txt | tr '\n' ' ') \
  && mkdir -p /workspace \
  && groupadd --gid ${GID} ${USER} \
  && groupadd --gid ${DOCKER_GID} docker \
  && useradd -c "" --no-log-init -u ${UID} -g ${GID} -m -G docker ${USER} \
  && echo "Defaults  env_keep += \"http_proxy https_proxy HTTP_PROXY HTTPS_PROXY no_proxy NO_PROXY\"" >> /etc/sudoers \
  && echo ${USER} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USER} \
  && install -m 755 /dev/null /usr/local/bin/devenv_entrypoint_custom.sh \
  && chmod 755 /devenv_entrypoint.sh \
  && rm -rf /var/lib/apt/lists/* /tmp/common_packages.txt /tmp/tools

RUN <<EOF
  set -eux
  # Install clangd using shared script
  /tmp/tools/install-clangd.sh "/usr/local"
  
  # Install git prompt script
  curl -Ls https://raw.githubusercontent.com/git/git/master/contrib/completion/git-prompt.sh -o /usr/local/bin/git-prompt.sh
EOF

COPY devenv/custom /tmp/custom
RUN : [ -f /tmp/custom/entrypoint.sh ] && cat /tmp/custom/entrypoint.sh > /usr/local/bin/devenv_entrypoint_custom.sh || true

USER ${USER}
WORKDIR /workspace
ENTRYPOINT ["/devenv_entrypoint.sh"]
CMD [ "sleep", "infinity" ]
