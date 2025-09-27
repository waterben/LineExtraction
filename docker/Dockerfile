
ARG VARIANT="ubuntu:24.04"

# Base image with all our dependencies needed for CI and development
# Uses multi-stage builds to keep the final image size small
FROM ${VARIANT} AS base

# Re-propagate ARG-value in new build stage
# See: https://docs.docker.com/engine/reference/builder/#understand-how-arg-and-from-interact
ARG VARIANT

# Install common dependencies
COPY base /tmp/base
COPY --chmod=755 scripts/install_apt_packages_from_list scripts/install_base_tools /usr/local/bin
RUN <<EOF
  set -eux
  apt-get update
  install_apt_packages_from_list --file /tmp/base/common_packages.txt
  locale-gen en_US en_US.UTF-8
  update-ca-certificates
  rm -rf /var/lib/apt/lists/* /tmp/base
EOF
ENV REQUESTS_CA_BUNDLE=/etc/ssl/certs/ca-certificates.crt

# Install base tools from custom sources
RUN install_base_tools

# Setup Python environment
COPY --from=repo_root_context pyproject.toml /tmp/
# Copy uv.lock if it exists to preserve dependency versions
RUN --mount=from=repo_root_context,source=.,target=/src \
    if [[ -f /src/uv.lock ]]; then \
        echo "Copying existing uv.lock for reproducible builds"; \
        cp /src/uv.lock /tmp/; \
    else \
        echo "No uv.lock found, will generate new one"; \
    fi
COPY --chmod=755 scripts/setup_python_env scripts/uvx_wrapper /usr/local/bin
RUN setup_python_env

# Copy uv.lock back to source and clean up temp files
RUN --mount=from=repo_root_context,source=.,target=/src,rw \
    if [[ -f /tmp/uv.lock ]]; then \
        echo "Copying uv.lock back to repository root"; \
        cp /tmp/uv.lock /src/uv.lock; \
    fi && \
    rm -f /tmp/pyproject.toml /tmp/uv.lock

ENV PATH="/opt/venv/deps/python/bin:$PATH"

# Default to bash shell
CMD ["/bin/bash"]

# CI image with user information added to the system (currently not used).
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

# Check if a user with this UID exists, and delete them before reading them later
RUN getent passwd ${UID} && userdel --remove $(getent passwd ${UID} | cut -d: -f1) || true

# Install common dependencies, add user, sudo and entrypoint
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
  && rm -rf /var/lib/apt/lists/* /tmp/common_packages.txt

# Install development tools from custom sources
COPY --chmod=755 scripts/install_devenv_tools /usr/local/bin
RUN install_devenv_tools

USER ${USER}
WORKDIR /workspace
ENTRYPOINT ["/devenv_entrypoint.sh"]
CMD [ "sleep", "infinity" ]
