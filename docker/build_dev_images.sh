#!/bin/bash

# This script is only supposed to be used by developers to easily create dev images.
set -euo pipefail
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
REPO_ROOT_DIR=$(git rev-parse --show-toplevel)
current_dir=$(pwd)
cd $SCRIPT_DIR

VALID_DISTRIBUTION_VERSION_NAMES=("ubuntu:focal" "ubuntu:jammy" "ubuntu:noble")

usage="
$(basename "$0") is supposed to provide the docker build command to create the devenv image.
Build the devimage with the following command:
  bash $(basename "$0") <distribution_version_name>
where:
  <distribution_version_name> is one of the following:
    ${VALID_DISTRIBUTION_VERSION_NAMES[@]}
"

if [ "$#" -ne 1 ]; then
  echo -e "\e[0;31mERROR: Wrong number of parameters passed to $(basename "$0")" 1>&2
  echo -e "\e[m$usage" 1>&2
  exit 1
fi

DISTRIBUTION_VERSION_NAME=${1,} # convert the first parameter to lower case


[[ ! " ${VALID_DISTRIBUTION_VERSION_NAMES[@]} " =~ " ${DISTRIBUTION_VERSION_NAME} " ]] && echo "Invalid target name: ${DISTRIBUTION_VERSION_NAME}" && exit 1

DOCKER_TAG=$(whoami)-devcontainer-line-extraction


# build devenv image
DOCKER_BUILDKIT=1 docker build ${SCRIPT_DIR}  \
  --build-context repo_root_context="${REPO_ROOT_DIR}" \
  -f $SCRIPT_DIR/Dockerfile                   \
  --network=host                              \
  --progress=plain                            \
  --build-arg http_proxy=${http_proxy:=""}    \
  --build-arg https_proxy=${https_proxy:=""}  \
  --build-arg no_proxy=${no_proxy:=""}        \
  --build-arg HTTP_PROXY=${HTTP_PROXY:=""}    \
  --build-arg HTTPS_PROXY=${HTTPS_PROXY:=""}  \
  --build-arg NO_PROXY=${NO_PROXY:=""}        \
  --build-arg VARIANT="${DISTRIBUTION_VERSION_NAME}"              \
  --build-arg DOCKER_GID="$(getent group docker | cut -d : -f 3)" \
  --build-arg USER="$(whoami)"                \
  --build-arg UID="$(id -u)"                  \
  --build-arg GID="$(id -g)"                  \
  --target devenv                             \
  --tag "${DOCKER_TAG}-devenv"

cd $current_dir
