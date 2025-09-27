# Docker and DevContainer Setup

## Docker-based Development (Recommended)

### build_dev_images.sh

Create docker image for local usage. You have to be inside the `docker` folder.
Usage example: `./build_dev_images.sh noble`

Supported Ubuntu versions: `focal`, `jammy`, `noble`

### Dockerfile

Multi-staged dockerfile to create base and devenv image.
Base images should also be used inside CI.

### devenv customization

Users can customize their specific devenv docker environment by adapting `devenv/custom/entrypoint.sh` or install
specific debian packages via `devenv/custom/packages.txt` and/or pip packages via `devenv/custom/requirements-pip.txt`.
Note, that whenever the former files are modified, a docker rebuild needs to be triggered, for instance by deleting the old image
via `docker rmi ...`.

## Local Development Setup (Without Docker)

If you prefer to work without Docker, you can set up the development environment locally by installing the same packages that are used in the Docker container.

### Ubuntu/Debian Setup

1. **Install system packages from base layer:**

```bash
sudo apt update
sudo apt install sudo ca-certificates curl cmake doxygen fontconfig moreutils jq \
  build-essential llvm clang gcc g++ git git-lfs python3 python3-venv \
  clang-format clang-tidy ccache lcov locales zstd libarchive-tools \
  shellcheck acl python3-dev xxd
```

2. **Install additional development packages:**

```bash
sudo apt install vim less gdb nano zsh
```

3. **Install UV (Python package manager):**

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

4. **Install Bazel:**

```bash
curl -Ls https://github.com/bazelbuild/bazelisk/releases/download/v1.26.0/bazelisk-linux-amd64 -o /tmp/bazelisk
sudo mv /tmp/bazelisk /usr/local/bin/bazel
sudo chmod +x /usr/local/bin/bazel
```

5. **Install clangd:**

```bash
curl -Ls https://github.com/clangd/clangd/releases/download/21.1.0/clangd-linux-21.1.0.zip | sudo bsdtar xf - --strip-components=1 -C /usr/local
```

6. **Setup Python environment:**

```bash
# Install Python dependencies
uv venv .venv
source .venv/bin/activate
uv sync --locked
```

7. **Install pre-commit hooks:**

```bash
pre-commit install --config .pre-commit-config.yaml
```

### Build Project

```bash
# For CMake build (as documented in main README)
mkdir build && cd build
cmake ..
cmake --build . -j$(nproc)

# For Bazel build (if using Bazel)
bazel build //...
```

## Package Files

- **base/packages_*.txt** - Packages to be installed during dev container image build in the base layer
- **devenv/common_packages.txt** - Additional packages for development environment
