# Building of devcontainer

## build_dev_images.sh

Create docker image for local usage. You have to be inside the `docker` folder.
Usage example: `./build_dev_images.sh noble`

## Dockerfile

Multi-staged dockerfile to create base and devenv image.
Base images should also be used inside CI.

## base/packages_*.txt

Packages to be installed during dev container image build in the base layer.

## devenv customization

Users can customize their specific devenv docker environment by adapting `devenv/custom/entrypoint.sh` or install
Specific debian packages via `devenv/custom/packages.txt` and/or pip packages via `devenv/custom/requirements-pip.txt`.
Note, that whenever the former files are modified, a docker rebuild needs to be retriggered, for instance by deleting the old image
via `docker rmi ...`.
