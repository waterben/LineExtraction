# Environment customization

- Shell
  - VSCode devcontainer will use the same shell that is defined in the environment via the ```${SHELL}``` variable

- Initialization
  - Create the file ```${WORKSPACE}/.devcontainer/scripts/custom/initialize_custom.sh```
  - Adjust it to your needs, it will be sourced **AFTER** the standard initialization script
  - The message *"Custom initialization settings applied"* will be shown if sources successful

- Post-start
  - Create the file ```${WORKSPACE}/.devcontainer/scripts/custom/post_start_custom.sh```
  - Adjust it to your needs, it will be sourced **INSTEAD** of the default settings
  - The message *"Custom post start settings applied"* will be shown if sources successful

The directory ```.devcontainer/scripts/custom``` is already completely excluded from git. You can therefore
store other custom stuff there without worrying about git.

If you change your shell, ensure you have the necessary packages installed (use custom packages in docker if needed)

## Rebuilding Container

### VS Code

For rebuilding the container it is best to rebuild the container with F1->Dev Containers: Rebuild Container.
Note: If you try to rebuild by reopening the container by closing VS Code, make sure that the Docker container is actually stopped
before re-opening VS Code.
Note: Changing the Dockerfile or the script which builds the image won't trigger VS Code to prompt you for a rebuild of the container.
