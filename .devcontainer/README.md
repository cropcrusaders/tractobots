# Dev Container

This repository includes a compressed copy of the **AgIsoStack++** library which
contains a `.devcontainer` folder with configuration files for Visual Studio
Code. The configuration uses the `mcr.microsoft.com/devcontainers/cpp:0-ubuntu-22.04`
base image and installs `can-utils` so example applications can interface with
SocketCAN.

To use the container locally:

1. Install the [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)
   extension for VS Code.
2. Unzip `AgIsoStack-plus-plus-main.zip` if you wish to work directly with the
   upstream library.
3. Open the extracted folder (or this repository) in VS Code and choose
   **Reopen in Container**.

The Dockerfile supports an optional `REINSTALL_CMAKE_VERSION_FROM_SOURCE` build
argument. Set this to a version (e.g. `3.22.6`) to compile a newer CMake inside
the container. The devcontainer also exposes network access (`--network=host`)
and runs `gcc -v` after creation to verify the toolchain.
