# Foxglove

Use Foxglove Desktop to inspect the live ROS 2 graph for Planning and Control.
Foxglove Bridge runs inside Planning and exposes the graph over a WebSocket.
AP1 uses host networking so the bridge can discover the ROS nodes and Foxglove
Desktop can reach the bridge on the host.

1. Ask a team admin for an invite to the team's Foxglove organization and accept
   it. You need a Developer seat for direct live connections.
2. [Download Foxglove Desktop](https://foxglove.dev/download) for your OS.
   Open the app, sign in with the invited account, and select the team's
   organization.

## macOS

### 1. Start Planning and Control

Install one container runtime on your Mac. Both options provide Docker Compose
and Buildx.

- **OrbStack** (recommended). Install and open [OrbStack](https://orbstack.dev/download).
  If your terminal is using a different Docker context, run
  `docker context use orbstack`. With [OrbStack host networking](https://docs.orbstack.dev/docker/network),
  the bridge is reachable on macOS at `localhost`.
- **Docker Desktop**. Install and open [Docker Desktop](https://docs.docker.com/get-docker/desktop/)
  4.34 or later. Under **Settings → Resources → Network**, enable
  [host networking](https://docs.docker.com/engine/network/drivers/host/) and
  select **Apply and restart**. Enhanced Container Isolation must be disabled.
  If your terminal is using a different Docker context, run
  `docker context use desktop-linux`.

Run the same Compose commands from the AP1 repo root with either runtime:

```bash
docker context show
test -f .env || cp .env.example .env
docker compose up -d planning control
docker compose ps planning control
```

Confirm the context matches your runtime and both services show **Up**. If
Compose cannot pull the images, or you want to run the code from your checkout,
build them in the active runtime and start the services:

```bash
docker buildx bake -f docker/docker-bake.hcl --load planning control
docker compose up -d planning control
```

### 2. Install and launch Foxglove Bridge

From the repo root, install the [Foxglove Bridge for ROS 2](https://docs.foxglove.dev/docs/getting-started/frameworks/ros2)
as root inside the running Planning container. Then open a shell as the container's regular user.

```bash
docker compose exec --user root planning bash -c 'apt-get update && apt-get install -y ros-jazzy-foxglove-bridge'
docker compose exec planning bash
```

Inside the Planning container, source the AP1 ROS environment, check for
`/ap1_control` and `/ap1_planning`, and start the bridge:

```bash
source /usr/local/share/ap1/ap1_env.sh
ros2 node list
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

Leave the terminal running once the bridge reports that it is listening on
port `8765`. Repeat the package install if Compose recreates the Planning
container.

### 3. Open the Topic Graph

1. In Foxglove Desktop, select **Open connection → Foxglove WebSocket**.
2. Enter `ws://127.0.0.1:8765` and connect.
3. Select **Add panel → [Topic Graph](https://docs.foxglove.dev/docs/visualization/panels/topic-graph)**. Look for the Planning and Control nodes
   and their publisher and subscriber arrows.
4. In the graph toolbar, open the topic visibility menu and select
   **Subscribed topics**. To show only topics with both a publisher and a
   subscriber, select **Connected topics**.

Use **Orientation** and **Zoom fit** to arrange the view. You can drag individual
items to clear overlaps during a session. Manually dragged positions did not
persist when reopening the saved layout in the tested setup.

To end the session, press **Ctrl+C** in the bridge terminal, exit the container
shell, and run `docker compose stop planning control` from the repo root.

## Windows

The Windows workflow still needs an end-to-end test.

### 1. Set up Docker Engine in Ubuntu on WSL 2

1. Set up Ubuntu 24.04 using the [AP1 WSL 2 instructions](../docker/README.md#windows-wsl-2).
2. In Ubuntu, install Docker Engine, Compose, and Buildx using the
   [Ubuntu Docker installation steps](../docker/README.md#setup).
3. Open your AP1 checkout from the Ubuntu terminal. Keep the checkout in the
   Ubuntu filesystem, for example under `~/code`.

These steps use Docker Engine inside Ubuntu so AP1 containers and ROS nodes
running directly in Ubuntu can share the same Linux network.

### 2. Start Planning and Control

From the AP1 repo root inside Ubuntu, run:

```bash
test -f .env || cp .env.example .env
docker compose up -d planning control
docker compose ps planning control
```

If you need a local build, use the [Planning and Control build commands](../docker/README.md#control-and-planning)
in the same Ubuntu terminal.

### 3. Install and launch Foxglove Bridge

Run the [bridge installation and launch commands](#2-install-and-launch-foxglove-bridge)
from your Ubuntu terminal. Leave the bridge running inside Planning.

### 4. Open the Topic Graph

In Foxglove Desktop on Windows, follow the [Topic Graph steps](#3-open-the-topic-graph)
using `ws://127.0.0.1:8765`. Windows can reach Linux services through
[WSL 2 localhost forwarding](https://learn.microsoft.com/windows/wsl/networking).
