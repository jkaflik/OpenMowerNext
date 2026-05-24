---
title: ROS MCP
---
# {{ $frontmatter.title }}

## Overview

OpenMowerNext can expose the running ROS graph to OpenCode through [`robotmcp/ros-mcp-server`](https://github.com/robotmcp/ros-mcp-server). The project configuration starts `ros-mcp` as an MCP stdio server, and `ros-mcp` talks to ROS through `rosbridge` on `localhost:9090`.

The default setup is localhost-only. Keep it that way for normal development on the machine running ROS.

## Install dependencies

Install the ROS side:

```bash
make deps
```

If `rosbridge_server` is still missing, install it directly:

```bash
make rosbridge-deps
```

Install `uv` for the OpenCode MCP command if `uvx` is not already available:

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

Restart your shell after installing `uv`, or ensure the `uvx` binary directory is on `PATH` before starting OpenCode.

## Run rosbridge

For foreground development:

```bash
make rosbridge
```

This runs `ros2 launch rosbridge_server rosbridge_websocket_launch.xml` after sourcing ROS 2 Jazzy and the workspace install if it exists.

For a persistent user service:

```bash
make rosbridge-service-enable
```

Useful service commands:

```bash
make rosbridge-service-status
make rosbridge-service-logs
make rosbridge-service-restart
make rosbridge-service-disable
```

The service binds to `127.0.0.1:9090` by default. Override the bind address only on a trusted network or over a VPN:

```bash
make ROSBRIDGE_ADDRESS=0.0.0.0 rosbridge-service-enable
```

If OpenCode runs on another machine, prefer an SSH tunnel instead of exposing rosbridge on the LAN:

```bash
ssh -L 9090:127.0.0.1:9090 <dev-host>
```

## OpenCode MCP

The project `opencode.json` configures:

```json
{
  "mcp": {
    "ros-mcp": {
      "type": "local",
      "command": ["uvx", "ros-mcp", "--transport=stdio"]
    }
  }
}
```

OpenCode reads this file only when it starts. Quit and restart OpenCode after changing the config or after installing `uv`.

Once OpenCode is restarted and rosbridge is running, ask it to connect to the robot on `localhost` and inspect topics or services.
