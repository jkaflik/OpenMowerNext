# AGENTS.md

## Repo Shape
- This repo root is the ROS 2 Jazzy workspace and a single `ament_cmake` package named `open_mower_next`.
- Do not add nested ROS packages under `src/`; C++ nodes live in functional dirs like `src/map_server`, `src/map_recorder`, `src/docking_helper`, `src/coverage_server`, and `src/sim`.
- Keep each node's headers and sources together; add new node targets through root `CMakeLists.txt` and the `cmake/*.cmake` include files.
- Interface files live in `src/msg`, `src/srv`, and `src/action`; add new ones to `rosidl_generate_interfaces()` in root `CMakeLists.txt`.
- Runtime resources installed by CMake are `config`, `launch`, `description`, `worlds`, `maps`, `protos`, and `resource`; update the root `INSTALL(DIRECTORY ...)` if adding another runtime resource dir.

## Environment
- Use ROS 2 Jazzy. The devcontainer mounts the repo at `/opt/ws` and runs post-create `make custom-deps deps`.
- Launches need `OM_MAP_PATH`, `OM_DATUM_LAT`, and `OM_DATUM_LONG`; `.devcontainer/openmower_config.bash` sources `.devcontainer/override/.env` if present, else `.devcontainer/default.env`.
- `docs/configuration.md` says `OM_DATUM_LON`, but the launch code reads `OM_DATUM_LONG`.
- `src/lib` is for external deps imported from `custom_deps.yaml`; `make custom-deps` uses `vcs import --force --shallow` and can overwrite those checkouts.

## Commands
- Fresh dependency setup: `make custom-deps deps`.
- Build all: `make build` (`colcon build --symlink-install`).
- Build only external libs: `make build-libs`.
- Release build: `make build-release`.
- Focused package build: `colcon build --symlink-install --packages-select open_mower_next`.
- Source before direct launches: `source /opt/ros/$ROS_DISTRO/setup.bash && source install/setup.bash`.
- Hardware launch from repo root: `make run`.
- Sim launch from repo root: `make sim`; it sources ROS, the installed workspace, `.devcontainer/default.env`, then launches `open_mower_next sim.launch.py` with Webots.
- Headless Webots smoke launch: `WEBOTS_OFFSCREEN=1 ros2 launch open_mower_next sim.launch.py gui:=false mode:=fast` after sourcing ROS and `install/setup.bash`.
- ROS MCP rosbridge foreground launch: `make rosbridge`; user service setup: `make rosbridge-service-enable`.
- Direct ROS package launches use `open_mower_next`, e.g. `ros2 launch open_mower_next openmower.launch.py` or `ros2 launch open_mower_next sim.launch.py`.
- Docs are isolated under `docs/`: `cd docs && npm ci && npm run docs:build`; dev server is `npm run docs:dev`.

## Verification
- CI builds Docker images; the root Dockerfile runs `make custom-deps deps` then `make build-release`.
- Unit CI runs `colcon test --ctest-args -LE integration`; integration CI is explicit and runs `colcon test --ctest-args -L integration`.
- Docs CI uses Node 18, `docs/package-lock.json`, and `npm run docs:build`.
- Repository-wide ament linters that scan `src/lib` or `docs/node_modules` are disabled in CMake; add focused tests instead of relying on global lint passes.
- Webots integration tests are gated behind `OPEN_MOWER_NEXT_ENABLE_INTEGRATION_TESTS=ON` and require `WEBOTS_HOME`, `WEBOTS_OFFSCREEN=1`, `xvfb`, and `libxcb-cursor0`.
- The launch test at `src/map_server/test/load_geojson_map.test.py` is not registered in CMake; do not assume `colcon test` runs it.
- C++ formatting uses `.clang-format` based on Google style with 100 columns and custom braces; `.clang-tidy` exists but no Make/CI target runs it.

## Gotchas
- `make dev` currently `cd`s into `.devcontainer` and runs `docker-compose up -d`, but no compose file exists there; use devcontainer support or root `docker-compose.yaml` directly.
- `launch/rsp.launch.py` references package `open_mower_ros`, unlike the rest of the repo; verify before relying on `make rsp`.
- If moving the docking plugin XML, update `src/docking_helper/plugins.xml`, `cmake/docking_helper.cmake`, `package.xml`, and the manual copy in `Dockerfile`.
- The runtime Docker entrypoint creates an empty GeoJSON map when `OM_MAP_PATH` is missing; normal dev shells do not.
- Webots is the only supported simulation backend; Gazebo assets and `ros_gz` bridge logic are legacy and should not be reintroduced.
- Webots launch depends on `webots_ros2_driver`, `webots_ros2_control`, and a Webots binary available on the host.
- If `webots_ros2_driver` auto-installs Webots, it usually lands in `~/.ros/webotsR2025a/webots`; export `WEBOTS_HOME` to that path or use `make sim`.
- Project OpenCode config starts `ros-mcp` with `uvx`; restart OpenCode after config changes or after installing `uv`.
- Rosbridge for MCP binds to `127.0.0.1:9090` by default; prefer SSH tunneling over binding it to the LAN.
