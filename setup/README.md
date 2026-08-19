# Setup

One-time environment bring-up for each of the 4 developer PCs. All commands run **inside WSL2 Ubuntu 24.04**, never on native Windows.

## 0. Prerequisite: WSL2 + Ubuntu 24.04 (run on Windows, PowerShell as Administrator)

```powershell
wsl --install -d Ubuntu-24.04
wsl --set-default-version 2
```

Reboot if prompted, then open "Ubuntu 24.04" from the Start menu and create your Linux user.

## 1. Clone the repo (inside WSL2)

```bash
cd ~
git clone <REPO_URL> gps-denied-uav
cd gps-denied-uav
```

## 2. Install dependencies

```bash
chmod +x setup/*.sh
./setup/install_dependencies.sh
```

Installs: ROS 2 Jazzy desktop, Gazebo (ros_gz/harmonic), PX4 SITL source tree, build-essential/CMake/GCC, Eigen3, PCL, colcon, rosdep, Python tooling.

## 3. Verify

```bash
./setup/verify_environment.sh
```

Every dependency prints `PASS` or `FAIL`. Do not proceed until everything is `PASS`. See `docs/DEVELOPMENT.md` for common fixes.

## 4. Build the workspace

```bash
./setup/setup_workspace.sh
source install/setup.bash
```

This also appends workspace sourcing to `~/.bashrc` so new shells pick it up automatically.

## 5. Pinned versions

See [versions.txt](versions.txt). If your `verify_environment.sh` reports an older version, upgrade the package rather than lowering the pin — the pin is the contract all 4 machines must match.

## VS Code

Install the "WSL" extension and open the repo with `code .` from inside the WSL2 shell (not from Windows Explorer). This ensures IntelliSense resolves ROS 2/Eigen/PCL headers from the Linux filesystem.
