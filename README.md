# MDP – Team Lupin – FloraNova Digital Twin

RO47007 Multidisciplinary Project, 2025–2026.
Robot platform: MIRTE Master V2 (holonomic 4-mecanum base, 4-DOF arm).
Client: FloraNova (commercial greenhouse).

> The team is called **Lupin**. The GitLab path is
> `cor/ro47007/2026/group_14/lupin` — `group_14` is the course-assigned
> subgroup, `lupin` is our project inside it.

## Prerequisites

Before cloning this repo, every team member's laptop must have a working
ROS 2 workspace with the MIRTE vendor stack installed. **All of the
following lives outside the `lupin` package — it's shared infrastructure
that every group on this course needs.** Once it's done, the `lupin`
repo drops in as one more package alongside the vendor folders.

### 1. Operating system
**Ubuntu 22.04** (dual-boot or native, **not** WSL).

### 2. ROS 2 Humble
Follow the
[official install guide](https://docs.ros.org/en/humble/Installation.html)
(`ros-humble-desktop` is enough). Make sure
`source /opt/ros/humble/setup.bash` is in your `~/.bashrc` afterwards.

### 3. Workspace + colcon top-level marker
Create the workspace and install the
[`colcon-top-level-workspace`](https://github.com/rhaschke/colcon-top-level-workspace)
extension so `colcon build` works from anywhere inside `~/ros2_ws/`:

```bash
mkdir -p ~/ros2_ws/src
touch ~/ros2_ws/.colcon_root
pip install colcon-top-level-workspace
```

The empty `.colcon_root` file marks the workspace root; the pip extension
teaches `colcon` to find it.

### 4. MIRTE Master vendor packages
Pull the vendor stack into `~/ros2_ws/src/` via `vcstool` and the
`mirte.repos` manifest, then install rosdeps and build. The full
procedure is documented by the MIRTE team — primary references:

- [MIRTE Master developer docs](https://docs.mirte.org/develop/index.html)
- [Simulation install guide](https://docs.mirte.org/0.2.0/doc/simulation/install_simulation.html)
- [Running MIRTE Master in Gazebo](https://docs.mirte.org/0.2.0/doc/simulation/mirte_master_gazebo.html)
- [Main mirte-ros-packages repo](https://github.com/mirte-robot/mirte-ros-packages/tree/develop/)

Individual upstream repos pulled in by `mirte.repos` (for reference):

| Component | Upstream |
| --- | --- |
| Main MIRTE packages | <https://github.com/mirte-robot/mirte-ros-packages/tree/develop/> |
| Mecanum wheel controller | <https://github.com/clearpathrobotics/clearpath_mecanum_drive_controller> |
| Camera (image transport + lazy publish) | <https://github.com/ArendJan/ros2_astra_camera/tree/fix-ros-jammy> |
| Lidar | <https://github.com/Slamtec/rplidar_ros/tree/ros2> |
| Navigation | <https://github.com/kas-lab/mirte_navigation/tree/physical_robot> |
| Gazebo / sim worlds | `mirte-gazebo`, `aws_robomaker_small_house_world`, `plasys_house_world`, `robocup_home_simulation`, `gazebo_grasp_fix`, `sdf_models` |

After this step, `~/ros2_ws/src/` should contain ~15 vendor folders plus
the `mirte.repos` file. Verify with:

```bash
ls ~/ros2_ws/src
```

You can now build the vendor workspace once before adding `lupin`:

```bash
cd ~/ros2_ws
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
source install/setup.bash
```

## Cloning this repository

Clone Lupin's code as a **sibling** of the MIRTE vendor packages —
not inside any of them:

```bash
cd ~/ros2_ws/src
git clone git@gitlab.tudelft.nl:cor/ro47007/2026/group_14/lupin.git
```

If the clone fails with a permissions error, you have not yet added
your SSH key to GitLab. See the
[GitLab SSH guide](https://docs.gitlab.com/ee/user/ssh.html), then add
the public key at
<https://gitlab.tudelft.nl/-/user_settings/ssh_keys>.

After cloning, check out the branch that matches what you want to do:

```bash
cd lupin
git checkout sim          # to develop or run in Gazebo
# or
git checkout hardware     # to run on the real MIRTE Master
```

## Branch model

This repository uses **three long-lived branches**:

| Branch | What lives here |
| --- | --- |
| `main` | Shared code that is identical regardless of deployment target — messages, perception, planning, HMI logic. **Default branch.** Most feature MRs target `main`. |
| `sim` | Everything in `main` plus simulation-specific bringup: Gazebo launch files, sim parameters, fake-hardware bridges, custom worlds. |
| `hardware` | Everything in `main` plus real-robot bringup: MIRTE driver launches, calibrated parameters for our unit, real-camera AprilTag config. The final demonstration runs from this branch. |

**Where do my changes go?**

- Shared logic, algorithms, messages → branch off `main` (`feat/<topic>`), MR back to `main`.
- Sim-only artefacts (new Gazebo world, sim debug tool) → branch off `sim` (`sim/<topic>`), MR back to `sim`.
- Hardware-only artefacts (calibration values, real-robot launch tweak) → branch off `hardware` (`hw/<topic>`), MR back to `hardware`.

When something lands on `main`, a maintenance MR merges `main` into `sim` and `hardware` so the deployment branches stay current. This is part of the regular weekly maintenance.

## Building

After cloning `lupin`, rebuild the workspace so the new packages are
indexed alongside the vendor stack:

```bash
cd ~/ros2_ws
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
source install/setup.bash
```

Add the `source` line to `~/.bashrc` to avoid repeating it every shell.

## Running

Top-level launch lives in `lupin_bringup`. The exact launch file depends on which branch you have checked out:

```bash
# On the `sim` branch:
ros2 launch lupin_bringup sim.launch.py

# On the `hardware` branch:
ros2 launch lupin_bringup hardware.launch.py
```

(Launch files will be added by the team in subsequent MRs.)

## Repository layout

| Package | Purpose |
| --- | --- |
| `lupin_bringup` | Top-level launch files, parameters, system glue |
| `lupin_navigation` | AprilTag-based localisation, path planning |
| `lupin_perception` | Flower detection, vision pipelines |
| `lupin_hmi` | Remote operation interface |
| `lupin_msgs` | Custom messages, services, actions |
| `docs/` | Architecture diagrams, design notes |

## Contributing

- All three long-lived branches (`main`, `sim`, `hardware`) are **protected** — direct pushes are blocked. All changes go through MRs.
- Branch naming:
  - `feat/<topic>`, `fix/<topic>`, `docs/<topic>`, `chore/<topic>` for branches targeting `main`.
  - `sim/<topic>` for branches targeting `sim`.
  - `hw/<topic>` for branches targeting `hardware`.
- Every MR needs **one approving review** from a teammate (this is the *buddy check* the course grades).
- Commit messages follow [Conventional Commits](https://www.conventionalcommits.org/),
  e.g. `feat(navigation): add AprilTag pose correction`.
- Commit under your **own** GitLab account — commit distribution is graded.

## Team

| Name | GitLab | Role |
| --- | --- | --- |
| Koen Vogels | @kvogels | Perception, Control Interface |
| Lapo Veca | @lapoveca | Navigation, Manipulation |
| Lievijn Simons | @lwmssimons | Hardware Integration, Logic |
| Oscar Devos | @odevos | Navigation, Manipulation |
| Tejas Stanley | @tstanley | Perception |
| Tibbe Wouters | @twouters | Task Logic, Control Interface |

## Course staff

- Chris Pek — responsible lecturer, main project lead
- Carlos Hernandez Corbato — systems architecture
- Martin Klomp / Arend-Jan van Hilten — MIRTE Master creators & developers
- Martijn Wisse — robot expert
- Thijs Hoedemakers — lecturer
- Gillian Saunders — course support, skills & reflection
