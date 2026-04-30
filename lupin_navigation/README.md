# lupin_navigation

Nav2 + slam_toolbox bringup for the MIRTE Master in sim (and, eventually,
hardware). Planned home for AprilTag-based pose corrections once the
detector pipeline lands.

## Layout

| Path | Contents |
| --- | --- |
| `launch/nav2.launch.py` | Full Nav2 lifecycle stack — controller (MPPI/Omni), planner, behaviour, BT navigator, waypoint follower, velocity smoother, plus map_server + AMCL by default. |
| `config/nav2_params.yaml` | Single source of truth for every Nav2 lifecycle node. Comments inline cite the rationale (BEST_EFFORT QoS, `base_link` not `base_footprint`, `min_laser_range: 0.25`, MPPI `motion_model: Omni`, `odom_topic: /mirte_base_controller/odom`, etc.). |
| `config/slam_toolbox_sim.yaml` | slam_toolbox config tuned for the sim (filters body-hit laser returns; `base_frame: base_link`). |
| `maps/krr_house.{yaml,pgm}` | Saved map of the AWS small-house world used as Nav2's default. Built with slam_toolbox; resolution 0.05 m. |

## Two launch modes

```bash
# Default — AMCL localisation against a saved map (KRR house by default)
ros2 launch lupin_navigation nav2.launch.py

# SLAM mode — drops map_server / AMCL / lifecycle_manager_localization
# so an external slam_toolbox node can own /map and the map→odom TF.
# Use this in worlds without a saved map (e.g. the greenhouse).
ros2 launch lupin_navigation nav2.launch.py slam:=true
```

`slam:=true` requires a separate `slam_toolbox` node:

```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true \
  slam_params_file:=$(ros2 pkg prefix lupin_navigation)/share/lupin_navigation/config/slam_toolbox_sim.yaml
```

Other launch args:

| Arg | Default | Meaning |
| --- | --- | --- |
| `use_sim_time` | `true` | Use `/clock` from Gazebo. Set `false` for hardware. |
| `autostart` | `true` | Lifecycle managers auto-activate the configured nodes. |
| `map` | `<pkg>/maps/krr_house.yaml` | Map handed to `map_server` in non-slam mode. |
| `params_file` | `<pkg>/config/nav2_params.yaml` | Nav2 params yaml. Override to fork tuning per-branch. |
| `slam` | `false` | If `true`, skip the localisation half of the stack. |

## Why MPPI, not DWB

`controller_server` uses `nav2_mppi_controller::MPPIController` with
`motion_model: Omni` so the holonomic mecanum base actually samples
lateral (`vy`) commands. Going straight to MPPI avoids the DWB-tuning
detour and keeps a single controller story for the MDP report. See the
top of `config/nav2_params.yaml` for the full critic stack and the
sim-specific tunings (lower `PreferForwardCritic.cost_weight`,
`vy_min/max: ±0.5`, accel envelope matched to the velocity smoother).

## Wiring into the cmd_vel chain

`controller_server` → `velocity_smoother` (smoothed Twist remapped to
`/cmd_vel_auto`) → `lupin_hmi/cmd_vel_mux` → `/cmd_vel`. Manual
joystick on `/cmd_vel_manual` always wins via the mux's takeover
logic; Nav2 hits the same chain manual teleop hits, so behaviour is
identical between teleop and autonomy.

**Do not publish to `/cmd_vel` directly from a Nav2 node** — that
bypasses the mux and feeds straight into the `gazebo_planar_move`
teleport plugin in sim (see the top-level README's "Sim — known
limitations").

## Sim caveat

The vendor URDF's `gazebo_planar_move` (P3D) plugin teleports the body
on `/cmd_vel`, so paths plan and the robot follows them visually but
the mecanum wheels don't actually drive. Wheel-encoder odom
(`/mirte_base_controller/odom`) lags true motion. Fine for
planner/costmap testing; not for tuning controller dynamics. The full
fix needs URDF surgery (deferred MR) — see `project_p3d_urdf_limitation`.

## Where to start

For the full multi-terminal workflow (sim + Nav2 + slam_toolbox + RViz
goal-sending), see the **Running** section of the [top-level
README](../README.md#running).

## Maintainer

Oscar Devos — `o.a.e.devos@student.tudelft.nl`
