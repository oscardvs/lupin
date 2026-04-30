# lupin_bringup

Top-level launch files and system glue for Team Lupin. Composes the
vendor MIRTE Master + Gazebo stack into a single launch per scenario,
so feature MRs in this repo have one stable entry point instead of
chasing vendor file names.

## Launches

| File | Purpose |
| --- | --- |
| `launch/sim.launch.py` | Generic sim entry point. Wraps `mirte_gazebo`'s empty / navigation launches; `nav:=true` brings up Nav2 + RViz against the KRR small-house world. |
| `launch/greenhouse_sim.launch.py` | Greenhouse-world entry point. Loads the SDF emitted by `scripts/generate_greenhouse_world.py` (coordinate frame matches `mdp-greenhouse`'s `tag_locations.json`), spawns the MIRTE Master in the south aisle, and starts ros2_control + twist_mux. Sets the Gazebo classic env vars from the launch so no `/usr/share/gazebo/setup.sh` source is needed. |

Each launch has a thorough docstring header — `python -c "open('.../*.launch.py').read()"`
or just open the file. Launch args are listed there with rationale.

## Worlds and scripts

- `worlds/greenhouse.world` — committed, deterministic output of
  `scripts/generate_greenhouse_world.py`. Regenerate after a
  `mdp-greenhouse` upgrade:
  ```bash
  python3 scripts/generate_greenhouse_world.py
  colcon build --symlink-install --packages-select lupin_bringup
  ```
  By default the script reads `tag_locations.json` from the installed
  `greenhouse_sim` Python package via `importlib.resources`; pass
  `--input` to point it elsewhere.

- AprilTag visuals in the generated world are bright magenta
  `_PLACEHOLDER` stand-ins until real `tag36h11` textures land. See
  the TODO inside `_render_tag` and the heads-up note in the top-level
  README's "Greenhouse Gazebo world" section.

## Where to start

For a full system bring-up workflow (sim + Nav2 + slam_toolbox + the
sensor bridge), see the **Running** section of the [top-level
README](../README.md#running). It covers the multi-terminal sequence,
known sim limitations (P3D teleport plugin, BEST_EFFORT QoS, KRR house
spawn time), and the pragmatic v1 boundaries.

## Maintainer

Oscar Devos — `o.a.e.devos@student.tudelft.nl`
