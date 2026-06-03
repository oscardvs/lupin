# Demo-day bringup — reverting wired → AP (Lupin)

Undo the wired-ethernet network setup and put **Mirte-247264 back on its WiFi AP**
(`192.168.42.1`, `Mirte-247264`), so you can connect cableless and follow
`DEMO_DAY.md`. This is the exact inverse of `DEMO_DAY_WIRED.md §A.3` (which
disabled the AP for the cable). Scan-able — look it up under pressure and follow it.

> **What actually changes.** Only the **network**, in three places: (1) the robot
> re-enables its AP service and reboots single-homed on `wlan0`; (2) the ethernet
> cable comes out; (3) the laptop's DDS env is repointed from the wired IP back to
> the AP IP. Nothing about the ROS stack, launches, or the HMI changes — once
> you're on the AP, `DEMO_DAY.md` is the procedure.

| What | Before (wired) | After (AP) |
|---|---|---|
| Robot link | `eth0` `10.42.0.1` | `wlan0` AP `192.168.42.1` (`Mirte-247264`) |
| `mirte-ap` / `mirte-wifi-watchdog` | disabled | enabled |
| Ethernet cable | plugged | **unplugged** |
| Laptop DDS env | `10.42.0.1:11811` | `192.168.42.1:11811` |
| SSH alias | `ssh lupin-wired` | `ssh lupin` |

---

## 1. Robot — re-enable the AP and reboot

While the AP is down, the **cable is your only link** to the robot, so do this
over `lupin-wired` (`10.42.0.1`) *before* you unplug anything:

```bash
# re-arm the AP so it comes back at every boot (inverse of the wired `disable`)
ssh lupin-wired 'sudo systemctl enable mirte-ap mirte-wifi-watchdog'

# confirm the flip took (both should print: enabled)
ssh lupin-wired 'systemctl is-enabled mirte-ap mirte-wifi-watchdog'

# reboot — the ssh session drops as it goes down (expected)
ssh lupin-wired sudo reboot
```

**Expect:** `enable` prints `Created symlink …` lines, `is-enabled` reports
`enabled` for both, and `reboot` returns nothing as the connection closes.
(`sudo` is password-less for `mirte` on the robot — same as `post-boot-sync.sh`
relies on.)

> **Why a reboot, not just `enable --now`.** The robot's DDS participants
> (`mirte-ros` nodes + the discovery server) announce their locators from the
> interfaces present **at start**. They booted `eth0`-only for the cable; bringing
> `wlan0` up under them won't make them re-announce on it. A reboot is the clean
> way to get every participant advertising the AP locator — and `DEMO_DAY.md §0`
> wants a fresh boot anyway (let the ~90 s boot storm settle).

---

## 2. Pull the ethernet cable — REQUIRED, not optional

Unplug the cable **while the robot reboots** so it comes back **single-homed on
`wlan0`**.

> **Why this matters (the 2026-06-03 hard-blocker).** If the cable stays in, the
> robot comes back **multi-homed** — `eth0` `10.42.0.1` *and* AP `192.168.42.1` —
> and advertises **both** DDS locators. A laptop on the AP can't reach
> `10.42.0.1`, and DDS does **not** cleanly fall back: **laptop→robot data
> silently dies** (HMI shows `LIVE`, but you can't drive and
> `ros2 control list_controllers` hangs), while robot→laptop telemetry keeps
> flowing so nothing looks broken. This is the *same* multi-homing failure
> `DEMO_DAY_WIRED.md §6` describes, just mirrored. Single-homing on `wlan0` is the
> fix. (`project_wired_dds_robot_multihoming`.)

Leaving the robot's `lupin-wired` NM profile in place is fine — with no cable,
`eth0` has no carrier and the profile stays inactive (keeps the wired option for
next time). Your laptop's `enx…` adapter also drops `10.42.0.2` on unplug, so the
laptop won't be multi-homed either.

> **Cable already back in and you're multi-homed?** Either unplug + reboot again,
> or — reachable over the AP — drop `eth0` on the robot and reboot:
> `ssh lupin 'sudo nmcli connection down lupin-wired && sudo reboot'`.

---

## 3. Laptop — reconnect WiFi and repoint DDS

```bash
# 1) ~90 s after the reboot, connect WiFi to the "Mirte-247264" AP. Verify:
ip -br addr | grep wlp            # expect 192.168.42.x/24 (DEMO_DAY.md cites .66)

# 2) repoint the DDS env from the wired IP back to the AP IP
cd ~/ros2_ws/src/lupin/lupin_bringup
./scripts/setup-laptop-dds-env.sh 192.168.42.1   # rewrites ~/.config/lupin/ros-env.sh

# 3) open a FRESH terminal (so .bashrc re-sources), then sanity-check:
echo $ROS_DISCOVERY_SERVER        # must read 192.168.42.1:11811  (NOT 10.42.0.1)
source ~/ros2_ws/install/setup.bash
ros2 daemon start && sleep 5
ros2 topic list | wc -l           # expect 50+, not 2
~/.config/lupin/post-boot-sync.sh # already defaults to mirte@192.168.42.1
```

**If `topic list` returns 2:** `echo $ROS_DISCOVERY_SERVER` — if it's still
`10.42.0.1:11811`, you're in a stale shell (the repoint only affects new
terminals); open a fresh one or `source ~/.config/lupin/ros-env.sh`. If it's
correct but still 2, the AP isn't up yet (give it longer) or the laptop didn't
associate — re-check step 1.

> **Pin the AP so the laptop doesn't roam off it mid-demo** (known footgun —
> `project_wifi_roam_masquerades_as_dead_controllers`):
> ```bash
> nmcli connection modify Mirte-247264 connection.autoconnect-priority 100
> ```

---

## 4. Then follow DEMO_DAY.md

You're back to the cableless setup. Continue with **`DEMO_DAY.md`** from §1
(`ssh lupin` — the un-suffixed alias — is now your way in). The
`setup-laptop-dds-env.sh 192.168.42.1` you ran above is exactly that doc's
First-time-setup step 3, so its §3 onward just works.

---

## 5. Reference

- **Robot AP IP:** `192.168.42.1` (`wlan0`, `Mirte-247264`)
- **Laptop IP on AP:** `192.168.42.66/24` (DHCP from the robot)
- **SSH:** `ssh lupin` (alias) or `ssh mirte@192.168.42.1`
- **DDS env (laptop):** `setup-laptop-dds-env.sh 192.168.42.1` →
  `ROS_DISCOVERY_SERVER=192.168.42.1:11811`
- **Confirm the AP is live (after reboot):** the `Mirte-247264` SSID appears and
  the laptop gets a `192.168.42.x` lease (step 3.1). Over the AP:
  `ssh lupin 'systemctl is-active mirte-ap'` → `active`.
- **Wired (forward) guide:** `DEMO_DAY_WIRED.md` · **Cableless guide:**
  `DEMO_DAY.md` · **Sim:** `DEMO_DAY_SIM.md`
