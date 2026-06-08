#!/usr/bin/env bash
# install-clock-sync.sh — install the Lupin boot-time clock sync on the Mirte.
#
# WHY
#   The Orange Pi has no RTC and no NTP over the cable, so it boots with a stale
#   clock (seconds-to-hours behind the laptop). Nav2 goals are stamped at
#   laptop-now; the robot's TF carries the robot clock; tf2 won't extrapolate a
#   now-stamped goal into the future past the robot's TF stamps, so the planner
#   fails with "Extrapolation Error … Could not transform the start or goal pose"
#   and the robot never moves. (Nav2 still reaches "active" — only *sending a
#   goal* exposes it.) Verified live 2026-06-07.
#
# WHAT THIS INSTALLS (robot side)
#   • /usr/local/sbin/lupin-clock-sync.sh  — steps the clock from the laptop's
#     NTP server using the robot's already-running chronyd.
#   • /etc/systemd/system/lupin-clock-sync.service — oneshot ordered
#     After=network-online.target chrony.service, Before=mirte-ros.service, so
#     the step lands BEFORE ros2_control starts (we never date-step a LIVE stack,
#     which is what wedges the controllers).
#   • /etc/chrony/conf.d/lupin-laptop.conf — adds the laptop as a chrony source
#     (best-effort; the script also adds it at runtime).
#
# MITIGATIONS (all the ones we agreed on)
#   • BOUNDED  — gives up after LUPIN_CLOCK_BUDGET s (+ systemd TimeoutStartSec);
#                never hangs boot.
#   • SOFT     — always exits 0. Laptop NTP unreachable ⇒ robot proceeds on its
#                existing clock = today's behaviour (then run post-boot-sync.sh).
#   • BEFORE   — step happens before mirte-ros ⇒ no live-stack wedge.
#   • NO mid-session auto-step — the installer DISABLES the stock `makestep 1 3`
#                in /etc/chrony/chrony.conf (Ubuntu ships it). If the laptop NTP
#                is unreachable at boot, those "first 3 updates" land LATE — after
#                mirte-ros is up — and an 84-min step then wedges ros2_control.
#                With it disabled, chrony only *slews* during a session; the one
#                step we want is the oneshot's explicit `chronyc makestep` before
#                mirte-ros (which works regardless of the config directive).
#   • UTC      — NTP is UTC; no timezone games.
#   • KILL SWITCH — `sudo systemctl mask lupin-clock-sync.service` (then reboot),
#                or `sudo bash install-clock-sync.sh --uninstall`.
#
# LAPTOP SIDE (the NTP server the robot syncs from): for a durable setup put it
# on the HOST so it auto-starts on PC boot and survives container recreation:
#     sudo apt-get install -y chrony
#     printf 'allow 10.42.0.0/24\nlocal stratum 10\n' | sudo tee -a /etc/chrony/chrony.conf
#     sudo systemctl enable --now chrony && sudo systemctl restart chrony
#     ss -lnup | grep ':123'      # expect it listening
# (A chrony in the dev container works for testing but is wiped on host
#  reboot/container recreation — see the script comments.)
#
# RUN
#   On the robot:   sudo bash install-clock-sync.sh
#   From laptop:    ssh mirte@10.42.0.1 'sudo bash -s' < install-clock-sync.sh
#   Uninstall:      sudo bash install-clock-sync.sh --uninstall
#   Then ACTIVATE with a reboot (it deliberately does NOT step the clock now —
#   that must happen at boot, before mirte-ros):  sudo reboot
set -euo pipefail

LAPTOP_NTP="${LUPIN_LAPTOP_NTP:-10.42.0.2}"
SCRIPT_PATH=/usr/local/sbin/lupin-clock-sync.sh
UNIT_PATH=/etc/systemd/system/lupin-clock-sync.service
CHRONY_DROPIN=/etc/chrony/conf.d/lupin-laptop.conf

if [[ "${1:-}" == "--uninstall" ]]; then
  systemctl disable --now lupin-clock-sync.service 2>/dev/null || true
  systemctl unmask lupin-clock-sync.service 2>/dev/null || true
  rm -f "$SCRIPT_PATH" "$UNIT_PATH" "$CHRONY_DROPIN"
  [[ -f /etc/chrony/chrony.conf.lupin-bak ]] && mv -f /etc/chrony/chrony.conf.lupin-bak /etc/chrony/chrony.conf && echo "restored stock makestep in chrony.conf"
  systemctl daemon-reload
  systemctl try-restart chrony 2>/dev/null || true
  echo "lupin-clock-sync uninstalled. (Reboot to drop the boot ordering.)"
  exit 0
fi

[[ $EUID -eq 0 ]] || { echo "run as root: sudo bash install-clock-sync.sh"; exit 1; }
command -v chronyc >/dev/null || { echo "error: chronyd/chronyc not found on this robot"; exit 1; }

# ── 1) runtime sync script ──────────────────────────────────────────────────
install -m 0755 /dev/stdin "$SCRIPT_PATH" <<'SYNC_SH'
#!/usr/bin/env bash
# lupin-clock-sync.sh — step the robot clock from the laptop NTP BEFORE
# ros2_control starts. Installed by install-clock-sync.sh; see it for rationale.
# Bounded + soft + UTC; uses the already-running chronyd via chronyc.
set -u
LAPTOP="${LUPIN_LAPTOP_NTP:-10.42.0.2}"
BUDGET="${LUPIN_CLOCK_BUDGET:-25}"
log(){ logger -t lupin-clock-sync "$*"; echo "lupin-clock-sync: $*"; }

deadline=$(( $(date +%s) + BUDGET ))
# (a) bounded wait for the laptop NTP host to answer ICMP
until ping -c1 -W1 "$LAPTOP" >/dev/null 2>&1; do
  if (( $(date +%s) >= deadline )); then
    log "laptop $LAPTOP unreachable within ${BUDGET}s — leaving clock as-is (run post-boot-sync.sh later if needed)"
    exit 0
  fi
  sleep 1
done

before=$(date -u +%s)
# (b) make sure chrony is measuring the laptop, force a burst, then step ONCE.
#     `chronyc makestep` overrides config, so we don't need makestep enabled
#     persistently (which would risk a late, controller-wedging step).
chronyc add server "$LAPTOP" iburst >/dev/null 2>&1 || true   # idempotent-ish
chronyc online                      >/dev/null 2>&1 || true
chronyc 'burst 4/4'                 >/dev/null 2>&1 || true

# (b2) WAIT FOR A SOLID MEASUREMENT before stepping. Stepping after a single
# noisy sample applied a ~6 s error once (2026-06-08) that chrony then only
# slewed (never corrected). Poll until the laptop source has ≥3 reachability
# bits set (reach octal ≥ 7) or we burn the budget; only then makestep.
i=0
while (( $(date +%s) < deadline )); do
  reach=$(chronyc -n sources 2>/dev/null | awk -v ip="$LAPTOP" '$0 ~ ip {print $5; exit}')
  if [ -n "$reach" ] && [ "$reach" != "0" ] && (( 8#$reach >= 8#7 )) 2>/dev/null; then break; fi
  sleep 1; i=$((i+1)); (( i % 4 == 0 )) && chronyc 'burst 2/2' >/dev/null 2>&1
done
chronyc makestep >/dev/null 2>&1 || true

# (c) bounded wait until chrony is synchronised to TIGHT tolerance (≤0.1 s),
# so we don't proceed on a still-wrong clock.
if chronyc waitsync 15 0.1 0.0 1 >/dev/null 2>&1; then
  log "clock synced from $LAPTOP (was ~$(( before - $(date -u +%s) ))s off) → $(date -u)"
else
  off=$(chronyc tracking 2>/dev/null | awk -F'[ :]+' '/Last offset/{print $3}')
  log "waitsync DID NOT reach 0.1 s (last offset ${off:-?}) — proceeding; Nav2 goals may need post-boot-sync"
fi
exit 0
SYNC_SH
echo "wrote $SCRIPT_PATH"

# ── 1b) disable the stock `makestep` BEFORE the chrony restarts below, so a
# reachable laptop can never make chrony step a LIVE stack. Ubuntu ships
# `makestep 1 3` (step within the first 3 updates); if the laptop NTP is
# unreachable at boot those updates land late (after mirte-ros) and the step
# wedges ros2_control. The oneshot's explicit `chronyc makestep` is unaffected.
if grep -qE '^[[:space:]]*makestep' /etc/chrony/chrony.conf 2>/dev/null; then
  sed -i.lupin-bak 's/^\([[:space:]]*makestep\)/#\1/' /etc/chrony/chrony.conf
  echo "disabled stock 'makestep' in /etc/chrony/chrony.conf (backup: chrony.conf.lupin-bak)"
  systemctl try-restart chrony 2>/dev/null || true
else
  echo "note: no active 'makestep' in /etc/chrony/chrony.conf (already slew-only)"
fi

# ── 2) chrony source drop-in (best-effort; script also adds it at runtime) ──
if [[ -d /etc/chrony/conf.d ]] && grep -qiE '^(confdir|include)\b.*conf\.d' /etc/chrony/chrony.conf 2>/dev/null; then
  cat > "$CHRONY_DROPIN" <<EOF
# Lupin: wired laptop as a time source. NO 'makestep' here on purpose — the only
# step is lupin-clock-sync.service's explicit 'chronyc makestep' before mirte-ros;
# during a session chrony only slews, so it can never re-wedge ros2_control.
server ${LAPTOP_NTP} iburst
EOF
  echo "wrote $CHRONY_DROPIN"
  systemctl try-restart chrony 2>/dev/null || true
else
  echo "note: /etc/chrony/conf.d not active — skipping drop-in (runtime 'chronyc add server' still covers it)"
fi

# ── 3) systemd oneshot, ordered before mirte-ros ───────────────────────────
cat > "$UNIT_PATH" <<EOF
[Unit]
Description=Lupin: step clock from laptop NTP before ros2_control (Nav2 tf needs it)
Documentation=https://gitlab.tudelft.nl/cor/ro47007/2026/group_14/lupin
After=network-online.target chrony.service
Wants=network-online.target
Before=mirte-ros.service

[Service]
Type=oneshot
RemainAfterExit=yes
Environment=LUPIN_LAPTOP_NTP=${LAPTOP_NTP}
ExecStart=${SCRIPT_PATH}
TimeoutStartSec=40

[Install]
WantedBy=multi-user.target
EOF
echo "wrote $UNIT_PATH"

systemctl daemon-reload
systemctl enable lupin-clock-sync.service >/dev/null 2>&1
echo
echo "installed + ENABLED (laptop NTP = ${LAPTOP_NTP})."
echo "NOT started now — it steps the clock, which must happen at boot BEFORE"
echo "mirte-ros (stepping a live stack wedges the controllers)."
echo "Activate + test:   sudo reboot     (then check: journalctl -u lupin-clock-sync -b)"
echo "Kill switch:        sudo systemctl mask lupin-clock-sync.service && sudo reboot"
