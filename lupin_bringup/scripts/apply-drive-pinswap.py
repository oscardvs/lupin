#!/usr/bin/env python3
"""Lupin: matched motor(p1<->p2) + encoder(A<->B) pin-swap for Mirte-247264.
Cancels the physical 180-deg drive inversion at the source so odom == physical.
Runs ON THE ROBOT (edits the robot's telemetrix config). Backup + abort-if-not-unique.
Reversible: restore the printed .lupin-bak-* file and power-cycle."""
import shutil, time, sys

f = "/home/mirte/mirte_ws/src/mirte-ros-packages/mirte_bringup/telemetrix_config/mirte_master_config.yaml"
s = open(f).read()
swaps = [
    ("          p1: 17\n          p2: 16", "          p1: 16\n          p2: 17"),  # motor front_left
    ("          p1: 10\n          p2: 11", "          p1: 11\n          p2: 10"),  # motor rear_right
    ("          p1: 21\n          p2: 20", "          p1: 20\n          p2: 21"),  # motor rear_left
    ("          p1: 14\n          p2: 15", "          p1: 15\n          p2: 14"),  # motor front_right
    ("          A: 18\n          B: 19", "          A: 19\n          B: 18"),      # encoder front_left
    ("          A: 22\n          B: 26", "          A: 26\n          B: 22"),      # encoder rear_left
    ("          A: 13\n          B: 12", "          A: 12\n          B: 13"),      # encoder front_right
    ("          A: 9\n          B: 8", "          A: 8\n          B: 9"),          # encoder rear_right
]
bad = [(o, s.count(o)) for o, _ in swaps if s.count(o) != 1]
if bad:
    print("ABORT - pattern(s) not uniquely matched, NO changes written:", bad)
    sys.exit(1)
bak = f + ".lupin-bak-" + time.strftime("%Y%m%d-%H%M%S")
shutil.copy2(f, bak)
for o, new in swaps:
    s = s.replace(o, new)
open(f, "w").write(s)
print("OK: applied %d matched swaps. backup: %s" % (len(swaps), bak))
