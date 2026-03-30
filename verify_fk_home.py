#!/usr/bin/env python3
"""Quick FK sanity check (same model as lerobot.teleoperators.axe_leader.fk)."""
import sys
from pathlib import Path

import numpy as np

# Repo root: .../axe3_teleoperator
ROOT = Path(__file__).resolve().parent
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from lerobot.teleoperators.axe_leader.fk import forward_kinematics  # noqa: E402

LINKS = (0.060, 0.210, 0.250)
left_q = np.radians([-90, -0.6, 113.5])
right_q = np.radians([-90, -140.0, -35.0])

print("LEFT:")
_, chain = forward_kinematics(left_q, LINKS)
for i, p in enumerate(chain):
    print(f"  p{i}={np.round(p, 3)}")

print("\nRIGHT (planar_mirror + π/2 elbow offset, bi_axe right arm):")
_, chain = forward_kinematics(
    right_q, LINKS, planar_mirror=True, planar_mirror_elbow_offset_rad=-np.pi / 2.0
)
for i, p in enumerate(chain):
    print(f"  p{i}={np.round(p, 3)}")
eef = chain[-1]
print(f"  EEF={np.round(eef, 3)}")
v = chain[2] - chain[1]
print(f"  v(shoulder->elbow)={np.round(v, 3)}")
v = chain[3] - chain[2]
print(f"  v(elbow->EEF)={np.round(v, 3)}")
