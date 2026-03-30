#!/usr/bin/env python3
"""Test FK with negated q2 for right arm."""
import numpy as np

_FC = np.array([[0,0,-1],[1,0,0],[0,1,0]], float)

def _rot_y(a):
    c,s = np.cos(a),np.sin(a)
    return np.array([[c,0,s],[0,1,0],[-s,0,c]],float)

def _rot_z(a):
    c,s = np.cos(a),np.sin(a)
    return np.array([[c,-s,0],[s,c,0],[0,0,1]],float)

def fk(q, L, negate_lift=False):
    q1,q2,q3 = q
    if negate_lift:
        q2 = -q2
    l1,l2,l3 = L
    v1=np.array([l1,0,0]); v2=np.array([l2,0,0]); v3=np.array([l3,0,0])
    p0=np.zeros(3)
    p1l=v1
    p2l=p1l+_rot_y(q2)@v2
    p3l=p2l+_rot_y(q2+q3)@v3
    rz=_rot_z(q1)
    chain=np.vstack([p0,rz@p1l,rz@p2l,rz@p3l])
    chain=(_FC@chain.T).T
    return chain[-1],chain

LINKS=(0.060,0.210,0.250)
left_q  = np.radians([-90, -0.6, 113.5])
right_q = np.radians([-90, -140.0, -35.0])

print("LEFT (no change):")
eef,chain=fk(left_q,LINKS)
for i,p in enumerate(chain): print(f"  p{i}={np.round(p,3)}")

print("\nRIGHT (negated lift q2):")
eef,chain=fk(right_q,LINKS,negate_lift=True)
for i,p in enumerate(chain): print(f"  p{i}={np.round(p,3)}")
print(f"  EEF={np.round(eef,3)}")
v=chain[2]-chain[1]; print(f"  v(shoulder->elbow)={np.round(v,3)}")
v=chain[3]-chain[2]; print(f"  v(elbow->EEF)={np.round(v,3)}")
