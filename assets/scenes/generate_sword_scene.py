#!/usr/bin/env python3
"""Generate assets/scenes/humanoid_sword.xml = stock humanoid.xml + a light training sword in the right hand + a hanging bag.
Usage: python3 assets/scenes/generate_sword_scene.py <repo root> [grip tilt deg, default 25]"""
import math, sys
repo = sys.argv[1]
src = open(f"{repo}/assets/scenes/humanoid.xml").read()

def norm(v):
    n = math.sqrt(sum(x * x for x in v)); return [x / n for x in v]
def cross(a, b):
    return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]
f = norm([1, 1, 1])                 # right forearm direction in the hand frame (lower_arm_right fromto)
e2 = norm([-1, -1, 2])              # "thumb up" direction, perpendicular to the forearm
tilt = math.radians(float(sys.argv[2]) if len(sys.argv) > 2 else 25)   # blade angle above the forearm axis
d = norm([math.cos(tilt) * f[i] + math.sin(tilt) * e2[i] for i in range(3)])   # blade direction
w = norm([e2[i] - sum(e2[k] * d[k] for k in range(3)) * d[i] for i in range(3)])   # edge direction: thumb-up axis made perpendicular to the blade
t = cross(d, w)                     # flat-face normal
fmt = lambda v: " ".join(f"{x:.5f}" for x in v)
at = lambda s: fmt([s * x for x in d])

sword = f"""
            <!-- training sword: grip through the fist, blade {math.degrees(tilt):.0f} deg above the forearm (neutral wrist) -->
            <geom name="sword_grip"   class="sword" type="capsule" fromto="{at(-0.09)} {at(0.05)}" size=".016" material="grip" mass=".06"/>
            <geom name="sword_pommel" class="sword" type="sphere" pos="{at(-0.10)}" size=".022" material="guard" mass=".03"/>
            <geom name="sword_guard"  class="sword" type="box" pos="{at(0.065)}" xyaxes="{fmt(w)} {fmt(t)}" size=".075 .014 .012" material="guard" mass=".05"/>
            <geom name="sword_blade"  class="sword" type="box" pos="{at(0.08 + 0.40)}" xyaxes="{fmt(w)} {fmt(t)}" size=".022 .004 .40" material="blade" mass=".16"/>
            <site name="sword_tip" pos="{at(0.88)}" size=".01" rgba="1 1 1 0"/>"""

bag = """
    <!-- hanging practice bag: collides only with the sword (the robot can walk through it) -->
    <body name="bag_anchor" pos=".85 -.15 2.0">
      <geom name="bag_beam" type="cylinder" fromto="0 -.25 0 0 .25 0" size=".025" material="rope" contype="0" conaffinity="0"/>
      <body name="bag" pos="0 0 0">
        <joint name="bag_swing" type="ball" damping=".6"/>
        <geom name="bag_rope" type="capsule" fromto="0 0 0 0 0 -.62" size=".008" material="rope" contype="0" conaffinity="0" mass=".05"/>
        <geom name="bag_body" type="capsule" fromto="0 0 -.75 0 0 -1.25" size=".11" material="bag" contype="2" conaffinity="2" condim="3" mass="1.2" solref=".03 1"/>
      </body>
    </body>
"""
assets = """    <material name="blade" rgba=".82 .85 .9 1" specular="1" shininess="1" reflectance=".25"/>
    <material name="guard" rgba=".75 .58 .2 1" specular=".8" shininess=".7"/>
    <material name="grip"  rgba=".25 .14 .08 1"/>
    <material name="bag"   rgba=".55 .12 .1 1"/>
    <material name="rope"  rgba=".6 .5 .35 1"/>
  </asset>"""
defaults = """  <default>
    <default class="sword">
      <!-- contype/conaffinity 2: never touches the robot or the floor (both 1), only the bag -->
      <geom contype="2" conaffinity="2" condim="3" group="1" friction=".5"/>
    </default>"""

hand = '<geom name="hand_right" zaxis="1 1 1" class="hand"/>'
assert src.count(hand) == 1 and src.count("</asset>") == 1 and src.count("  <default>\n") == 1 and src.count("</worldbody>") == 1
out = src.replace(hand, hand + sword).replace("  </asset>", assets, 1).replace("  <default>\n", defaults + "\n", 1).replace("</worldbody>", bag + "  </worldbody>", 1)
out = out.replace('<mujoco model="Humanoid">', '<mujoco model="Humanoid with sword">', 1)
out = "<!-- Generated from humanoid.xml: same robot + a 0.3 kg training sword in the right hand + a hanging practice bag. -->\n" + out
open(f"{repo}/assets/scenes/humanoid_sword.xml", "w").write(out)
print("blade dir (hand frame):", fmt(d))
