"""Parametric CAD of the 4-DOF feeding arm (units: mm). Arm drawn at q = [0,0,0,0]:
fully extended along +X.  Joint axes: base yaw = +Z at origin; shoulder/elbow/wrist = parallel to Y."""
import cadquery as cq

L0, L1, L2, L3 = 120.0, 320.0, 300.0, 140.0       # must match feeding_robot_sim.py (x1000)
Z = L0                                             # height of the pitch-axis line

def link(x0, x1, thick, h, bore=12.0, y_off=0.0):
    """Stadium-shaped plate in the XZ plane between two joint axes with bores at each end."""
    L = x1 - x0
    p = (cq.Workplane("XZ").center((x0 + x1) / 2, Z).slot2D(L + h, h, 0)
         .extrude(thick / 2, both=True))
    for x in (x0, x1):
        p = p.cut(cq.Workplane("XZ").center(x, Z).circle(bore / 2).extrude(thick, both=True))
    return p.translate((0, y_off, 0))

def hub(x, r, length, bore=12.0, y_off=0.0):
    """Cylindrical joint hub with its axis along Y."""
    h = cq.Workplane("XZ").center(x, Z).circle(r).extrude(length / 2, both=True)
    h = h.cut(cq.Workplane("XZ").center(x, Z).circle(bore / 2).extrude(length, both=True))
    return h.translate((0, y_off, 0))

parts = {}
xw = L1 + L2                                               # wrist axis x
def pin(x, length=64, d=12):
    return cq.Workplane("XZ").center(x, Z).circle(d / 2).extrude(length / 2, both=True)
# --- base plate + yaw column (yaw axis = Z) ---
parts["01_base_plate"] = cq.Workplane("XY").circle(90).extrude(16).faces(">Z").workplane() \
    .polarArray(72, 0, 360, 4).circle(4.5).cutThruAll()
parts["02_yaw_column"] = cq.Workplane("XY").workplane(offset=16).circle(48).extrude(L0 - 16 - 40)
# --- shoulder fork (rides on the yaw column) ---
fork = cq.Workplane("XY").workplane(offset=L0 - 40).circle(50).extrude(40)
for s_ in (-1, 1):
    fork = fork.union(cq.Workplane("XY").box(60, 12, 80).translate((0, s_ * 44, L0)))
fork = fork.cut(cq.Workplane("XY").box(100, 56, 60).translate((15, 0, L0 - 20)))   # slot so the arm can dip below horizontal
fork = fork.cut(cq.Workplane("XZ").center(0, Z).circle(6).extrude(200, both=True))
parts["03_shoulder_fork"] = fork
parts["04_shoulder_motor_PLACEHOLDER"] = cq.Workplane("XY").box(56, 56, 56).translate((0, 50 + 28, Z))
# --- upper arm: TWO parallel plates (y = +-20) so the forearm can sit between them ---
ua = link(0, L1, 12, 44, y_off=20).union(link(0, L1, 12, 44, y_off=-20))
ua = ua.union(cq.Workplane("XY").box(24, 52, 30).translate((L1 / 2, 0, Z)))      # cross-tie
parts["05_upper_arm"] = ua
# --- forearm: single slim plate between the upper-arm plates ---
parts["06_forearm"] = link(L1, xw, 22, 36)
# --- wrist clevis block: forearm end sits in its slot ---
wr = cq.Workplane("XY").box(54, 44, 28).translate((xw + 9, 0, Z))
wr = wr.cut(cq.Workplane("XY").box(40, 25, 60).translate((xw, 0, Z)))
wr = wr.cut(cq.Workplane("XZ").center(xw, Z).circle(6).extrude(100, both=True))
parts["07_wrist_block"] = wr
# --- spoon (tip at x = L1+L2+L3) ---
x_h = xw + 36
handle = cq.Workplane("XY").box(L3 - 36 - 30 + 6, 10, 6).translate((x_h + (L3 - 60 + 6) / 2, 0, Z))
bowl = cq.Workplane("XY").workplane(offset=Z - 2).center(xw + L3 - 30, 0).ellipse(32, 20).extrude(4)
dish = cq.Workplane("XY").sphere(60).translate((xw + L3 - 30, 0, Z + 58))
parts["08_spoon"] = handle.union(bowl).cut(dish)
# --- pins (stand-ins for the joint shafts / bearings) ---
parts["09_pin_shoulder"] = pin(0, 100)
parts["10_pin_elbow"] = pin(L1, 64)
parts["11_pin_wrist"] = pin(xw, 52)

colors = {"01": (.22, .28, .31), "02": (.35, .42, .46), "03": (.35, .42, .46), "04": (.1, .1, .1),
          "05": (.12, .53, .9), "06": (.98, .55, 0), "07": (.3, .3, .3), "08": (.8, .85, .87), "09": (.7, .7, .7), "10": (.7, .7, .7), "11": (.7, .7, .7)}
asm = cq.Assembly(name="feeding_arm_4dof")
for name, wp in parts.items():
    asm.add(wp, name=name, color=cq.Color(*colors[name[:2]], 1.0))
asm.save("../feeding_arm_cad/feeding_arm_4dof_assembly.step")

# per-part STL + STEP, and one merged STL
import os
os.makedirs("../feeding_arm_cad/parts", exist_ok=True)
for name, wp in parts.items():
    cq.exporters.export(wp, f"../feeding_arm_cad/parts/{name}.step")
    cq.exporters.export(wp, f"../feeding_arm_cad/parts/{name}.stl", tolerance=0.1, angularTolerance=0.1)
comp = cq.Compound.makeCompound([w.val() for w in parts.values()])
cq.exporters.export(comp, "../feeding_arm_cad/feeding_arm_4dof_all.stl", tolerance=0.1, angularTolerance=0.1)
print("parts:", len(parts))
for n, w in parts.items():
    bb = w.val().BoundingBox(); print(f"{n:34s} vol={w.val().Volume()/1e3:8.1f} cm3  x[{bb.xmin:6.0f},{bb.xmax:6.0f}] z[{bb.zmin:5.0f},{bb.zmax:5.0f}]")

# ---- interference check between every pair of parts ----
names = list(parts)
bad = []
for i in range(len(names)):
    for j in range(i + 1, len(names)):
        v = parts[names[i]].val().intersect(parts[names[j]].val()).Volume()
        if v > 5.0:
            bad.append((names[i], names[j], round(v, 1)))
print("INTERFERENCES (mm3 > 5):", bad if bad else "none")
