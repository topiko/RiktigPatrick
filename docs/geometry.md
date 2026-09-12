# CAD-derived simulation geometry and camera frames

The active dimensions and forward kinematics are in
`src/riktigpatric/geometry.py`. They come from the current assembly in
`archive/scad/build.scad`, which uses `usedims.scad`, `frame.scad`, `head.scad`
and the camera proxy in `electronics.scad`.

The simulator uses convex tapered shell proxies. Rounded corners, mounting holes,
gearing and electronics are not meshed individually. OpenSCAD is not required at
runtime; dimensions are explicitly converted to SI in the shared module.

## Dimensions and reference points

| Quantity | Value | Source/interpretation |
|----------|-------|-----------------------|
| Neck-pitch pivot CAD height | 193 mm | `Hframe` |
| Body bottom width | 100 mm | `Wbottom` |
| Body side taper | 4.5° | `alpha`; width at neck-height reference ≈69.6 mm |
| Body depth construction parameter | 55 mm | `Tframe`; not a bounding-box dimension |
| Motor mounting reference | CAD Y=9 mm, Z=23 mm | `frame.scad: motors()` |
| Wheel diameter | 100 mm | User-confirmed approximation, independent of body depth |
| Wheel width / mounting gap | 20 mm / 1 mm | Retained approximations |
| Head height / depth | 115 mm / 42 mm | `Hhead`, `Lhead` |
| Head side taper | 4.5° | Same `edgex()` profile as body |
| Head front-face slope | 6.9° | `headtheta`, a shell-shape parameter |
| Neck-to-head lift | 11.4 mm | `Rtop + Hneck` |
| Camera proxy diameter / length | 32 mm / 35 mm | `electronics.scad: camera()` |
| Camera mounting-hole pattern | 30 × 30 mm | Not used as collision geometry |

The body proxy follows a simplified forward/height outline of the CAD hull, rather
than using a 193×100×55 mm box. Motors and wheel axes follow the 4.5° side taper.
Both positive wheel joint velocities still correspond to forward travel. Initial
spawn height accounts for wheel radius, width and camber so the tires meet the floor.

## Coordinate conventions

Robot coordinates are +X forward, +Y left, +Z up. The body origin is the nominal
motor-mount midpoint. For a point in the assembled CAD body frame (millimeters):

```text
robot_x = (cad_y - 9) * 0.001
robot_y = -cad_x * 0.001
robot_z = (cad_z - 23) * 0.001
```

Thus the neck-pitch pivot in body coordinates is `(-0.009, 0, 0.170) m`.
The head origin is `(0, 0, 0.0114) m` above that pivot in the pitched-neck frame.

At neutral, the camera cylinder in head coordinates runs from
`(-0.0215, 0, 0.090)` to `(0.0135, 0, 0.090) m`. The camera site and optical
viewpoint are at the forward tip. The optical axis points along head +X.
The camera's neutral mounting pitch is **zero**; the 6.9° shell slope is not an
optical-axis offset.

MuJoCo cameras look down local -Z, with +Y up. `CAMERA_MOUNT_ROTATION` converts
that camera convention to the robot/head convention. The viewer's 60° vertical
field of view is an approximation, not a calibration of the real lens.

## Head kinematics and limits

The head chain is explicit: torso → neck pitch → head yaw → fixed camera mount.

- Neck pitch rotates about robot/neck **-Y**, positive looking up.
- Head yaw rotates about the pitched neck's local **+Z**, positive turning left.
- Body pitch uses the existing quaternion/Euler convention: positive about +Y,
  nose down. It therefore has the opposite positive direction to neck pitch.
- Pitch limits: **−28° to +50°**. Neck-yaw limits: **−40° to +40°**.
- Joint bounds and integrated actuator-position setpoints use those ranges.
  MuJoCo's soft joint-limit dynamics can allow small transient deviations.

The shared camera calculation is:

```text
R_world_camera = R_world_body_estimate
                 @ R_y(-neck_pitch)
                 @ R_z(neck_yaw)
                 @ R_head_camera

forward = -R_world_camera[:, 2]
camera_elevation = atan2(forward.z, hypot(forward.x, forward.y))
```

Camera elevation is positive above the world horizon. This computes a measurement,
not a control command. For planar straight-ahead motion, positive neck pitch
counters positive nose-down body pitch. With roll or neck yaw, the full rotation
chain handles the coupling; adding Euler pitch angles is insufficient in general.

The NN receives the estimated elevation. A head-mounted frame-quaternion sensor
provides MuJoCo truth for training reward and diagnostics, not a policy input.
Yaw is a bounded neck-local joint target, not an absolute compass heading. There
is no independent camera-roll stabilization objective.

## Dynamics assumptions

The body mass remains 0.4 kg. The existing 0.2 kg combined head/neck budget is
split as 5% neck and 95% head so both articulated bodies have positive inertia.
Each wheel is 0.02 kg. Camera and wheel-marker geometry have zero mass and no
contact contribution; the camera mass is represented by the combined head budget.

Total nominal model mass is 0.64 kg. Inertias follow the simplified mass-carrying
shapes. Loaded mass distribution, exact IMU location, motor/servo response and
wheel width still require hardware measurement. The head speed limit defaults to
1 rad/s and is configurable separately from joint-angle limits.

These changes affect dynamics, the neck-pitch sign, policy inputs and action bins.
Train a fresh policy for this model rather than treating old checkpoints as equivalent.
