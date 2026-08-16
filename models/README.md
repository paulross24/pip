# PiP simulator model

`pip.urdf` is an exact, byte-for-byte copy of the approved stationary PiP
model from `C:\Users\paul_\Documents\virtual-pip\pip.urdf`. Its SHA-256 is
`920EF8F9045726FC3EE0E7919EEC029F9FC1D9F79812FD9A64E408C82BE51CE1`.

The model is deliberately copied rather than simplified: it retains the
measured `0.314215685 kg` body mass, `0.022443978 kg` upper-leg masses,
`0.017955182 kg` lower-leg masses, their inertia tensors, collision and visual
geometry, eight leg joint names, and the physical geometry used for the turn
experiment. The body is `70 x 120 x 30` mm and the leg solver uses the URDF's
40 mm upper and 80 mm lower link lengths. Its factory feet are front
`(-15, 95)` mm and rear `(5, 90)` mm in the planar `(x, down)` convention; the
selected branch has negative knee angles, matching the URDF joint axes.

The simulator attitude baseline uses `ROLL_ZERO = -2.397` degrees. It is a
calibration offset, not an attempt to rewrite the model pose.

Rejected alternatives:

- A hand-authored "lighter" URDF: it would replace measured mass and inertia.
- A geometry-only proxy: it would omit contact geometry needed by PyBullet.
- The positive-knee IK branch: it bends opposite to the deployed joint
  convention.
