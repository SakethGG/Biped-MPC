# Catalogs

Flat, hand-curated hardware catalogs consumed by `actuator.py`. `motors.json`
and `actuators.json` share a single **combined** `"index"` field (0-based),
assigned across both files together by increasing torque density -- see the
table below. `index` is a readability/cross-referencing label only;
`actuator.py`'s loaders never read it, and `DesignSpec.act_*` fields in
`variable_params.json` should reference actuators by **name**, not index,
since index is just rank position and shifts whenever either file is edited.

This README ignores `catalogs/compact/` (the bundled COMPAct motor x
gearbox x ratio artifacts) -- those are external published data, not
hand-curated, and out of scope here.

## Combined motor + actuator torque-density ranking

Two different quantities are being compared here, so treat this as a
bookkeeping ranking, not a literal apples-to-apples comparison:

- **Motors** (`motors.json`) are bare, pre-gearbox parts -- torque density
  uses the motor-side peak torque, `(Kt * Imax) / mass` (N*m/kg), since
  motors have no `peak_torque` field.
- **Actuators** (`actuators.json`) are complete joint-side, post-gearbox
  assemblies -- torque density uses `peak_torque / mass` (N*m/kg).

A gearbox multiplies torque faster than it adds mass, so every motor in this
catalog lands below every actuator -- indices 0-5 are all motors, 6-16 are
all actuators. That split is a property of the current data, not a rule.

| index | type | name | torque density (N*m/kg) | peak/motor torque source | mass (kg) | diameter (m) | length (m) | cost ($) |
|---|---|---|---|---|---|---|---|---|
| 0 | Motor | Maxon-EC-i-52 | 1.09 | Kt=0.068, Imax=12.0 | 0.75 | 0.052 | 0.055 | 480 |
| 1 | Motor | M-mini | 4.91 | Kt=0.060, Imax=18.0 | 0.22 | 0.046 | 0.024 | 180 |
| 2 | Motor | M-mid | 6.64 | Kt=0.085, Imax=25.0 | 0.32 | 0.060 | 0.028 | 240 |
| 3 | Motor | TMotor-U10II | 7.49 | Kt=0.096, Imax=32.4 | 0.415 | 0.092 | 0.0345 | 340 |
| 4 | Motor | M-big | 7.56 | Kt=0.110, Imax=33.0 | 0.48 | 0.087 | 0.034 | 360 |
| 5 | Motor | TMotor-U8-Lite | 7.92 | Kt=0.0637, Imax=29.7 | 0.239 | 0.0868 | 0.0265 | 230 |
| 6 | Actuator | ODrive-D5065-DD | 12.66 | peak_torque=5.0 | 0.395 | 0.050 | 0.065 | 320 |
| 7 | Actuator | TMotor-AK80-6 | 24.74 | peak_torque=12.0 | 0.485 | 0.098 | 0.0385 | 510 |
| 8 | Actuator | MAB-MD80 | 24.74 | peak_torque=12.0 | 0.485 | 0.098 | 0.0385 | 480 |
| 9 | Actuator | ANYdrive-3 | 36.36 | peak_torque=40.0 | 1.10 | 0.090 | 0.095 | 4000 |
| 10 | Actuator | TMotor-AK80-9 | 45.36 | peak_torque=22.0 | 0.485 | 0.098 | 0.0385 | 530 |
| 11 | Actuator | Bota-Roboticer-50 | 52.63 | peak_torque=50.0 | 0.95 | 0.098 | 0.0588 | 1800 |
| 12 | Actuator | Unitree-A1 | 55.37 | peak_torque=33.5 | 0.605 | 0.095 | 0.045 | 600 |
| 13 | Actuator | TMotor-AK10-9 | 56.38 | peak_torque=53.0 | 0.940 | 0.098 | 0.0617 | 720 |
| 14 | Actuator | MIT-Cheetah-3 | 68.00 | peak_torque=34.0 | 0.50 | 0.096 | 0.040 | 700 |
| 15 | Actuator | HEBI-X8-16 | 77.55 | peak_torque=38.0 | 0.490 | 0.073 | 0.045 | 2900 |
| 16 | Actuator | Unitree-B1 | 107.69 | peak_torque=210.0 | 1.95 | 0.107 | 0.074 | 1500 |

Ties (indices 7/8, AK80-6 / MAB-MD80, both 24.74 N*m/kg) keep their original
file order -- there's no other basis to break them.

## Re-sorting after an edit

If you add/remove/edit an entry in either file, recompute the combined
ranking above by hand (merge both files' torque densities, sort ascending),
reassign every `"index"` across *both* JSON files to match, and update this
table. Nothing in `actuator.py` enforces this automatically.
