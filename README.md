# KUKA Robot URDF Library (from KUKA.Sim 4.3)

A collection of **315 verified URDF packages** (**2,145 binary STL link meshes**) covering six KUKA.Sim 4.3 robot categories — **Cobots (3–15 kg)**, **Fenceless (4–24 kg)**, **Small Robots (3–10 kg)**, **Low Payloads (5–22 kg)**, **Medium Payloads (30–70 kg)** and **High Payloads (90–300 kg)** — along with the zero-dependency Python converter (`batch_vcmx_to_urdf.py`) used to generate and validate them.

| Category | Models | Robot families / kinematics |
| :--- | ---: | :--- |
| Cobots (3kg - 15kg) | 9 | LBR iisy (6-DOF), LBR iiwa 7 / 14 incl. CleanRoom (7-DOF redundant) |
| Fenceless (4kg - 24kg) | 36 | KR AGILUS / CYBERTECH / KR 16–20 AIRSKIN (6-DOF), 2 AIRSKIN safety flanges (0-DOF, fixed) |
| Small Robots (3kg - 10kg) | 70 | KR AGILUS-1/2/3 & KR 4 (6-DOF), KR-SCARA (4-DOF, prismatic A3), KR-DELTA (4-DOF parallel, modelled as serial chain) |
| Low Payloads (5kg - 22kg) | 73 | KR CYBERTECH / nano / ARC / -2 variants, KR 16 / KR 16L (6-DOF) |
| Medium Payloads (30kg - 70kg) | 43 | KR IONTEC / IONTEC-2, KR 30/60 (-3/-4 KS, L), KR JET (6-DOF) |
| High Payloads (90kg - 300kg) | 84 | KR QUANTEC (extra / nano / prime / pro / ultra / -2) (6-DOF), KR 80L 3/4/5-axis gantries (prismatic) |

---

## 1. Repository Layout

Every payload category, product series, and robot variant has its own self-contained directory containing `<robot_slug>.urdf` and `meshes/visual/*.stl`:

```text
kuka_urdf/
├── Cobots (3kg - 15kg)/
│   ├── LBR iisy (3kg - 15kg)/       # 5 models  (LBR iisy 3 / 6 / 8 / 11 / 15)
│   └── LBR iiwa (7kg - 14kg)/       # 4 models  (LBR iiwa 7 R800, 14 R820, + CleanRoom/)
├── Fenceless (4kg - 24kg)/          # 26 models (AGILUS / CYBERTECH AIRSKIN, safety flanges)
│   └── Archiv/                      # 10 models (KR 16 / 20 AIRSKIN legacy)
├── Small Robots (3kg - 10kg)/
│   ├── AGILUS KR4/                  # 1 model
│   ├── AGILUS-2 Series/             # 19 models (incl. HM_SC/)
│   ├── AGILUS-3 Series/             # 6 models
│   ├── Archiv/                      # 24 models (AGILUS-1 fivve / sixx, KR-SCARA)
│   ├── KR-DELTA/                    # 2 models  (4-DOF delta)
│   ├── KR-SCARA_2/                  # 2 models  (4-DOF SCARA)
│   └── SCARA 12kg/                  # 16 models (Standard / CleanRoom / HO_Variant)
├── Low Payloads (5kg - 22kg)/
│   ├── Archiv/                      # 42 models (KR 16, KR 16L, CYBERTECH-1 / nano / ARC)
│   ├── KR CYBERTECH nano-2/         # 4 models
│   ├── KR CYBERTECH nano-2 ARC/     # 6 models
│   ├── KR_CYBERTECH_2/              # 20 models
│   └── KR_CYBERTECH_2_ARC/          # 1 model
├── Medium Payloads (30kg - 70kg)/
│   ├── Archiv/                      # 20 models (KR 30-3 / 30-4 KS / 60-3 / 60-4 KS / 60L30 / 60L45)
│   ├── Dummy_JET/                   # 8 models  (KR 30 / 60 / 60L30 / 60L45 JET ceiling & wall variants)
│   ├── IONTEC-2 Series/             # 5 models  (KR 50 R2100-2 / R2500-2, KR 70 R2100-2)
│   └── Iontec-Series/               # 10 models (KR 20 R3100, KR 30 R2100, KR 50 R2100/R2500, KR 70 R2100)
├── High Payloads (90kg - 300kg)/
│   ├── KR L Series/                 # 4 models  (KR 80L 3/4/5-axis Cartesian laser portals)
│   ├── Quantec Series/
│   │   ├── extra/                   # 20 models
│   │   ├── nano/                    # 6 models
│   │   ├── prime/                   # 12 models
│   │   ├── pro/                     # 2 models
│   │   └── ultra/                   # 19 models
│   └── Quantec-2/                   # 21 models
├── batch_vcmx_to_urdf.py            # Standalone .vcmx -> URDF converter & 6-check validator (pure stdlib)
├── BATCH_CONVERSION_REPORT.md       # Per-robot verification table across all 315 models
└── README.md
```

---

## 2. What `batch_vcmx_to_urdf.py` Does

KUKA.Sim 4.3 stores each robot as a `.vcmx` file (a Visual Components ZIP archive). `batch_vcmx_to_urdf.py` converts `.vcmx` files directly into standard URDF packages using **only the Python 3 standard library** (`zipfile`, `struct`, `math`, `re`, `xml.etree.ElementTree`):

1. **Parses the `component.rsc` Scene Graph:**
   - Extracts the `Functionality "rKinArticulated2"` block (`L01Z`, `L12X`, `L12Y`, `L23X`, `L23Z`, `L34X`, `L34Z`, `L45X`, `L45Z`, `L56Z`, `A45`, `JointZeroOffset1..6`, `JointSign1..6`) and root `VariableSpace` parameters.
   - Walks the `Node "rSimLink"` → `NodeClass` → `Feature` tree, evaluating cumulative 4×4 homogeneous transforms (`Matrix` fields and chained `Tx/Ty/Tz/Rx/Ry/Rz` expressions) to determine each joint's world-frame pivot (`A1`–`A6` and `mountplate`/`tool0`) and unit rotation/translation axis (`AxisType`).
   - Automatically handles standard 6-DOF revolute arms, ceiling (`_C`, `CLG`) and wall (`WALL`) mount transforms (`RobRoot`), custom-expression joints (`Dummy_JET`), and prismatic gantry axes (`KR 80L`).
   - Supports non-6-axis kinematics: 7-DOF redundant `LBR iiwa` (`A1`–`A7`), 4-DOF `KR-SCARA` (`A1`, `A2`, prismatic `A3`, `A4`), 4-DOF `KR-DELTA` (`Arm1`–`Arm3`, `J4` mapped to `link_1`–`link_4`), and 0-DOF static accessories (`AIRSKIN Safetyflange`, exported as `base_link` + fixed `tool0`).

2. **Extracts & Re-Centers `.3ds` Link Meshes into Binary `.stl`:**
   - Reads every embedded Autodesk `.3ds` (`0x4D4D`) mesh buffer directly from the `.vcmx` archive.
   - Filters out non-robot visualization geometry (`ShowEnvelope`, `ShowProfile`, dresspack sphere chains `Link1..Link39`, `VisPart_*`, `Pin_*`, `CableLink`, `Schlepp*` energy chains, iiwa `MonitoringSpaces`, optional floor plates `Base_plate`, and `Mediabox_small` duplicates).
   - Transforms mesh vertices from local `.3ds` millimeter coordinates into world meters at the default pose, then shifts each link's vertices into its own local joint frame so every `<visual>` and `<collision>` block in the URDF has a clean `xyz="0 0 0" rpy="0 0 0"` origin.

3. **Extracts Factory Controller Parameters (`_machine.dat` & `_robcor.dat`):**
   - When the `.vcmx` embeds KUKA controller MADA files (`krc/r1/mada/_machine.dat` and `_robcor.dat`, present in 218 / 315 models), the script extracts:
     - **Software joint limits:** `$SOFTN_END[1..6]` and `$SOFTP_END[1..6]` (shifted relative to the default pose `JointValue`, e.g. `A2 = -90°` upright).
     - **Max joint velocities:** computed from max motor RPM `$VEL_AXIS_MA[1..6]` divided by gear ratio `|$RAT_MOT_AX[1..6]|`.
     - **Peak joint torques (effort):** `$DYN_DAT[37..42]` from `_robcor.dat`.
     - **Link masses:** `$DYN_DAT[49..54]` from `_robcor.dat`.
   - When a `.vcmx` does not embed a `krc/` folder (97 / 315 models — e.g. all cobots, AIRSKIN, SCARA, DELTA), it falls back to `MinLimit`, `MaxLimit`, and `MaxSpeed` inside `component.rsc`'s `Dof` blocks, with payload-proportional link masses and joint torque estimates.

4. **Computes Area-Weighted COM & Strictly Positive-Definite Inertia Tensors:**
   - Computes each link's area-weighted center of mass (COM) and 3×3 moment-of-inertia tensor about the COM scaled to the link mass, with diagonal regularization guaranteeing a strictly symmetric positive-definite (SPD) inertia matrix for physics engines (PhysX, MuJoCo, Bullet, Dart).

---

## 3. How to Use

### A. Importing the Pre-Built URDFs into Simulators

#### NVIDIA Isaac Sim (GUI or Python)
1. In Isaac Sim, go to **`File` → `Import`** and select any `<robot>.urdf` (for example, `High Payloads (90kg - 300kg)/Quantec Series/pro/KR90_R2700_pro/kr90_r2700_pro.urdf`).
2. Recommended **URDF Importer** settings:
   - **Fix Base Link:** Checked
   - **Self Collision:** Checked (all non-adjacent links are verified clear at the zero pose)
   - **Collision Type:** **Convex Hull** (fast, conservative safety margin) or **Convex Decomposition** (tight concave fit; `<collision>` points directly to `meshes/visual/*.stl` so Isaac Sim can decompose the true link geometry)
   - **Joint Drive Type:** `Position` (for motion planning, pick-and-place, and position-controlled RL; increase stiffness/damping via **Gain Tuner** for heavy 90–300 kg arms) or `None` (for direct torque/force control)

> **Zero-Pose Convention Note:** At `q = 0`, articulated arms stand in KUKA.Sim's default upright L-pose (lower arm vertical along `+Z`, forearm horizontal along `+X`), which corresponds to `A2 = -90°, A3 = +90°` on a physical KUKA controller. Joint limits in `<limit lower="..." upper="..."/>` are already shifted relative to this pose.

#### PyBullet / MuJoCo / Pinocchio / ROS 2
All `<mesh filename="meshes/visual/link_*.stl"/>` paths are relative to each robot's `.urdf` directory, so standard loaders work directly:
```python
import pybullet as p
p.connect(p.DIRECT)
robot_id = p.loadURDF(
    "High Payloads (90kg - 300kg)/Quantec Series/pro/KR90_R2700_pro/kr90_r2700_pro.urdf",
    useFixedBase=True,
)
```

---

### B. Running `batch_vcmx_to_urdf.py` to Convert `.vcmx` Files

No third-party packages are needed (runs on any Python 3.8+ installation):

```powershell
# 1. Convert a single .vcmx file
python batch_vcmx_to_urdf.py `
  --single "C:\Users\Public\Documents\KUKA Public\KUKA.Sim 4.3\Models\KUKA.Sim Library 4.3\KUKA_ROBOTS\High Payloads (90kg - 300kg)\Quantec Series\pro\KR90_R2700_pro.vcmx" `
  --output-dir .\output_urdfs

# 2. Batch-convert an entire folder of .vcmx files recursively
python batch_vcmx_to_urdf.py `
  --input-dir "C:\Users\Public\Documents\KUKA Public\KUKA.Sim 4.3\Models\KUKA.Sim Library 4.3\KUKA_ROBOTS" `
  --output-dir .\output_urdfs
```

---

## 4. Constraints & Verification Checks Enforced

Every generated URDF package is automatically validated against **6 constraints** before conversion succeeds:

| Check ID | Constraint Checked | Why It Matters in Simulation |
| :---: | :--- | :--- |
| **`C1`** | **Scene-Graph & Kinematic Chain Consistency** — Evaluates `rKinArticulated2` link lengths (`L01Z`, `L12X`, `L23Z`, `L34X`, `L34Z`, `L56Z`, `A45`, `JointZeroOffset1..6`) and cumulative 4×4 `Node` → `Feature` transforms. | Prevents detached links, wrong reach, or rotated sub-components across floor, ceiling (`_C`), and wall variants. |
| **`C2`** | **Unit-Norm Joint Axes & Valid Joint Limits** — Transforms local `Dof` `AxisType` vectors into the world frame and asserts `\|\|axis\|\| == 1.0`, `lower < upper`, `velocity > 0`, and `effort > 0`. | Prevents zero/NaN joint axes or inverted limit bounds that cause physics solvers or IK (Lula / MoveIt) to reject the URDF. |
| **`C3`** | **Mesh Completeness & Zero-Origin Local Frames** — Strips 2D/3D workspace envelope geometry, shifts vertices into each link's local joint frame (`origin xyz="0 0 0"`), and asserts every link STL has `> 0` triangles and `> 84` bytes on disk. | Ensures no reach-envelope spheres pollute collisions and every link renders and collides at the exact joint pivot. |
| **`C4`** | **Factory MADA Parameter Alignment** — Extracts `$SOFTN_END`, `$SOFTP_END`, `$RAT_MOT_AX`, `$VEL_AXIS_MA` (`_machine.dat`) and `$DYN_DAT` peak torques & link masses (`_robcor.dat`), with fallback to `component.rsc` `Dof` limits. | Gives realistic actuator torque caps, speed limits, and mass distribution for RL policy training and torque/force control. |
| **`C5`** | **Strictly Symmetric Positive-Definite (SPD) 3×3 Inertia Tensors** — Asserts `mass > 0` and verifies Sylvester's criterion ($I_{xx} > 0$, $I_{xx}I_{yy} - I_{xy}^2 > 0$, $\det(I) > 0$) on every link. | Prevents PhysX / MuJoCo / Bullet instability or NaN accelerations caused by singular or negative-eigenvalue inertia matrices. |
| **`C6`** | **URDF XML Well-Formedness & Mesh Path Resolution** — Parses the written `.urdf` with `xml.etree.ElementTree`, verifies all `<mesh filename="..."/>` relative paths exist on disk, and logs the forward-kinematics `tool0` flange position. | Guarantees every `.urdf` loads cleanly on the first try without missing-file or XML syntax errors. |

---

## 5. Summary of All 315 Converted Models

- **Total `.vcmx` robots processed:** 315
- **Succeeded (passed all 6 verification checks):** 315 / 315 (0 failures)
- **With embedded factory `_machine.dat` / `_robcor.dat`:** 218 / 315 (remaining 97 use `component.rsc` limits)

| # | Category / Series / Model | DOFs | Triangles | Factory MADA | Flange `(x, y, z)` (m) | Warnings |
|---:|:---|---:|---:|:---:|:---:|:---|
| 1 | `Cobots (3kg - 15kg)\LBR iisy (3kg - 15kg)\LBR iisy 11 R1300.vcmx` | 6 | 340,331 | No (rsc) | `(+0.710, +0.000, +0.890)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 2 | `Cobots (3kg - 15kg)\LBR iisy (3kg - 15kg)\LBR iisy 15 R930.vcmx` | 6 | 337,826 | No (rsc) | `(+0.545, +0.000, +0.685)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 3 | `Cobots (3kg - 15kg)\LBR iisy (3kg - 15kg)\LBR iisy 3 R760.vcmx` | 6 | 66,294 | No (rsc) | `(+0.461, +0.000, +0.515)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 4 | `Cobots (3kg - 15kg)\LBR iisy (3kg - 15kg)\LBR iisy 6 R1300.vcmx` | 6 | 340,331 | No (rsc) | `(+0.710, +0.000, +0.890)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 5 | `Cobots (3kg - 15kg)\LBR iisy (3kg - 15kg)\LBR iisy 8 R930.vcmx` | 6 | 337,826 | No (rsc) | `(+0.545, +0.000, +0.685)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 6 | `Cobots (3kg - 15kg)\LBR iiwa (7kg - 14kg)\CleanRoom\LBR_iiwa_14_R820_CR.vcmx` | 7 | 530,989 | No (rsc) | `(+0.400, +0.000, +0.654)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 7 | `Cobots (3kg - 15kg)\LBR iiwa (7kg - 14kg)\CleanRoom\LBR_iiwa_7_R800_CR.vcmx` | 7 | 582,542 | No (rsc) | `(+0.400, +0.000, +0.614)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 8 | `Cobots (3kg - 15kg)\LBR iiwa (7kg - 14kg)\LBR_iiwa_14_R820.vcmx` | 7 | 530,989 | No (rsc) | `(+0.400, +0.000, +0.654)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 9 | `Cobots (3kg - 15kg)\LBR iiwa (7kg - 14kg)\LBR_iiwa_7_R800.vcmx` | 7 | 582,542 | No (rsc) | `(+0.400, +0.000, +0.614)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 10 | `Fenceless (4kg - 24kg)\AIRSKIN Safetyflange 10.vcmx` | 0 | 94,389 | No (rsc) | `(+0.000, +0.000, +0.000)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 11 | `Fenceless (4kg - 24kg)\AIRSKIN Safetyflange 20.vcmx` | 0 | 70,789 | No (rsc) | `(+0.000, +0.000, +0.000)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 12 | `Fenceless (4kg - 24kg)\Archiv\KR 16 R2010 AIRSKIN PA.vcmx` | 6 | 347,707 | No (rsc) | `(+1.173, +0.000, +1.650)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 13 | `Fenceless (4kg - 24kg)\Archiv\KR 16 R2010 AIRSKIN.vcmx` | 6 | 288,299 | No (rsc) | `(+1.173, +0.000, +1.650)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 14 | `Fenceless (4kg - 24kg)\Archiv\KR 16 R2010-2 AIRSKIN PA.vcmx` | 6 | 393,184 | No (rsc) | `(+1.173, +0.000, +1.650)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 15 | `Fenceless (4kg - 24kg)\Archiv\KR 16 R2010-2 AIRSKIN.vcmx` | 6 | 333,776 | No (rsc) | `(+1.173, +0.000, +1.650)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 16 | `Fenceless (4kg - 24kg)\Archiv\KR 20 R1810 AIRSKIN PA.vcmx` | 6 | 342,570 | No (rsc) | `(+1.173, +0.000, +1.450)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 17 | `Fenceless (4kg - 24kg)\Archiv\KR 20 R1810 AIRSKIN.vcmx` | 6 | 283,162 | No (rsc) | `(+1.173, +0.000, +1.450)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 18 | `Fenceless (4kg - 24kg)\Archiv\KR 20 R1810-2 AIRSKIN PA.vcmx` | 6 | 387,561 | No (rsc) | `(+1.173, +0.000, +1.450)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 19 | `Fenceless (4kg - 24kg)\Archiv\KR 20 R1810-2 AIRSKIN.vcmx` | 6 | 328,153 | No (rsc) | `(+1.173, +0.000, +1.450)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 20 | `Fenceless (4kg - 24kg)\Archiv\KR 22 R1610-2 AIRSKIN PA.vcmx` | 6 | 430,832 | No (rsc) | `(+0.968, +0.000, +1.450)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 21 | `Fenceless (4kg - 24kg)\Archiv\KR 22 R1610-2 AIRSKIN.vcmx` | 6 | 371,424 | No (rsc) | `(+0.968, +0.000, +1.450)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 22 | `Fenceless (4kg - 24kg)\KR 10 R1100-2 AIRSKIN PA.vcmx` | 6 | 244,138 | No (rsc) | `(+0.630, +0.000, +0.985)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 23 | `Fenceless (4kg - 24kg)\KR 10 R1100-2 AIRSKIN SF10.vcmx` | 6 | 273,623 | No (rsc) | `(+0.630, +0.000, +0.985)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 24 | `Fenceless (4kg - 24kg)\KR 10 R1100-2 AIRSKIN.vcmx` | 6 | 179,234 | No (rsc) | `(+0.630, +0.000, +0.985)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 25 | `Fenceless (4kg - 24kg)\KR 10 R900-2 AIRSKIN PA.vcmx` | 6 | 239,877 | No (rsc) | `(+0.535, +0.000, +0.880)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 26 | `Fenceless (4kg - 24kg)\KR 10 R900-2 AIRSKIN SF10.vcmx` | 6 | 269,362 | No (rsc) | `(+0.535, +0.000, +0.880)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 27 | `Fenceless (4kg - 24kg)\KR 10 R900-2 AIRSKIN.vcmx` | 6 | 174,973 | No (rsc) | `(+0.535, +0.000, +0.880)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 28 | `Fenceless (4kg - 24kg)\KR 12 R1810-2 AIRSKIN PA.vcmx` | 6 | 323,930 | No (rsc) | `(+1.173, +0.000, +1.450)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 29 | `Fenceless (4kg - 24kg)\KR 12 R1810-2 AIRSKIN SF20.vcmx` | 6 | 364,841 | No (rsc) | `(+1.173, +0.000, +1.450)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 30 | `Fenceless (4kg - 24kg)\KR 12 R1810-2 AIRSKIN.vcmx` | 6 | 294,052 | No (rsc) | `(+1.173, +0.000, +1.450)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 31 | `Fenceless (4kg - 24kg)\KR 16 R1610-2 AIRSKIN PA.vcmx` | 6 | 324,605 | No (rsc) | `(+0.968, +0.000, +1.450)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 32 | `Fenceless (4kg - 24kg)\KR 16 R1610-2 AIRSKIN SF20.vcmx` | 6 | 365,516 | No (rsc) | `(+0.968, +0.000, +1.450)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 33 | `Fenceless (4kg - 24kg)\KR 16 R1610-2 AIRSKIN.vcmx` | 6 | 294,727 | No (rsc) | `(+0.968, +0.000, +1.450)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 34 | `Fenceless (4kg - 24kg)\KR 16 R2010-2 AIRSKIN PA.vcmx` | 6 | 316,958 | No (rsc) | `(+1.173, +0.000, +1.650)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 35 | `Fenceless (4kg - 24kg)\KR 16 R2010-2 AIRSKIN SF20.vcmx` | 6 | 357,879 | No (rsc) | `(+1.173, +0.000, +1.650)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 36 | `Fenceless (4kg - 24kg)\KR 16 R2010-2 AIRSKIN.vcmx` | 6 | 287,090 | No (rsc) | `(+1.173, +0.000, +1.650)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 37 | `Fenceless (4kg - 24kg)\KR 20 R1810-2 AIRSKIN PA.vcmx` | 6 | 323,940 | No (rsc) | `(+1.173, +0.000, +1.450)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 38 | `Fenceless (4kg - 24kg)\KR 20 R1810-2 AIRSKIN SF20.vcmx` | 6 | 364,866 | No (rsc) | `(+1.173, +0.000, +1.450)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 39 | `Fenceless (4kg - 24kg)\KR 20 R1810-2 AIRSKIN.vcmx` | 6 | 294,077 | No (rsc) | `(+1.173, +0.000, +1.450)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 40 | `Fenceless (4kg - 24kg)\KR 22 R1610-2 AIRSKIN PA.vcmx` | 6 | 324,580 | No (rsc) | `(+0.968, +0.000, +1.450)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 41 | `Fenceless (4kg - 24kg)\KR 22 R1610-2 AIRSKIN SF20.vcmx` | 6 | 365,491 | No (rsc) | `(+0.968, +0.000, +1.450)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 42 | `Fenceless (4kg - 24kg)\KR 22 R1610-2 AIRSKIN.vcmx` | 6 | 294,702 | No (rsc) | `(+0.968, +0.000, +1.450)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 43 | `Fenceless (4kg - 24kg)\KR 6 R900-2 AIRSKIN PA.vcmx` | 6 | 254,809 | No (rsc) | `(+0.535, +0.000, +0.880)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 44 | `Fenceless (4kg - 24kg)\KR 6 R900-2 AIRSKIN SF10.vcmx` | 6 | 284,294 | No (rsc) | `(+0.535, +0.000, +0.880)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 45 | `Fenceless (4kg - 24kg)\KR 6 R900-2 AIRSKIN.vcmx` | 6 | 189,905 | No (rsc) | `(+0.535, +0.000, +0.880)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 46 | `High Payloads (90kg - 300kg)\KR L Series\KR 80L 4ax H.vcmx` | 4 | 167,438 | No (rsc) | `(+0.000, +0.000, +0.000)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 47 | `High Payloads (90kg - 300kg)\KR L Series\KR 80L 4ax V.vcmx` | 4 | 126,880 | No (rsc) | `(+0.000, +0.000, +0.000)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 48 | `High Payloads (90kg - 300kg)\KR L Series\KR 80L 5ax.vcmx` | 5 | 184,716 | No (rsc) | `(+0.000, +0.000, +0.000)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 49 | `High Payloads (90kg - 300kg)\KR L Series\KR 80L.vcmx` | 3 | 113,296 | No (rsc) | `(+0.000, +0.000, +0.000)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 50 | `High Payloads (90kg - 300kg)\Quantec Series\extra\KR120_R2700_extra_HA.vcmx` | 6 | 115,778 | Yes | `(+1.765, +0.000, +1.784)` | None |
| 51 | `High Payloads (90kg - 300kg)\Quantec Series\extra\KR120_R2900_extra.vcmx` | 6 | 164,071 | Yes | `(+1.765, +0.000, +1.984)` | None |
| 52 | `High Payloads (90kg - 300kg)\Quantec Series\extra\KR120_R2900_extra_C.vcmx` | 6 | 164,071 | Yes | `(+1.765, +0.000, +1.984)` | None |
| 53 | `High Payloads (90kg - 300kg)\Quantec Series\extra\KR120_R2900_extra_C_F_HP.vcmx` | 6 | 164,071 | Yes | `(+1.790, +0.000, +1.984)` | None |
| 54 | `High Payloads (90kg - 300kg)\Quantec Series\extra\KR120_R2900_extra_F_HP.vcmx` | 6 | 164,071 | Yes | `(+1.790, +0.000, +1.984)` | None |
| 55 | `High Payloads (90kg - 300kg)\Quantec Series\extra\KR150_R2700_extra.vcmx` | 6 | 130,266 | Yes | `(+1.765, +0.000, +1.784)` | None |
| 56 | `High Payloads (90kg - 300kg)\Quantec Series\extra\KR150_R2700_extra_C.vcmx` | 6 | 130,266 | Yes | `(+1.765, +0.000, +1.784)` | None |
| 57 | `High Payloads (90kg - 300kg)\Quantec Series\extra\KR150_R2700_extra_C_F_HP.vcmx` | 6 | 130,266 | Yes | `(+1.790, +0.000, +1.784)` | None |
| 58 | `High Payloads (90kg - 300kg)\Quantec Series\extra\KR150_R2700_extra_F_HP.vcmx` | 6 | 130,266 | Yes | `(+1.790, +0.000, +1.784)` | None |
| 59 | `High Payloads (90kg - 300kg)\Quantec Series\extra\KR180_R2500_extra.vcmx` | 6 | 127,347 | Yes | `(+1.565, +0.000, +1.784)` | None |
| 60 | `High Payloads (90kg - 300kg)\Quantec Series\extra\KR180_R2500_extra_C.vcmx` | 6 | 127,347 | Yes | `(+1.565, +0.000, +1.784)` | None |
| 61 | `High Payloads (90kg - 300kg)\Quantec Series\extra\KR180_R2500_extra_C_F_HP.vcmx` | 6 | 127,347 | Yes | `(+1.590, +0.000, +1.784)` | None |
| 62 | `High Payloads (90kg - 300kg)\Quantec Series\extra\KR180_R2500_extra_F_HP.vcmx` | 6 | 127,347 | Yes | `(+1.590, +0.000, +1.784)` | None |
| 63 | `High Payloads (90kg - 300kg)\Quantec Series\extra\KR210_R2700_extra.vcmx` | 6 | 117,008 | Yes | `(+1.765, +0.000, +1.784)` | None |
| 64 | `High Payloads (90kg - 300kg)\Quantec Series\extra\KR90_R2900_extra_HA.vcmx` | 6 | 116,284 | Yes | `(+1.765, +0.000, +1.984)` | None |
| 65 | `High Payloads (90kg - 300kg)\Quantec Series\extra\KR90_R3100_extra.vcmx` | 6 | 167,371 | Yes | `(+1.965, +0.000, +1.984)` | None |
| 66 | `High Payloads (90kg - 300kg)\Quantec Series\extra\KR90_R3100_extra_C.vcmx` | 6 | 167,371 | Yes | `(+1.965, +0.000, +1.984)` | None |
| 67 | `High Payloads (90kg - 300kg)\Quantec Series\extra\KR90_R3100_extra_C_F_HP.vcmx` | 6 | 167,371 | Yes | `(+1.990, +0.000, +1.984)` | None |
| 68 | `High Payloads (90kg - 300kg)\Quantec Series\extra\KR90_R3100_extra_F_HP.vcmx` | 6 | 167,371 | Yes | `(+1.990, +0.000, +1.984)` | None |
| 69 | `High Payloads (90kg - 300kg)\Quantec Series\extra\KR90_R3100_extra_HA.vcmx` | 6 | 128,929 | Yes | `(+1.965, +0.000, +1.984)` | None |
| 70 | `High Payloads (90kg - 300kg)\Quantec Series\nano\KR120_R1800_nano.vcmx` | 6 | 136,032 | Yes | `(+1.245, +0.000, +1.200)` | None |
| 71 | `High Payloads (90kg - 300kg)\Quantec Series\nano\KR120_R1800_nano_C.vcmx` | 6 | 136,230 | Yes | `(+1.245, +0.000, +1.200)` | None |
| 72 | `High Payloads (90kg - 300kg)\Quantec Series\nano\KR120_R2100_nano_F_exclusive.vcmx` | 6 | 190,527 | Yes | `(+1.440, +0.000, +1.470)` | None |
| 73 | `High Payloads (90kg - 300kg)\Quantec Series\nano\KR160_R1570_nano.vcmx` | 6 | 102,087 | Yes | `(+1.175, +0.000, +1.040)` | None |
| 74 | `High Payloads (90kg - 300kg)\Quantec Series\nano\KR160_R1570_nano_C.vcmx` | 6 | 102,285 | Yes | `(+1.175, +0.000, +1.040)` | None |
| 75 | `High Payloads (90kg - 300kg)\Quantec Series\nano\KR180 R2100 nano F exclusive.vcmx` | 6 | 238,132 | Yes | `(+1.440, +0.000, +1.470)` | None |
| 76 | `High Payloads (90kg - 300kg)\Quantec Series\prime\KR120_R3500_prime_K.vcmx` | 6 | 127,695 | Yes | `(+2.365, +0.000, +1.899)` | None |
| 77 | `High Payloads (90kg - 300kg)\Quantec Series\prime\KR150_R3100_prime.vcmx` | 6 | 117,702 | Yes | `(+1.965, +0.000, +1.984)` | None |
| 78 | `High Payloads (90kg - 300kg)\Quantec Series\prime\KR150_R3300_prime_K.vcmx` | 6 | 125,396 | Yes | `(+2.165, +0.000, +1.899)` | None |
| 79 | `High Payloads (90kg - 300kg)\Quantec Series\prime\KR180_R2900_prime.vcmx` | 6 | 115,396 | Yes | `(+1.765, +0.000, +1.984)` | None |
| 80 | `High Payloads (90kg - 300kg)\Quantec Series\prime\KR180_R3100_prime_K.vcmx` | 6 | 124,102 | Yes | `(+2.165, +0.000, +1.699)` | None |
| 81 | `High Payloads (90kg - 300kg)\Quantec Series\prime\KR210_R2700_prime.vcmx` | 6 | 204,051 | Yes | `(+1.790, +0.000, +1.784)` | None |
| 82 | `High Payloads (90kg - 300kg)\Quantec Series\prime\KR210_R2700_prime_C.vcmx` | 6 | 164,958 | Yes | `(+1.790, +0.000, +1.784)` | None |
| 83 | `High Payloads (90kg - 300kg)\Quantec Series\prime\KR210_R2700_prime_CR.vcmx` | 6 | 204,051 | Yes | `(+1.790, +0.000, +1.784)` | None |
| 84 | `High Payloads (90kg - 300kg)\Quantec Series\prime\KR210_R2900_prime_K.vcmx` | 6 | 137,571 | Yes | `(+1.990, +0.000, +1.699)` | None |
| 85 | `High Payloads (90kg - 300kg)\Quantec Series\prime\KR240_R2500_prime.vcmx` | 6 | 126,854 | Yes | `(+1.590, +0.000, +1.784)` | None |
| 86 | `High Payloads (90kg - 300kg)\Quantec Series\prime\KR240_R2700_prime.vcmx` | 6 | 128,537 | Yes | `(+1.790, +0.000, +1.784)` | None |
| 87 | `High Payloads (90kg - 300kg)\Quantec Series\prime\KR90_R3700_prime_K.vcmx` | 6 | 124,978 | Yes | `(+2.565, +0.000, +1.899)` | None |
| 88 | `High Payloads (90kg - 300kg)\Quantec Series\pro\KR120_R2500_pro.vcmx` | 6 | 113,403 | Yes | `(+1.565, +0.000, +1.784)` | None |
| 89 | `High Payloads (90kg - 300kg)\Quantec Series\pro\KR90_R2700_pro.vcmx` | 6 | 115,922 | Yes | `(+1.765, +0.000, +1.784)` | None |
| 90 | `High Payloads (90kg - 300kg)\Quantec Series\ultra\KR120_R3900_ultra_K.vcmx` | 6 | 223,221 | Yes | `(+2.765, +0.000, +1.899)` | None |
| 91 | `High Payloads (90kg - 300kg)\Quantec Series\ultra\KR120_R3900_ultra_K_F_HP.vcmx` | 6 | 223,221 | Yes | `(+2.790, +0.000, +1.899)` | None |
| 92 | `High Payloads (90kg - 300kg)\Quantec Series\ultra\KR150_R3700_ultra_K.vcmx` | 6 | 189,863 | Yes | `(+2.565, +0.000, +1.899)` | None |
| 93 | `High Payloads (90kg - 300kg)\Quantec Series\ultra\KR150_R3700_ultra_K_F_HP.vcmx` | 6 | 189,863 | Yes | `(+2.590, +0.000, +1.899)` | None |
| 94 | `High Payloads (90kg - 300kg)\Quantec Series\ultra\KR180_R3500_ultra_K.vcmx` | 6 | 192,580 | Yes | `(+2.365, +0.000, +1.899)` | None |
| 95 | `High Payloads (90kg - 300kg)\Quantec Series\ultra\KR180_R3500_ultra_K_F_HP.vcmx` | 6 | 192,580 | Yes | `(+2.390, +0.000, +1.899)` | None |
| 96 | `High Payloads (90kg - 300kg)\Quantec Series\ultra\KR210_R3100_ultra.vcmx` | 6 | 165,910 | Yes | `(+1.990, +0.000, +1.984)` | None |
| 97 | `High Payloads (90kg - 300kg)\Quantec Series\ultra\KR210_R3100_ultra_C.vcmx` | 6 | 165,910 | Yes | `(+1.990, +0.000, +1.984)` | None |
| 98 | `High Payloads (90kg - 300kg)\Quantec Series\ultra\KR210_R3300_ultra_K.vcmx` | 6 | 194,430 | Yes | `(+2.190, +0.000, +1.899)` | None |
| 99 | `High Payloads (90kg - 300kg)\Quantec Series\ultra\KR240_R2900_ultra.vcmx` | 6 | 164,380 | Yes | `(+1.790, +0.000, +1.984)` | None |
| 100 | `High Payloads (90kg - 300kg)\Quantec Series\ultra\KR240_R2900_ultra_C.vcmx` | 6 | 164,380 | Yes | `(+1.790, +0.000, +1.984)` | None |
| 101 | `High Payloads (90kg - 300kg)\Quantec Series\ultra\KR240_R3100_ultra_K.vcmx` | 6 | 193,134 | Yes | `(+2.190, +0.000, +1.699)` | None |
| 102 | `High Payloads (90kg - 300kg)\Quantec Series\ultra\KR270_R2700_ultra.vcmx` | 6 | 164,500 | Yes | `(+1.790, +0.000, +1.784)` | None |
| 103 | `High Payloads (90kg - 300kg)\Quantec Series\ultra\KR270_R2700_ultra_C.vcmx` | 6 | 164,408 | Yes | `(+1.790, +0.000, +1.784)` | None |
| 104 | `High Payloads (90kg - 300kg)\Quantec Series\ultra\KR270_R2900_ultra_K.vcmx` | 6 | 192,792 | Yes | `(+1.990, +0.000, +1.699)` | None |
| 105 | `High Payloads (90kg - 300kg)\Quantec Series\ultra\KR270_R3100_ultra_K.vcmx` | 6 | 193,230 | Yes | `(+2.190, +0.000, +1.699)` | None |
| 106 | `High Payloads (90kg - 300kg)\Quantec Series\ultra\KR300_R2500_ultra.vcmx` | 6 | 162,073 | Yes | `(+1.590, +0.000, +1.784)` | None |
| 107 | `High Payloads (90kg - 300kg)\Quantec Series\ultra\KR300_R2500_ultra_C.vcmx` | 6 | 162,073 | Yes | `(+1.590, +0.000, +1.784)` | None |
| 108 | `High Payloads (90kg - 300kg)\Quantec Series\ultra\KR300_R2500_ultra_SE.vcmx` | 6 | 162,073 | Yes | `(+1.590, +0.000, +1.784)` | None |
| 109 | `High Payloads (90kg - 300kg)\Quantec-2\KR 120 R3500-2 P-C.vcmx` | 6 | 277,998 | No (rsc) | `(+2.390, +0.000, +2.015)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 110 | `High Payloads (90kg - 300kg)\Quantec-2\KR 120 R3500-2 P.vcmx` | 6 | 277,998 | No (rsc) | `(+2.390, +0.000, +2.015)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 111 | `High Payloads (90kg - 300kg)\Quantec-2\KR 210 R2700-2 C01.vcmx` | 6 | 307,810 | Yes | `(+1.765, +0.000, +1.910)` | None |
| 112 | `High Payloads (90kg - 300kg)\Quantec-2\KR250R2700_2 C4 FLR.vcmx` | 6 | 324,411 | Yes | `(+1.790, +0.000, +1.910)` | None |
| 113 | `High Payloads (90kg - 300kg)\Quantec-2\KR300R2700_2 C4 FLR.vcmx` | 6 | 324,411 | Yes | `(+1.790, +0.000, +1.910)` | None |
| 114 | `High Payloads (90kg - 300kg)\Quantec-2\KR_120_R2700-2.vcmx` | 6 | 307,784 | Yes | `(+1.765, +0.000, +1.910)` | None |
| 115 | `High Payloads (90kg - 300kg)\Quantec-2\KR_120_R3100-2.vcmx` | 6 | 308,516 | Yes | `(+1.965, +0.000, +2.110)` | None |
| 116 | `High Payloads (90kg - 300kg)\Quantec-2\KR_120_R3900-2_K.vcmx` | 6 | 304,207 | No (rsc) | `(+2.765, +0.000, +2.015)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 117 | `High Payloads (90kg - 300kg)\Quantec-2\KR_150_R2700-2.vcmx` | 6 | 307,784 | Yes | `(+1.765, +0.000, +1.910)` | None |
| 118 | `High Payloads (90kg - 300kg)\Quantec-2\KR_150_R3100-2.vcmx` | 6 | 308,516 | Yes | `(+1.965, +0.000, +2.110)` | None |
| 119 | `High Payloads (90kg - 300kg)\Quantec-2\KR_180_R2900-2.vcmx` | 6 | 321,284 | Yes | `(+1.790, +0.000, +2.110)` | None |
| 120 | `High Payloads (90kg - 300kg)\Quantec-2\KR_180_R3500-2_K.vcmx` | 6 | 286,005 | No (rsc) | `(+2.365, +0.000, +2.015)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 121 | `High Payloads (90kg - 300kg)\Quantec-2\KR_210_R2700-2.vcmx` | 6 | 307,810 | Yes | `(+1.765, +0.000, +1.910)` | None |
| 122 | `High Payloads (90kg - 300kg)\Quantec-2\KR_210_R3100-2.vcmx` | 6 | 404,001 | No (rsc) | `(+1.990, +0.000, +2.110)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 123 | `High Payloads (90kg - 300kg)\Quantec-2\KR_210_R3100-2_C.vcmx` | 6 | 414,455 | Yes | `(+1.990, +0.000, +2.110)` | None |
| 124 | `High Payloads (90kg - 300kg)\Quantec-2\KR_210_R3300-2_K.vcmx` | 6 | 729,420 | No (rsc) | `(+2.190, +0.000, +2.015)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 125 | `High Payloads (90kg - 300kg)\Quantec-2\KR_240_R2900-2.vcmx` | 6 | 321,284 | Yes | `(+1.790, +0.000, +2.110)` | None |
| 126 | `High Payloads (90kg - 300kg)\Quantec-2\KR_240_R2900-2_C.vcmx` | 6 | 331,738 | Yes | `(+1.790, +0.000, +2.110)` | None |
| 127 | `High Payloads (90kg - 300kg)\Quantec-2\KR_250_R2700-2_C.vcmx` | 6 | 323,545 | Yes | `(+1.790, +0.000, +1.910)` | None |
| 128 | `High Payloads (90kg - 300kg)\Quantec-2\KR_270_R3100-2_K.vcmx` | 6 | 255,462 | No (rsc) | `(+2.190, +0.000, +1.815)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 129 | `High Payloads (90kg - 300kg)\Quantec-2\KR_300_R2700-2_C.vcmx` | 6 | 323,545 | Yes | `(+1.790, +0.000, +1.910)` | None |
| 130 | `Low Payloads (5kg - 22kg)\Archiv\KR CYBERTECH ARC nano\KR6 R1820 HW_Cybertech.vcmx` | 6 | 211,223 | Yes | `(+1.070, +0.000, +1.460)` | None |
| 131 | `Low Payloads (5kg - 22kg)\Archiv\KR CYBERTECH ARC nano\KR8 R1420 HW_Cybertech.vcmx` | 6 | 217,957 | Yes | `(+0.860, +0.000, +1.260)` | None |
| 132 | `Low Payloads (5kg - 22kg)\Archiv\KR CYBERTECH ARC nano\KR8 R1620 HW_Cybertech.vcmx` | 6 | 220,610 | Yes | `(+0.860, +0.000, +1.460)` | None |
| 133 | `Low Payloads (5kg - 22kg)\Archiv\KR CYBERTECH nano-2\KR 12 R1450-2 E.vcmx` | 6 | 230,567 | No (rsc) | `(+0.945, +0.000, +1.122)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 134 | `Low Payloads (5kg - 22kg)\Archiv\KR CYBERTECH nano\KR10_R1420_Cybertech.vcmx` | 6 | 135,327 | Yes | `(+0.890, +0.000, +1.080)` | None |
| 135 | `Low Payloads (5kg - 22kg)\Archiv\KR CYBERTECH nano\KR6_R1820_Cybertech.vcmx` | 6 | 139,515 | Yes | `(+1.090, +0.000, +1.280)` | None |
| 136 | `Low Payloads (5kg - 22kg)\Archiv\KR CYBERTECH nano\KR8_R1620_Cybertech.vcmx` | 6 | 136,232 | Yes | `(+0.890, +0.000, +1.280)` | None |
| 137 | `Low Payloads (5kg - 22kg)\Archiv\KR CYBERTECH-1 ARC\KR8 R2100 arc HW_Cybertech.vcmx` | 6 | 104,752 | Yes | `(+1.175, +0.000, +1.720)` | None |
| 138 | `Low Payloads (5kg - 22kg)\Archiv\KR CYBERTECH-1\KR 20 R2010 KS-F.vcmx` | 6 | 181,809 | Yes | `(+1.373, +0.000, +1.130)` | None |
| 139 | `Low Payloads (5kg - 22kg)\Archiv\KR CYBERTECH-1\KR12 R1810_Cybertech.vcmx` | 6 | 154,452 | Yes | `(+1.173, +0.000, +1.450)` | None |
| 140 | `Low Payloads (5kg - 22kg)\Archiv\KR CYBERTECH-1\KR16 R2010_Cybertech.vcmx` | 6 | 151,702 | Yes | `(+1.173, +0.000, +1.650)` | None |
| 141 | `Low Payloads (5kg - 22kg)\Archiv\KR CYBERTECH-1\KR16R1610_Cybertech.vcmx` | 6 | 149,898 | Yes | `(+0.968, +0.000, +1.450)` | None |
| 142 | `Low Payloads (5kg - 22kg)\Archiv\KR CYBERTECH-1\KR20R1810_Cybertech.vcmx` | 6 | 154,452 | Yes | `(+1.173, +0.000, +1.450)` | None |
| 143 | `Low Payloads (5kg - 22kg)\Archiv\KR CYBERTECH-1\KR22 R1610_Cybertech.vcmx` | 6 | 149,898 | Yes | `(+0.968, +0.000, +1.450)` | None |
| 144 | `Low Payloads (5kg - 22kg)\Archiv\KR CYBERTECH-1\KR8 R2010_Cybertech.vcmx` | 6 | 151,702 | Yes | `(+1.173, +0.000, +1.650)` | None |
| 145 | `Low Payloads (5kg - 22kg)\Archiv\KR16-2.vcmx` | 6 | 58,605 | Yes | `(+1.088, +0.000, +1.320)` | None |
| 146 | `Low Payloads (5kg - 22kg)\Archiv\KR16-2_C.vcmx` | 6 | 58,605 | Yes | `(+1.088, +0.000, +1.320)` | None |
| 147 | `Low Payloads (5kg - 22kg)\Archiv\KR16-2_KS.vcmx` | 6 | 70,084 | Yes | `(+1.278, +0.000, +0.880)` | None |
| 148 | `Low Payloads (5kg - 22kg)\Archiv\KR16-2_KS_C.vcmx` | 6 | 70,084 | Yes | `(+1.278, +0.000, +0.880)` | None |
| 149 | `Low Payloads (5kg - 22kg)\Archiv\KR16-2_W.vcmx` | 6 | 58,605 | Yes | `(+1.088, +0.000, +1.320)` | None |
| 150 | `Low Payloads (5kg - 22kg)\Archiv\KR16L6-2.vcmx` | 6 | 55,217 | Yes | `(+1.345, +0.000, +1.320)` | None |
| 151 | `Low Payloads (5kg - 22kg)\Archiv\KR16L6-2_C.vcmx` | 6 | 55,217 | Yes | `(+1.345, +0.000, +1.320)` | None |
| 152 | `Low Payloads (5kg - 22kg)\Archiv\KR16L6-2_KS.vcmx` | 6 | 65,535 | Yes | `(+1.535, +0.000, +0.880)` | None |
| 153 | `Low Payloads (5kg - 22kg)\Archiv\KR16L8-3_arc_HW.vcmx` | 6 | 126,581 | Yes | `(+1.522, +0.000, +1.525)` | None |
| 154 | `Low Payloads (5kg - 22kg)\Archiv\KR16_arc_HW.vcmx` | 6 | 196,749 | Yes | `(+1.135, +0.000, +1.525)` | None |
| 155 | `Low Payloads (5kg - 22kg)\Archiv\KR16_arc_HW_C.vcmx` | 6 | 196,749 | Yes | `(+1.135, +0.000, +1.525)` | None |
| 156 | `Low Payloads (5kg - 22kg)\Archiv\KR20-3.vcmx` | 6 | 58,605 | Yes | `(+1.088, +0.000, +1.320)` | None |
| 157 | `Low Payloads (5kg - 22kg)\Archiv\KR20-3_C.vcmx` | 6 | 58,605 | Yes | `(+1.088, +0.000, +1.320)` | None |
| 158 | `Low Payloads (5kg - 22kg)\Archiv\KR30L16-2.vcmx` | 6 | 37,997 | Yes | `(+2.053, +0.000, +2.160)` | None |
| 159 | `Low Payloads (5kg - 22kg)\Archiv\KR30L16-2_C.vcmx` | 6 | 37,997 | Yes | `(+2.053, +0.000, +2.160)` | None |
| 160 | `Low Payloads (5kg - 22kg)\Archiv\KR5-2_arc_HW.vcmx` | 6 | 130,810 | Yes | `(+1.000, +0.000, +1.170)` | None |
| 161 | `Low Payloads (5kg - 22kg)\Archiv\KR5-2_arc_HW_C.vcmx` | 6 | 130,810 | Yes | `(+1.000, +0.000, +1.170)` | None |
| 162 | `Low Payloads (5kg - 22kg)\Archiv\KR5_R1400.vcmx` | 6 | 44,011 | Yes | `(+0.880, +0.000, +1.040)` | None |
| 163 | `Low Payloads (5kg - 22kg)\Archiv\KR5_R1400_C.vcmx` | 6 | 44,011 | Yes | `(+0.880, +0.000, +1.040)` | None |
| 164 | `Low Payloads (5kg - 22kg)\Archiv\KR5_arc.vcmx` | 6 | 74,419 | Yes | `(+0.915, +0.000, +1.120)` | None |
| 165 | `Low Payloads (5kg - 22kg)\Archiv\KR5_arc_C.vcmx` | 6 | 74,419 | Yes | `(+0.915, +0.000, +1.120)` | None |
| 166 | `Low Payloads (5kg - 22kg)\Archiv\KR6-2.vcmx` | 6 | 20,322 | Yes | `(+1.045, +0.000, +1.320)` | link_2 had 0 visual meshes; created 1cm placeholder box |
| 167 | `Low Payloads (5kg - 22kg)\Archiv\KR6-2_C.vcmx` | 6 | 20,322 | Yes | `(+1.045, +0.000, +1.320)` | link_2 had 0 visual meshes; created 1cm placeholder box |
| 168 | `Low Payloads (5kg - 22kg)\Archiv\KR6-2_KS.vcmx` | 6 | 64,613 | Yes | `(+1.235, +0.000, +0.880)` | None |
| 169 | `Low Payloads (5kg - 22kg)\Archiv\KR6-2_W.vcmx` | 6 | 20,322 | Yes | `(+1.045, +0.000, +1.320)` | link_2 had 0 visual meshes; created 1cm placeholder box |
| 170 | `Low Payloads (5kg - 22kg)\Archiv\KR60L16-2_KS.vcmx` | 6 | 61,424 | Yes | `(+2.253, +0.000, +1.245)` | None |
| 171 | `Low Payloads (5kg - 22kg)\Archiv\KR_CYBERTECH_2\KR 35 R1840-2 E.vcmx` | 6 | 221,840 | No (rsc) | `(+1.097, +0.000, +1.454)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 172 | `Low Payloads (5kg - 22kg)\KR CYBERTECH nano-2 ARC\KR 6 R2010-2 arc HW E.vcmx` | 6 | 175,626 | Yes | `(+1.270, +0.000, +1.475)` | None |
| 173 | `Low Payloads (5kg - 22kg)\KR CYBERTECH nano-2 ARC\KR_6_R1440-2_arc_HW_E.vcmx` | 6 | 192,832 | Yes | `(+0.909, +0.000, +1.254)` | None |
| 174 | `Low Payloads (5kg - 22kg)\KR CYBERTECH nano-2 ARC\KR_6_R1840-2_arc_HW.vcmx` | 6 | 163,818 | No (rsc) | `(+1.087, +0.000, +1.445)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 175 | `Low Payloads (5kg - 22kg)\KR CYBERTECH nano-2 ARC\KR_8_R1440-2_arc_HW.vcmx` | 6 | 161,830 | No (rsc) | `(+0.877, +0.000, +1.245)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 176 | `Low Payloads (5kg - 22kg)\KR CYBERTECH nano-2 ARC\KR_8_R1440-2_arc_HW_E.vcmx` | 6 | 192,832 | Yes | `(+0.909, +0.000, +1.254)` | None |
| 177 | `Low Payloads (5kg - 22kg)\KR CYBERTECH nano-2 ARC\KR_8_R1640-2_arc_HW.vcmx` | 6 | 160,346 | No (rsc) | `(+0.877, +0.000, +1.445)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 178 | `Low Payloads (5kg - 22kg)\KR CYBERTECH nano-2\KR12 R1450-3 HW.vcmx` | 6 | 230,567 | Yes | `(+0.945, +0.000, +1.122)` | None |
| 179 | `Low Payloads (5kg - 22kg)\KR CYBERTECH nano-2\KR_10_R1440-2.vcmx` | 6 | 184,857 | No (rsc) | `(+0.930, +0.000, +1.055)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 180 | `Low Payloads (5kg - 22kg)\KR CYBERTECH nano-2\KR_6_R1840-2.vcmx` | 6 | 181,076 | No (rsc) | `(+1.130, +0.000, +1.255)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 181 | `Low Payloads (5kg - 22kg)\KR CYBERTECH nano-2\KR_8_R1640-2.vcmx` | 6 | 183,185 | No (rsc) | `(+0.930, +0.000, +1.255)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 182 | `Low Payloads (5kg - 22kg)\KR_CYBERTECH_2\KR12_R1810_2 C.vcmx` | 6 | 199,418 | Yes | `(+1.173, +0.000, +1.450)` | None |
| 183 | `Low Payloads (5kg - 22kg)\KR_CYBERTECH_2\KR12_R1810_2 W.vcmx` | 6 | 199,418 | Yes | `(+1.173, +0.000, +1.450)` | None |
| 184 | `Low Payloads (5kg - 22kg)\KR_CYBERTECH_2\KR12_R1810_2.vcmx` | 6 | 199,418 | Yes | `(+1.173, +0.000, +1.450)` | None |
| 185 | `Low Payloads (5kg - 22kg)\KR_CYBERTECH_2\KR16_R1610_2 C.vcmx` | 6 | 197,481 | Yes | `(+0.968, +0.000, +1.450)` | None |
| 186 | `Low Payloads (5kg - 22kg)\KR_CYBERTECH_2\KR16_R1610_2 W.vcmx` | 6 | 197,481 | Yes | `(+0.968, +0.000, +1.450)` | None |
| 187 | `Low Payloads (5kg - 22kg)\KR_CYBERTECH_2\KR16_R1610_2.vcmx` | 6 | 197,481 | Yes | `(+0.968, +0.000, +1.450)` | None |
| 188 | `Low Payloads (5kg - 22kg)\KR_CYBERTECH_2\KR16_R2010_2 C.vcmx` | 6 | 197,179 | Yes | `(+1.173, +0.000, +1.650)` | None |
| 189 | `Low Payloads (5kg - 22kg)\KR_CYBERTECH_2\KR16_R2010_2 W.vcmx` | 6 | 197,179 | Yes | `(+1.173, +0.000, +1.650)` | None |
| 190 | `Low Payloads (5kg - 22kg)\KR_CYBERTECH_2\KR16_R2010_2.vcmx` | 6 | 197,179 | Yes | `(+1.173, +0.000, +1.650)` | None |
| 191 | `Low Payloads (5kg - 22kg)\KR_CYBERTECH_2\KR20R1820-2 E.vcmx` | 6 | 356,647 | Yes | `(+1.153, +0.000, +1.484)` | None |
| 192 | `Low Payloads (5kg - 22kg)\KR_CYBERTECH_2\KR20_R1810_2 C.vcmx` | 6 | 199,443 | Yes | `(+1.173, +0.000, +1.450)` | None |
| 193 | `Low Payloads (5kg - 22kg)\KR_CYBERTECH_2\KR20_R1810_2 W.vcmx` | 6 | 199,443 | Yes | `(+1.173, +0.000, +1.450)` | None |
| 194 | `Low Payloads (5kg - 22kg)\KR_CYBERTECH_2\KR20_R1810_2.vcmx` | 6 | 199,443 | Yes | `(+1.173, +0.000, +1.450)` | None |
| 195 | `Low Payloads (5kg - 22kg)\KR_CYBERTECH_2\KR22_R1610_2 C.vcmx` | 6 | 197,456 | Yes | `(+0.968, +0.000, +1.450)` | None |
| 196 | `Low Payloads (5kg - 22kg)\KR_CYBERTECH_2\KR22_R1610_2 W.vcmx` | 6 | 197,456 | Yes | `(+0.968, +0.000, +1.450)` | None |
| 197 | `Low Payloads (5kg - 22kg)\KR_CYBERTECH_2\KR22_R1610_2.vcmx` | 6 | 197,456 | Yes | `(+0.968, +0.000, +1.450)` | None |
| 198 | `Low Payloads (5kg - 22kg)\KR_CYBERTECH_2\KR35 R1840-3 HW.vcmx` | 6 | 221,840 | Yes | `(+1.097, +0.000, +1.454)` | None |
| 199 | `Low Payloads (5kg - 22kg)\KR_CYBERTECH_2\KR8_R2010_2 C.vcmx` | 6 | 197,154 | Yes | `(+1.173, +0.000, +1.650)` | None |
| 200 | `Low Payloads (5kg - 22kg)\KR_CYBERTECH_2\KR8_R2010_2 W.vcmx` | 6 | 197,154 | Yes | `(+1.173, +0.000, +1.650)` | None |
| 201 | `Low Payloads (5kg - 22kg)\KR_CYBERTECH_2\KR8_R2010_2.vcmx` | 6 | 197,154 | Yes | `(+1.173, +0.000, +1.650)` | None |
| 202 | `Low Payloads (5kg - 22kg)\KR_CYBERTECH_2_ARC\KR8_R2100-2_arc_HW.vcmx` | 6 | 99,194 | Yes | `(+1.175, +0.000, +1.720)` | None |
| 203 | `Medium Payloads (30kg - 70kg)\Archiv\KR30-3.vcmx` | 6 | 205,912 | Yes | `(+1.340, +0.000, +1.810)` | None |
| 204 | `Medium Payloads (30kg - 70kg)\Archiv\KR30-3_C.vcmx` | 6 | 205,912 | Yes | `(+1.340, +0.000, +1.810)` | None |
| 205 | `Medium Payloads (30kg - 70kg)\Archiv\KR30-3_HA.vcmx` | 6 | 205,912 | Yes | `(+1.340, +0.000, +1.810)` | None |
| 206 | `Medium Payloads (30kg - 70kg)\Archiv\KR30-3_HA_C.vcmx` | 6 | 205,912 | Yes | `(+1.340, +0.000, +1.810)` | None |
| 207 | `Medium Payloads (30kg - 70kg)\Archiv\KR30-4_KS.vcmx` | 6 | 277,160 | Yes | `(+1.540, +0.000, +1.245)` | None |
| 208 | `Medium Payloads (30kg - 70kg)\Archiv\KR60-3.vcmx` | 6 | 205,912 | Yes | `(+1.340, +0.000, +1.810)` | None |
| 209 | `Medium Payloads (30kg - 70kg)\Archiv\KR60-3_C.vcmx` | 6 | 205,912 | Yes | `(+1.340, +0.000, +1.810)` | None |
| 210 | `Medium Payloads (30kg - 70kg)\Archiv\KR60-3_HA.vcmx` | 6 | 205,912 | Yes | `(+1.340, +0.000, +1.810)` | None |
| 211 | `Medium Payloads (30kg - 70kg)\Archiv\KR60-3_HA_C.vcmx` | 6 | 205,912 | Yes | `(+1.340, +0.000, +1.810)` | None |
| 212 | `Medium Payloads (30kg - 70kg)\Archiv\KR60-4_KS.vcmx` | 6 | 277,160 | Yes | `(+1.540, +0.000, +1.245)` | None |
| 213 | `Medium Payloads (30kg - 70kg)\Archiv\KR60L30-3.vcmx` | 6 | 207,586 | Yes | `(+1.740, +0.000, +1.810)` | None |
| 214 | `Medium Payloads (30kg - 70kg)\Archiv\KR60L30-3_C.vcmx` | 6 | 207,586 | Yes | `(+1.740, +0.000, +1.810)` | None |
| 215 | `Medium Payloads (30kg - 70kg)\Archiv\KR60L30-3_HA_C.vcmx` | 6 | 207,586 | Yes | `(+1.740, +0.000, +1.810)` | None |
| 216 | `Medium Payloads (30kg - 70kg)\Archiv\KR60L30-3_HA_CR.vcmx` | 6 | 207,586 | Yes | `(+1.740, +0.000, +1.810)` | None |
| 217 | `Medium Payloads (30kg - 70kg)\Archiv\KR60L30-4KS.vcmx` | 6 | 269,664 | Yes | `(+1.940, +0.000, +1.245)` | None |
| 218 | `Medium Payloads (30kg - 70kg)\Archiv\KR60L45-3.vcmx` | 6 | 213,295 | Yes | `(+1.540, +0.000, +1.810)` | None |
| 219 | `Medium Payloads (30kg - 70kg)\Archiv\KR60L45-3_C.vcmx` | 6 | 213,295 | Yes | `(+1.540, +0.000, +1.810)` | None |
| 220 | `Medium Payloads (30kg - 70kg)\Archiv\KR60L45-3_HA.vcmx` | 6 | 213,295 | Yes | `(+1.540, +0.000, +1.810)` | None |
| 221 | `Medium Payloads (30kg - 70kg)\Archiv\KR60L45-3_HA_C.vcmx` | 6 | 213,295 | Yes | `(+1.540, +0.000, +1.810)` | None |
| 222 | `Medium Payloads (30kg - 70kg)\Archiv\KR60L45-4_KS.vcmx` | 6 | 282,111 | Yes | `(+1.740, +0.000, +1.245)` | None |
| 223 | `Medium Payloads (30kg - 70kg)\Dummy_JET\KR 30 JET CLG.vcmx` | 6 | 18,534 | Yes | `(+0.990, -0.000, -0.995)` | None |
| 224 | `Medium Payloads (30kg - 70kg)\Dummy_JET\KR 30 JET WALL.vcmx` | 6 | 18,400 | Yes | `(+0.995, +0.000, -0.990)` | None |
| 225 | `Medium Payloads (30kg - 70kg)\Dummy_JET\KR 60 JET CLG.vcmx` | 6 | 18,534 | Yes | `(+0.990, -0.000, -0.995)` | None |
| 226 | `Medium Payloads (30kg - 70kg)\Dummy_JET\KR 60 JET WALL.vcmx` | 6 | 18,400 | Yes | `(+0.995, +0.000, -0.990)` | None |
| 227 | `Medium Payloads (30kg - 70kg)\Dummy_JET\KR 60L30 JET CLG.vcmx` | 6 | 18,534 | Yes | `(+1.390, -0.000, -0.995)` | None |
| 228 | `Medium Payloads (30kg - 70kg)\Dummy_JET\KR 60L30 JET WALL.vcmx` | 6 | 18,400 | Yes | `(+0.995, +0.000, -1.390)` | None |
| 229 | `Medium Payloads (30kg - 70kg)\Dummy_JET\KR 60L45 JET CLG.vcmx` | 6 | 18,534 | Yes | `(+1.190, -0.000, -0.995)` | None |
| 230 | `Medium Payloads (30kg - 70kg)\Dummy_JET\KR 60L45 JET WALL.vcmx` | 6 | 18,400 | Yes | `(+0.995, +0.000, -1.190)` | None |
| 231 | `Medium Payloads (30kg - 70kg)\IONTEC-2 Series\KR 50 R2100-2.vcmx` | 6 | 186,829 | No (rsc) | `(+1.385, +0.000, +1.597)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 232 | `Medium Payloads (30kg - 70kg)\IONTEC-2 Series\KR 50 R2500-2 F.vcmx` | 6 | 193,756 | No (rsc) | `(+1.585, +0.000, +1.797)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 233 | `Medium Payloads (30kg - 70kg)\IONTEC-2 Series\KR 50 R2500-2.vcmx` | 6 | 193,756 | No (rsc) | `(+1.585, +0.000, +1.797)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 234 | `Medium Payloads (30kg - 70kg)\IONTEC-2 Series\KR 70 R2100-2 F.vcmx` | 6 | 186,829 | No (rsc) | `(+1.385, +0.000, +1.597)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 235 | `Medium Payloads (30kg - 70kg)\IONTEC-2 Series\KR 70 R2100-2.vcmx` | 6 | 186,829 | No (rsc) | `(+1.385, +0.000, +1.597)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 236 | `Medium Payloads (30kg - 70kg)\Iontec-Series\KR 50 R2100 E.vcmx` | 6 | 186,829 | No (rsc) | `(+1.385, +0.000, +1.597)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 237 | `Medium Payloads (30kg - 70kg)\Iontec-Series\KR 50 R2500 E.vcmx` | 6 | 193,756 | No (rsc) | `(+1.585, +0.000, +1.797)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 238 | `Medium Payloads (30kg - 70kg)\Iontec-Series\KR 70 R2100 E.vcmx` | 6 | 186,829 | Yes | `(+1.385, +0.000, +1.597)` | None |
| 239 | `Medium Payloads (30kg - 70kg)\Iontec-Series\KR_20_R3100.vcmx` | 6 | 231,572 | No (rsc) | `(+1.963, +0.000, +1.915)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 240 | `Medium Payloads (30kg - 70kg)\Iontec-Series\KR_30_R2100.vcmx` | 6 | 313,916 | No (rsc) | `(+1.395, +0.000, +1.515)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 241 | `Medium Payloads (30kg - 70kg)\Iontec-Series\KR_50_R2100.vcmx` | 6 | 313,916 | No (rsc) | `(+1.395, +0.000, +1.515)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 242 | `Medium Payloads (30kg - 70kg)\Iontec-Series\KR_50_R2500.vcmx` | 6 | 300,195 | No (rsc) | `(+1.395, +0.000, +1.915)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 243 | `Medium Payloads (30kg - 70kg)\Iontec-Series\KR_50_R2500_F.vcmx` | 6 | 301,530 | No (rsc) | `(+1.395, +0.000, +1.915)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 244 | `Medium Payloads (30kg - 70kg)\Iontec-Series\KR_70_R2100.vcmx` | 6 | 316,170 | No (rsc) | `(+1.395, +0.000, +1.515)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 245 | `Medium Payloads (30kg - 70kg)\Iontec-Series\KR_70_R2100_F.vcmx` | 6 | 317,515 | No (rsc) | `(+1.395, +0.000, +1.515)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 246 | `Small Robots (3kg - 10kg)\AGILUS KR4\KR4_R600.vcmx` | 6 | 46,921 | No (rsc) | `(+0.385, +0.000, +0.640)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 247 | `Small Robots (3kg - 10kg)\AGILUS-2 Series\HM_SC\KR_10_R1100_HM_SC.vcmx` | 6 | 180,234 | Yes | `(+0.627, +0.000, +0.995)` | None |
| 248 | `Small Robots (3kg - 10kg)\AGILUS-2 Series\HM_SC\KR_10_R900_HM_SC.vcmx` | 6 | 224,536 | Yes | `(+0.532, +0.000, +0.890)` | None |
| 249 | `Small Robots (3kg - 10kg)\AGILUS-2 Series\HM_SC\KR_6_R700_HM_SC.vcmx` | 6 | 172,507 | Yes | `(+0.477, +0.000, +0.750)` | None |
| 250 | `Small Robots (3kg - 10kg)\AGILUS-2 Series\HM_SC\KR_6_R900_HM_SC.vcmx` | 6 | 224,536 | Yes | `(+0.532, +0.000, +0.890)` | None |
| 251 | `Small Robots (3kg - 10kg)\AGILUS-2 Series\KR 10 R1100-2 C.vcmx` | 6 | 121,356 | Yes | `(+0.630, +0.000, +0.985)` | None |
| 252 | `Small Robots (3kg - 10kg)\AGILUS-2 Series\KR 10 R1100-2 W.vcmx` | 6 | 121,356 | Yes | `(+0.630, +0.000, +0.985)` | None |
| 253 | `Small Robots (3kg - 10kg)\AGILUS-2 Series\KR10_R1100_2.vcmx` | 6 | 121,356 | Yes | `(+0.630, +0.000, +0.985)` | None |
| 254 | `Small Robots (3kg - 10kg)\AGILUS-2 Series\KR10_R900_2 C.vcmx` | 6 | 122,931 | Yes | `(+0.535, +0.000, +0.880)` | None |
| 255 | `Small Robots (3kg - 10kg)\AGILUS-2 Series\KR10_R900_2 W.vcmx` | 6 | 122,931 | Yes | `(+0.535, +0.000, +0.880)` | None |
| 256 | `Small Robots (3kg - 10kg)\AGILUS-2 Series\KR10_R900_2.vcmx` | 6 | 122,931 | Yes | `(+0.535, +0.000, +0.880)` | None |
| 257 | `Small Robots (3kg - 10kg)\AGILUS-2 Series\KR3_R540 C.vcmx` | 6 | 196,864 | Yes | `(+0.355, +0.000, +0.625)` | None |
| 258 | `Small Robots (3kg - 10kg)\AGILUS-2 Series\KR3_R540 W.vcmx` | 6 | 196,864 | Yes | `(+0.355, +0.000, +0.625)` | None |
| 259 | `Small Robots (3kg - 10kg)\AGILUS-2 Series\KR3_R540.vcmx` | 6 | 196,864 | Yes | `(+0.355, +0.000, +0.625)` | None |
| 260 | `Small Robots (3kg - 10kg)\AGILUS-2 Series\KR6_R700_2 C.vcmx` | 6 | 124,685 | Yes | `(+0.480, +0.000, +0.760)` | None |
| 261 | `Small Robots (3kg - 10kg)\AGILUS-2 Series\KR6_R700_2 W.vcmx` | 6 | 124,685 | Yes | `(+0.480, +0.000, +0.760)` | None |
| 262 | `Small Robots (3kg - 10kg)\AGILUS-2 Series\KR6_R700_2.vcmx` | 6 | 124,685 | Yes | `(+0.480, +0.000, +0.760)` | None |
| 263 | `Small Robots (3kg - 10kg)\AGILUS-2 Series\KR6_R900_2 C.vcmx` | 6 | 122,931 | Yes | `(+0.535, +0.000, +0.880)` | None |
| 264 | `Small Robots (3kg - 10kg)\AGILUS-2 Series\KR6_R900_2 W.vcmx` | 6 | 122,931 | Yes | `(+0.535, +0.000, +0.880)` | None |
| 265 | `Small Robots (3kg - 10kg)\AGILUS-2 Series\KR6_R900_2.vcmx` | 6 | 122,931 | Yes | `(+0.535, +0.000, +0.880)` | None |
| 266 | `Small Robots (3kg - 10kg)\AGILUS-3 Series\KR 4 R500-3 C01.vcmx` | 6 | 86,773 | No (rsc) | `(+0.325, +0.000, +0.595)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 267 | `Small Robots (3kg - 10kg)\AGILUS-3 Series\KR 4 R600-3 C01.vcmx` | 6 | 88,099 | No (rsc) | `(+0.385, +0.000, +0.635)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 268 | `Small Robots (3kg - 10kg)\AGILUS-3 Series\KR 7 R700-3 E.vcmx` | 6 | 85,700 | Yes | `(+0.427, +0.000, +0.737)` | None |
| 269 | `Small Robots (3kg - 10kg)\AGILUS-3 Series\KR 7 R900-3 E.vcmx` | 6 | 83,829 | No (rsc) | `(+0.567, +0.000, +0.797)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 270 | `Small Robots (3kg - 10kg)\AGILUS-3 Series\KR7 R700-3 C01.vcmx` | 6 | 85,700 | No (rsc) | `(+0.427, +0.000, +0.737)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 271 | `Small Robots (3kg - 10kg)\AGILUS-3 Series\KR7 R900-3 C01.vcmx` | 6 | 83,829 | No (rsc) | `(+0.567, +0.000, +0.797)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 272 | `Small Robots (3kg - 10kg)\Archiv\AGILUS-1 Series\fivve\KR10_R1100_fivve.vcmx` | 6 | 23,559 | Yes | `(+0.540, +0.000, +0.915)` | None |
| 273 | `Small Robots (3kg - 10kg)\Archiv\AGILUS-1 Series\fivve\KR10_R1100_fivve_C.vcmx` | 6 | 23,628 | Yes | `(+0.540, +0.000, +1.075)` | None |
| 274 | `Small Robots (3kg - 10kg)\Archiv\AGILUS-1 Series\fivve\KR6_R700_fivve.vcmx` | 6 | 25,338 | Yes | `(+0.390, +0.000, +0.670)` | None |
| 275 | `Small Robots (3kg - 10kg)\Archiv\AGILUS-1 Series\fivve\KR6_R700_fivve_C.vcmx` | 6 | 24,098 | Yes | `(+0.390, +0.000, +0.830)` | None |
| 276 | `Small Robots (3kg - 10kg)\Archiv\AGILUS-1 Series\fivve\KR6_R900_fivve.vcmx` | 6 | 23,036 | Yes | `(+0.445, +0.000, +0.810)` | None |
| 277 | `Small Robots (3kg - 10kg)\Archiv\AGILUS-1 Series\fivve\KR6_R900_fivve_C.vcmx` | 6 | 23,026 | Yes | `(+0.445, +0.000, +0.970)` | None |
| 278 | `Small Robots (3kg - 10kg)\Archiv\AGILUS-1 Series\sixx\KR 10 R1100 sixx_CR.vcmx` | 6 | 147,970 | Yes | `(+0.620, +0.000, +0.995)` | None |
| 279 | `Small Robots (3kg - 10kg)\Archiv\AGILUS-1 Series\sixx\KR 10 R900 sixx_CR.vcmx` | 6 | 116,458 | Yes | `(+0.525, +0.000, +0.890)` | None |
| 280 | `Small Robots (3kg - 10kg)\Archiv\AGILUS-1 Series\sixx\KR 6 R700 sixx_CR.vcmx` | 6 | 129,395 | Yes | `(+0.470, +0.000, +0.750)` | None |
| 281 | `Small Robots (3kg - 10kg)\Archiv\AGILUS-1 Series\sixx\KR 6 R900 sixx_CR.vcmx` | 6 | 116,458 | Yes | `(+0.525, +0.000, +0.890)` | None |
| 282 | `Small Robots (3kg - 10kg)\Archiv\AGILUS-1 Series\sixx\KR 6 R900 sixx_W.vcmx` | 6 | 112,821 | Yes | `(+0.525, +0.000, +0.890)` | None |
| 283 | `Small Robots (3kg - 10kg)\Archiv\AGILUS-1 Series\sixx\KR10_R1100_sixx.vcmx` | 6 | 146,987 | Yes | `(+0.620, +0.000, +0.995)` | None |
| 284 | `Small Robots (3kg - 10kg)\Archiv\AGILUS-1 Series\sixx\KR10_R1100_sixx_C.vcmx` | 6 | 144,333 | Yes | `(+0.620, +0.000, +0.995)` | None |
| 285 | `Small Robots (3kg - 10kg)\Archiv\AGILUS-1 Series\sixx\KR10_R1100_sixx_W.vcmx` | 6 | 144,333 | Yes | `(+0.620, +0.000, +0.995)` | None |
| 286 | `Small Robots (3kg - 10kg)\Archiv\AGILUS-1 Series\sixx\KR10_R900_sixx.vcmx` | 6 | 112,821 | Yes | `(+0.525, +0.000, +0.890)` | None |
| 287 | `Small Robots (3kg - 10kg)\Archiv\AGILUS-1 Series\sixx\KR10_R900_sixx_C.vcmx` | 6 | 112,821 | Yes | `(+0.525, +0.000, +0.890)` | None |
| 288 | `Small Robots (3kg - 10kg)\Archiv\AGILUS-1 Series\sixx\KR10_R900_sixx_W.vcmx` | 6 | 112,821 | Yes | `(+0.525, +0.000, +0.890)` | None |
| 289 | `Small Robots (3kg - 10kg)\Archiv\AGILUS-1 Series\sixx\KR6_R700_sixx.vcmx` | 6 | 125,758 | Yes | `(+0.470, +0.000, +0.750)` | None |
| 290 | `Small Robots (3kg - 10kg)\Archiv\AGILUS-1 Series\sixx\KR6_R700_sixx_C.vcmx` | 6 | 125,758 | Yes | `(+0.470, +0.000, +0.750)` | None |
| 291 | `Small Robots (3kg - 10kg)\Archiv\AGILUS-1 Series\sixx\KR6_R700_sixx_W.vcmx` | 6 | 125,758 | Yes | `(+0.470, +0.000, +0.750)` | None |
| 292 | `Small Robots (3kg - 10kg)\Archiv\AGILUS-1 Series\sixx\KR6_R900_sixx.vcmx` | 6 | 112,821 | Yes | `(+0.525, +0.000, +0.890)` | None |
| 293 | `Small Robots (3kg - 10kg)\Archiv\AGILUS-1 Series\sixx\KR6_R900_sixx_C.vcmx` | 6 | 112,821 | Yes | `(+0.525, +0.000, +0.890)` | None |
| 294 | `Small Robots (3kg - 10kg)\Archiv\KR-SCARA\KR6R500 SC 200.vcmx` | 4 | 77,479 | Yes | `(+0.000, +0.000, +0.000)` | link_4 had 0 visual meshes; created 1cm placeholder box |
| 295 | `Small Robots (3kg - 10kg)\Archiv\KR-SCARA\KR6R700 SC 200.vcmx` | 4 | 87,736 | Yes | `(+0.000, +0.000, +0.000)` | link_4 had 0 visual meshes; created 1cm placeholder box |
| 296 | `Small Robots (3kg - 10kg)\KR-DELTA\KR3 D1200 DELTA Beta-Version.vcmx` | 4 | 161,117 | Yes | `(-0.313, +0.000, -0.125)` | None |
| 297 | `Small Robots (3kg - 10kg)\KR-DELTA\KR3-D1200-DELTA-HM.vcmx` | 4 | 198,464 | Yes | `(-0.238, +0.000, -0.110)` | None |
| 298 | `Small Robots (3kg - 10kg)\KR-SCARA_2\KR_6_R500_Z200_2.vcmx` | 4 | 264,354 | Yes | `(+0.000, +0.000, +0.000)` | None |
| 299 | `Small Robots (3kg - 10kg)\KR-SCARA_2\KR_6_R700_Z200_2.vcmx` | 4 | 371,131 | Yes | `(+0.000, +0.000, +0.000)` | None |
| 300 | `Small Robots (3kg - 10kg)\SCARA 12kg\CleanRoom\KR_12_R650_Z340_CR.vcmx` | 4 | 328,998 | No (rsc) | `(+0.000, +0.000, +0.000)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 301 | `Small Robots (3kg - 10kg)\SCARA 12kg\CleanRoom\KR_12_R750_Z340_CR.vcmx` | 4 | 340,797 | No (rsc) | `(+0.000, +0.000, +0.000)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 302 | `Small Robots (3kg - 10kg)\SCARA 12kg\CleanRoom\KR_12_R850_Z340_CR.vcmx` | 4 | 327,451 | No (rsc) | `(+0.000, +0.000, +0.000)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 303 | `Small Robots (3kg - 10kg)\SCARA 12kg\CleanRoom\KR_20_R1000_Z390_CR.vcmx` | 4 | 323,395 | No (rsc) | `(+0.000, +0.000, +0.000)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 304 | `Small Robots (3kg - 10kg)\SCARA 12kg\CleanRoom\KR_20_R1200_Z390_CR.vcmx` | 4 | 319,429 | No (rsc) | `(+0.000, +0.000, +0.000)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 305 | `Small Robots (3kg - 10kg)\SCARA 12kg\CleanRoom\KR_20_R800_Z390_CR.vcmx` | 4 | 374,898 | No (rsc) | `(+0.000, +0.000, +0.000)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 306 | `Small Robots (3kg - 10kg)\SCARA 12kg\HO_Variant\KR6 R700 Z170 HO.vcmx` | 4 | 118,784 | No (rsc) | `(+0.000, +0.000, +0.000)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 307 | `Small Robots (3kg - 10kg)\SCARA 12kg\HO_Variant\KR_13_R850_Z340_HO.vcmx` | 4 | 135,982 | Yes | `(+0.000, +0.000, +0.000)` | None |
| 308 | `Small Robots (3kg - 10kg)\SCARA 12kg\Standard\KR20 R1000 Z450.vcmx` | 4 | 169,252 | No (rsc) | `(+0.000, +0.000, +0.000)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 309 | `Small Robots (3kg - 10kg)\SCARA 12kg\Standard\KR20 R1200 Z450.vcmx` | 4 | 193,739 | No (rsc) | `(+0.000, +0.000, +0.000)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 310 | `Small Robots (3kg - 10kg)\SCARA 12kg\Standard\KR20 R800 Z450.vcmx` | 4 | 237,282 | No (rsc) | `(+0.000, +0.000, +0.000)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 311 | `Small Robots (3kg - 10kg)\SCARA 12kg\Standard\KR60 R1200 Z400.vcmx` | 4 | 74,273 | No (rsc) | `(+0.000, +0.000, +0.000)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 312 | `Small Robots (3kg - 10kg)\SCARA 12kg\Standard\KR_12_R650_Z400.vcmx` | 4 | 254,093 | No (rsc) | `(+0.000, +0.000, +0.000)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 313 | `Small Robots (3kg - 10kg)\SCARA 12kg\Standard\KR_12_R750_Z400.vcmx` | 4 | 350,968 | Yes | `(+0.000, +0.000, +0.000)` | None |
| 314 | `Small Robots (3kg - 10kg)\SCARA 12kg\Standard\KR_12_R750_Z600.vcmx` | 4 | 325,565 | Yes | `(+0.000, +0.000, +0.000)` | None |
| 315 | `Small Robots (3kg - 10kg)\SCARA 12kg\Standard\KR_12_R850_Z400.vcmx` | 4 | 252,546 | No (rsc) | `(+0.000, +0.000, +0.000)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
