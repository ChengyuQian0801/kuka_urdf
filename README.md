# KUKA Medium & High Payload URDF Library (from KUKA.Sim 4.3)

A collection of **127 verified URDF packages** (**881 binary STL link meshes**) covering all **Medium Payload (30 kg – 70 kg)** and **High Payload (90 kg – 300 kg)** KUKA industrial robots from **KUKA.Sim 4.3**, along with the zero-dependency Python converter (`batch_vcmx_to_urdf.py`) used to generate and validate them.

---

## 1. Repository Layout

Every payload category, product series, and robot variant has its own self-contained directory containing `<robot_slug>.urdf` and `meshes/visual/*.stl`:

```text
kuka_urdf/
├── High Payloads (90kg - 300kg)/
│   ├── KR L Series/                 # 4 models (KR 80L 3/4/5-axis Cartesian laser portals)
│   ├── Quantec Series/
│   │   ├── extra/                   # 20 models (KR 90 / 120 / 150 / 180 / 210 extra, FLR/C/F/HP/HA)
│   │   ├── nano/                    # 6 models  (KR 120 / 160 / 180 nano)
│   │   ├── prime/                   # 12 models (KR 90 / 120 / 150 / 180 / 210 / 240 prime & K-shelf)
│   │   ├── pro/                     # 2 models  (KR 90 R2700 pro, KR 120 R2500 pro)
│   │   └── ultra/                   # 19 models (KR 120 / 150 / 180 / 210 / 240 / 270 / 300 ultra)
│   └── Quantec-2/                   # 21 models (KR 120 / 150 / 180 / 210 / 240 / 250 / 270 / 300 -2)
├── Medium Payloads (30kg - 70kg)/
│   ├── Archiv/                      # 20 models (KR 30-3 / 30-4 KS / 60-3 / 60-4 KS / 60L30 / 60L45)
│   ├── Dummy_JET/                   # 8 models  (KR 30 / 60 / 60L30 / 60L45 JET ceiling & wall variants)
│   ├── IONTEC-2 Series/             # 5 models  (KR 50 R2100-2 / R2500-2, KR 70 R2100-2)
│   └── Iontec-Series/               # 10 models (KR 20 R3100, KR 30 R2100, KR 50 R2100/R2500, KR 70 R2100)
├── batch_vcmx_to_urdf.py            # Standalone .vcmx -> URDF converter & 6-check validator (pure stdlib)
├── BATCH_CONVERSION_REPORT.md       # Per-robot verification table across all 127 models
└── README.md
```

---

## 2. What `batch_vcmx_to_urdf.py` Does

KUKA.Sim 4.3 stores each robot as a `.vcmx` file (a Visual Components ZIP archive). `batch_vcmx_to_urdf.py` converts `.vcmx` files directly into standard URDF packages using **only the Python 3 standard library** (`zipfile`, `struct`, `math`, `re`, `xml.etree.ElementTree`):

1. **Parses the `component.rsc` Scene Graph:**
   - Extracts the `Functionality "rKinArticulated2"` block (`L01Z`, `L12X`, `L12Y`, `L23X`, `L23Z`, `L34X`, `L34Z`, `L45X`, `L45Z`, `L56Z`, `A45`, `JointZeroOffset1..6`, `JointSign1..6`) and root `VariableSpace` parameters.
   - Walks the `Node "rSimLink"` → `NodeClass` → `Feature` tree, evaluating cumulative 4×4 homogeneous transforms (`Matrix` fields and chained `Tx/Ty/Tz/Rx/Ry/Rz` expressions) to determine each joint's world-frame pivot (`A1`–`A6` and `mountplate`/`tool0`) and unit rotation/translation axis (`AxisType`).
   - Automatically handles standard 6-DOF revolute arms, ceiling (`_C`, `CLG`) and wall (`WALL`) mount transforms (`RobRoot`), custom-expression joints (`Dummy_JET`), and prismatic gantry axes (`KR 80L`).

2. **Extracts & Re-Centers `.3ds` Link Meshes into Binary `.stl`:**
   - Reads every embedded Autodesk `.3ds` (`0x4D4D`) mesh buffer directly from the `.vcmx` archive.
   - Filters out non-robot visualization geometry (`ShowEnvelope`, `ShowProfile`, dresspack sphere chains `Link1..Link39`, optional floor plates `Base_plate`, and `Mediabox_small` duplicates).
   - Transforms mesh vertices from local `.3ds` millimeter coordinates into world meters at the default pose, then shifts each link's vertices into its own local joint frame so every `<visual>` and `<collision>` block in the URDF has a clean `xyz="0 0 0" rpy="0 0 0"` origin.

3. **Extracts Factory Controller Parameters (`_machine.dat` & `_robcor.dat`):**
   - When the `.vcmx` embeds KUKA controller MADA files (`krc/r1/mada/_machine.dat` and `_robcor.dat`, present in 102 / 127 models), the script extracts:
     - **Software joint limits:** `$SOFTN_END[1..6]` and `$SOFTP_END[1..6]` (shifted relative to the default pose `JointValue`, e.g. `A2 = -90°` upright).
     - **Max joint velocities:** computed from max motor RPM `$VEL_AXIS_MA[1..6]` divided by gear ratio `|$RAT_MOT_AX[1..6]|`.
     - **Peak joint torques (effort):** `$DYN_DAT[37..42]` from `_robcor.dat`.
     - **Link masses:** `$DYN_DAT[49..54]` from `_robcor.dat`.
   - When a `.vcmx` does not embed a `krc/` folder (25 / 127 models), it falls back to `MinLimit`, `MaxLimit`, and `MaxSpeed` inside `component.rsc`'s `Dof` blocks.

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

> **Zero-Pose Convention Note:** At `q = [0, 0, 0, 0, 0, 0]`, articulated arms stand in KUKA.Sim's default upright L-pose (lower arm vertical along `+Z`, forearm horizontal along `+X`), which corresponds to `A2 = -90°, A3 = +90°` on a physical KUKA controller. Joint limits in `<limit lower="..." upper="..."/>` are already shifted relative to this pose.

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

## 5. Summary of All 127 Converted Models

- **Total `.vcmx` robots processed:** 127
- **Succeeded (passed all 6 verification checks):** 127 / 127 (0 failures)
- **With embedded factory `_machine.dat` / `_robcor.dat`:** 102 / 127 (remaining 25 use `component.rsc` limits)

| # | Category / Series / Model | DOFs | Triangles | Factory MADA | Flange `(x, y, z)` (m) | Warnings |
|---:|:---|---:|---:|:---:|:---:|:---|
| 1 | `High Payloads (90kg - 300kg)\KR L Series\KR 80L 4ax H.vcmx` | 4 | 171,474 | No (rsc) | `(+0.000, +0.000, +0.000)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 2 | `High Payloads (90kg - 300kg)\KR L Series\KR 80L 4ax V.vcmx` | 4 | 130,916 | No (rsc) | `(+0.000, +0.000, +0.000)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 3 | `High Payloads (90kg - 300kg)\KR L Series\KR 80L 5ax.vcmx` | 5 | 188,752 | No (rsc) | `(+0.000, +0.000, +0.000)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 4 | `High Payloads (90kg - 300kg)\KR L Series\KR 80L.vcmx` | 3 | 117,332 | No (rsc) | `(+0.000, +0.000, +0.000)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 5 | `High Payloads (90kg - 300kg)\Quantec Series\extra\KR120_R2700_extra_HA.vcmx` | 6 | 115,778 | Yes | `(+1.765, +0.000, +1.784)` | None |
| 6 | `High Payloads (90kg - 300kg)\Quantec Series\extra\KR120_R2900_extra.vcmx` | 6 | 164,071 | Yes | `(+1.765, +0.000, +1.984)` | None |
| 7 | `High Payloads (90kg - 300kg)\Quantec Series\extra\KR120_R2900_extra_C.vcmx` | 6 | 164,071 | Yes | `(+1.765, +0.000, +1.984)` | None |
| 8 | `High Payloads (90kg - 300kg)\Quantec Series\extra\KR120_R2900_extra_C_F_HP.vcmx` | 6 | 164,071 | Yes | `(+1.790, +0.000, +1.984)` | None |
| 9 | `High Payloads (90kg - 300kg)\Quantec Series\extra\KR120_R2900_extra_F_HP.vcmx` | 6 | 164,071 | Yes | `(+1.790, +0.000, +1.984)` | None |
| 10 | `High Payloads (90kg - 300kg)\Quantec Series\extra\KR150_R2700_extra.vcmx` | 6 | 130,266 | Yes | `(+1.765, +0.000, +1.784)` | None |
| 11 | `High Payloads (90kg - 300kg)\Quantec Series\extra\KR150_R2700_extra_C.vcmx` | 6 | 130,266 | Yes | `(+1.765, +0.000, +1.784)` | None |
| 12 | `High Payloads (90kg - 300kg)\Quantec Series\extra\KR150_R2700_extra_C_F_HP.vcmx` | 6 | 130,266 | Yes | `(+1.790, +0.000, +1.784)` | None |
| 13 | `High Payloads (90kg - 300kg)\Quantec Series\extra\KR150_R2700_extra_F_HP.vcmx` | 6 | 130,266 | Yes | `(+1.790, +0.000, +1.784)` | None |
| 14 | `High Payloads (90kg - 300kg)\Quantec Series\extra\KR180_R2500_extra.vcmx` | 6 | 127,347 | Yes | `(+1.565, +0.000, +1.784)` | None |
| 15 | `High Payloads (90kg - 300kg)\Quantec Series\extra\KR180_R2500_extra_C.vcmx` | 6 | 127,347 | Yes | `(+1.565, +0.000, +1.784)` | None |
| 16 | `High Payloads (90kg - 300kg)\Quantec Series\extra\KR180_R2500_extra_C_F_HP.vcmx` | 6 | 127,347 | Yes | `(+1.590, +0.000, +1.784)` | None |
| 17 | `High Payloads (90kg - 300kg)\Quantec Series\extra\KR180_R2500_extra_F_HP.vcmx` | 6 | 127,347 | Yes | `(+1.590, +0.000, +1.784)` | None |
| 18 | `High Payloads (90kg - 300kg)\Quantec Series\extra\KR210_R2700_extra.vcmx` | 6 | 117,008 | Yes | `(+1.765, +0.000, +1.784)` | None |
| 19 | `High Payloads (90kg - 300kg)\Quantec Series\extra\KR90_R2900_extra_HA.vcmx` | 6 | 116,284 | Yes | `(+1.765, +0.000, +1.984)` | None |
| 20 | `High Payloads (90kg - 300kg)\Quantec Series\extra\KR90_R3100_extra.vcmx` | 6 | 167,371 | Yes | `(+1.965, +0.000, +1.984)` | None |
| 21 | `High Payloads (90kg - 300kg)\Quantec Series\extra\KR90_R3100_extra_C.vcmx` | 6 | 167,371 | Yes | `(+1.965, +0.000, +1.984)` | None |
| 22 | `High Payloads (90kg - 300kg)\Quantec Series\extra\KR90_R3100_extra_C_F_HP.vcmx` | 6 | 167,371 | Yes | `(+1.990, +0.000, +1.984)` | None |
| 23 | `High Payloads (90kg - 300kg)\Quantec Series\extra\KR90_R3100_extra_F_HP.vcmx` | 6 | 167,371 | Yes | `(+1.990, +0.000, +1.984)` | None |
| 24 | `High Payloads (90kg - 300kg)\Quantec Series\extra\KR90_R3100_extra_HA.vcmx` | 6 | 128,929 | Yes | `(+1.965, +0.000, +1.984)` | None |
| 25 | `High Payloads (90kg - 300kg)\Quantec Series\nano\KR120_R1800_nano.vcmx` | 6 | 136,032 | Yes | `(+1.245, +0.000, +1.200)` | None |
| 26 | `High Payloads (90kg - 300kg)\Quantec Series\nano\KR120_R1800_nano_C.vcmx` | 6 | 136,230 | Yes | `(+1.245, +0.000, +1.200)` | None |
| 27 | `High Payloads (90kg - 300kg)\Quantec Series\nano\KR120_R2100_nano_F_exclusive.vcmx` | 6 | 190,527 | Yes | `(+1.440, +0.000, +1.470)` | None |
| 28 | `High Payloads (90kg - 300kg)\Quantec Series\nano\KR160_R1570_nano.vcmx` | 6 | 102,087 | Yes | `(+1.175, +0.000, +1.040)` | None |
| 29 | `High Payloads (90kg - 300kg)\Quantec Series\nano\KR160_R1570_nano_C.vcmx` | 6 | 102,285 | Yes | `(+1.175, +0.000, +1.040)` | None |
| 30 | `High Payloads (90kg - 300kg)\Quantec Series\nano\KR180 R2100 nano F exclusive.vcmx` | 6 | 238,132 | Yes | `(+1.440, +0.000, +1.470)` | None |
| 31 | `High Payloads (90kg - 300kg)\Quantec Series\prime\KR120_R3500_prime_K.vcmx` | 6 | 127,695 | Yes | `(+2.365, +0.000, +1.899)` | None |
| 32 | `High Payloads (90kg - 300kg)\Quantec Series\prime\KR150_R3100_prime.vcmx` | 6 | 117,702 | Yes | `(+1.965, +0.000, +1.984)` | None |
| 33 | `High Payloads (90kg - 300kg)\Quantec Series\prime\KR150_R3300_prime_K.vcmx` | 6 | 125,396 | Yes | `(+2.165, +0.000, +1.899)` | None |
| 34 | `High Payloads (90kg - 300kg)\Quantec Series\prime\KR180_R2900_prime.vcmx` | 6 | 115,396 | Yes | `(+1.765, +0.000, +1.984)` | None |
| 35 | `High Payloads (90kg - 300kg)\Quantec Series\prime\KR180_R3100_prime_K.vcmx` | 6 | 124,102 | Yes | `(+2.165, +0.000, +1.699)` | None |
| 36 | `High Payloads (90kg - 300kg)\Quantec Series\prime\KR210_R2700_prime.vcmx` | 6 | 204,051 | Yes | `(+1.790, +0.000, +1.784)` | None |
| 37 | `High Payloads (90kg - 300kg)\Quantec Series\prime\KR210_R2700_prime_C.vcmx` | 6 | 164,958 | Yes | `(+1.790, +0.000, +1.784)` | None |
| 38 | `High Payloads (90kg - 300kg)\Quantec Series\prime\KR210_R2700_prime_CR.vcmx` | 6 | 204,051 | Yes | `(+1.790, +0.000, +1.784)` | None |
| 39 | `High Payloads (90kg - 300kg)\Quantec Series\prime\KR210_R2900_prime_K.vcmx` | 6 | 137,571 | Yes | `(+1.990, +0.000, +1.699)` | None |
| 40 | `High Payloads (90kg - 300kg)\Quantec Series\prime\KR240_R2500_prime.vcmx` | 6 | 126,854 | Yes | `(+1.590, +0.000, +1.784)` | None |
| 41 | `High Payloads (90kg - 300kg)\Quantec Series\prime\KR240_R2700_prime.vcmx` | 6 | 128,537 | Yes | `(+1.790, +0.000, +1.784)` | None |
| 42 | `High Payloads (90kg - 300kg)\Quantec Series\prime\KR90_R3700_prime_K.vcmx` | 6 | 124,978 | Yes | `(+2.565, +0.000, +1.899)` | None |
| 43 | `High Payloads (90kg - 300kg)\Quantec Series\pro\KR120_R2500_pro.vcmx` | 6 | 113,403 | Yes | `(+1.565, +0.000, +1.784)` | None |
| 44 | `High Payloads (90kg - 300kg)\Quantec Series\pro\KR90_R2700_pro.vcmx` | 6 | 115,922 | Yes | `(+1.765, +0.000, +1.784)` | None |
| 45 | `High Payloads (90kg - 300kg)\Quantec Series\ultra\KR120_R3900_ultra_K.vcmx` | 6 | 223,221 | Yes | `(+2.765, +0.000, +1.899)` | None |
| 46 | `High Payloads (90kg - 300kg)\Quantec Series\ultra\KR120_R3900_ultra_K_F_HP.vcmx` | 6 | 223,221 | Yes | `(+2.790, +0.000, +1.899)` | None |
| 47 | `High Payloads (90kg - 300kg)\Quantec Series\ultra\KR150_R3700_ultra_K.vcmx` | 6 | 189,863 | Yes | `(+2.565, +0.000, +1.899)` | None |
| 48 | `High Payloads (90kg - 300kg)\Quantec Series\ultra\KR150_R3700_ultra_K_F_HP.vcmx` | 6 | 189,863 | Yes | `(+2.590, +0.000, +1.899)` | None |
| 49 | `High Payloads (90kg - 300kg)\Quantec Series\ultra\KR180_R3500_ultra_K.vcmx` | 6 | 192,580 | Yes | `(+2.365, +0.000, +1.899)` | None |
| 50 | `High Payloads (90kg - 300kg)\Quantec Series\ultra\KR180_R3500_ultra_K_F_HP.vcmx` | 6 | 192,580 | Yes | `(+2.390, +0.000, +1.899)` | None |
| 51 | `High Payloads (90kg - 300kg)\Quantec Series\ultra\KR210_R3100_ultra.vcmx` | 6 | 165,910 | Yes | `(+1.990, +0.000, +1.984)` | None |
| 52 | `High Payloads (90kg - 300kg)\Quantec Series\ultra\KR210_R3100_ultra_C.vcmx` | 6 | 165,910 | Yes | `(+1.990, +0.000, +1.984)` | None |
| 53 | `High Payloads (90kg - 300kg)\Quantec Series\ultra\KR210_R3300_ultra_K.vcmx` | 6 | 194,430 | Yes | `(+2.190, +0.000, +1.899)` | None |
| 54 | `High Payloads (90kg - 300kg)\Quantec Series\ultra\KR240_R2900_ultra.vcmx` | 6 | 164,380 | Yes | `(+1.790, +0.000, +1.984)` | None |
| 55 | `High Payloads (90kg - 300kg)\Quantec Series\ultra\KR240_R2900_ultra_C.vcmx` | 6 | 164,380 | Yes | `(+1.790, +0.000, +1.984)` | None |
| 56 | `High Payloads (90kg - 300kg)\Quantec Series\ultra\KR240_R3100_ultra_K.vcmx` | 6 | 193,134 | Yes | `(+2.190, +0.000, +1.699)` | None |
| 57 | `High Payloads (90kg - 300kg)\Quantec Series\ultra\KR270_R2700_ultra.vcmx` | 6 | 164,500 | Yes | `(+1.790, +0.000, +1.784)` | None |
| 58 | `High Payloads (90kg - 300kg)\Quantec Series\ultra\KR270_R2700_ultra_C.vcmx` | 6 | 164,408 | Yes | `(+1.790, +0.000, +1.784)` | None |
| 59 | `High Payloads (90kg - 300kg)\Quantec Series\ultra\KR270_R2900_ultra_K.vcmx` | 6 | 192,792 | Yes | `(+1.990, +0.000, +1.699)` | None |
| 60 | `High Payloads (90kg - 300kg)\Quantec Series\ultra\KR270_R3100_ultra_K.vcmx` | 6 | 193,230 | Yes | `(+2.190, +0.000, +1.699)` | None |
| 61 | `High Payloads (90kg - 300kg)\Quantec Series\ultra\KR300_R2500_ultra.vcmx` | 6 | 162,073 | Yes | `(+1.590, +0.000, +1.784)` | None |
| 62 | `High Payloads (90kg - 300kg)\Quantec Series\ultra\KR300_R2500_ultra_C.vcmx` | 6 | 162,073 | Yes | `(+1.590, +0.000, +1.784)` | None |
| 63 | `High Payloads (90kg - 300kg)\Quantec Series\ultra\KR300_R2500_ultra_SE.vcmx` | 6 | 162,073 | Yes | `(+1.590, +0.000, +1.784)` | None |
| 64 | `High Payloads (90kg - 300kg)\Quantec-2\KR 120 R3500-2 P-C.vcmx` | 6 | 277,998 | No (rsc) | `(+2.390, +0.000, +2.015)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 65 | `High Payloads (90kg - 300kg)\Quantec-2\KR 120 R3500-2 P.vcmx` | 6 | 277,998 | No (rsc) | `(+2.390, +0.000, +2.015)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 66 | `High Payloads (90kg - 300kg)\Quantec-2\KR 210 R2700-2 C01.vcmx` | 6 | 307,810 | Yes | `(+1.765, +0.000, +1.910)` | None |
| 67 | `High Payloads (90kg - 300kg)\Quantec-2\KR250R2700_2 C4 FLR.vcmx` | 6 | 324,411 | Yes | `(+1.790, +0.000, +1.910)` | None |
| 68 | `High Payloads (90kg - 300kg)\Quantec-2\KR300R2700_2 C4 FLR.vcmx` | 6 | 324,411 | Yes | `(+1.790, +0.000, +1.910)` | None |
| 69 | `High Payloads (90kg - 300kg)\Quantec-2\KR_120_R2700-2.vcmx` | 6 | 307,784 | Yes | `(+1.765, +0.000, +1.910)` | None |
| 70 | `High Payloads (90kg - 300kg)\Quantec-2\KR_120_R3100-2.vcmx` | 6 | 308,516 | Yes | `(+1.965, +0.000, +2.110)` | None |
| 71 | `High Payloads (90kg - 300kg)\Quantec-2\KR_120_R3900-2_K.vcmx` | 6 | 304,207 | No (rsc) | `(+2.765, +0.000, +2.015)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 72 | `High Payloads (90kg - 300kg)\Quantec-2\KR_150_R2700-2.vcmx` | 6 | 307,784 | Yes | `(+1.765, +0.000, +1.910)` | None |
| 73 | `High Payloads (90kg - 300kg)\Quantec-2\KR_150_R3100-2.vcmx` | 6 | 308,516 | Yes | `(+1.965, +0.000, +2.110)` | None |
| 74 | `High Payloads (90kg - 300kg)\Quantec-2\KR_180_R2900-2.vcmx` | 6 | 321,284 | Yes | `(+1.790, +0.000, +2.110)` | None |
| 75 | `High Payloads (90kg - 300kg)\Quantec-2\KR_180_R3500-2_K.vcmx` | 6 | 286,005 | No (rsc) | `(+2.365, +0.000, +2.015)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 76 | `High Payloads (90kg - 300kg)\Quantec-2\KR_210_R2700-2.vcmx` | 6 | 307,810 | Yes | `(+1.765, +0.000, +1.910)` | None |
| 77 | `High Payloads (90kg - 300kg)\Quantec-2\KR_210_R3100-2.vcmx` | 6 | 404,001 | No (rsc) | `(+1.990, +0.000, +2.110)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 78 | `High Payloads (90kg - 300kg)\Quantec-2\KR_210_R3100-2_C.vcmx` | 6 | 414,455 | Yes | `(+1.990, +0.000, +2.110)` | None |
| 79 | `High Payloads (90kg - 300kg)\Quantec-2\KR_210_R3300-2_K.vcmx` | 6 | 729,420 | No (rsc) | `(+2.190, +0.000, +2.015)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 80 | `High Payloads (90kg - 300kg)\Quantec-2\KR_240_R2900-2.vcmx` | 6 | 321,284 | Yes | `(+1.790, +0.000, +2.110)` | None |
| 81 | `High Payloads (90kg - 300kg)\Quantec-2\KR_240_R2900-2_C.vcmx` | 6 | 331,738 | Yes | `(+1.790, +0.000, +2.110)` | None |
| 82 | `High Payloads (90kg - 300kg)\Quantec-2\KR_250_R2700-2_C.vcmx` | 6 | 323,545 | Yes | `(+1.790, +0.000, +1.910)` | None |
| 83 | `High Payloads (90kg - 300kg)\Quantec-2\KR_270_R3100-2_K.vcmx` | 6 | 255,462 | No (rsc) | `(+2.190, +0.000, +1.815)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 84 | `High Payloads (90kg - 300kg)\Quantec-2\KR_300_R2700-2_C.vcmx` | 6 | 323,545 | Yes | `(+1.790, +0.000, +1.910)` | None |
| 85 | `Medium Payloads (30kg - 70kg)\Archiv\KR30-3.vcmx` | 6 | 205,912 | Yes | `(+1.340, +0.000, +1.810)` | None |
| 86 | `Medium Payloads (30kg - 70kg)\Archiv\KR30-3_C.vcmx` | 6 | 205,912 | Yes | `(+1.340, +0.000, +1.810)` | None |
| 87 | `Medium Payloads (30kg - 70kg)\Archiv\KR30-3_HA.vcmx` | 6 | 205,912 | Yes | `(+1.340, +0.000, +1.810)` | None |
| 88 | `Medium Payloads (30kg - 70kg)\Archiv\KR30-3_HA_C.vcmx` | 6 | 205,912 | Yes | `(+1.340, +0.000, +1.810)` | None |
| 89 | `Medium Payloads (30kg - 70kg)\Archiv\KR30-4_KS.vcmx` | 6 | 277,160 | Yes | `(+1.540, +0.000, +1.245)` | None |
| 90 | `Medium Payloads (30kg - 70kg)\Archiv\KR60-3.vcmx` | 6 | 205,912 | Yes | `(+1.340, +0.000, +1.810)` | None |
| 91 | `Medium Payloads (30kg - 70kg)\Archiv\KR60-3_C.vcmx` | 6 | 205,912 | Yes | `(+1.340, +0.000, +1.810)` | None |
| 92 | `Medium Payloads (30kg - 70kg)\Archiv\KR60-3_HA.vcmx` | 6 | 205,912 | Yes | `(+1.340, +0.000, +1.810)` | None |
| 93 | `Medium Payloads (30kg - 70kg)\Archiv\KR60-3_HA_C.vcmx` | 6 | 205,912 | Yes | `(+1.340, +0.000, +1.810)` | None |
| 94 | `Medium Payloads (30kg - 70kg)\Archiv\KR60-4_KS.vcmx` | 6 | 277,160 | Yes | `(+1.540, +0.000, +1.245)` | None |
| 95 | `Medium Payloads (30kg - 70kg)\Archiv\KR60L30-3.vcmx` | 6 | 207,586 | Yes | `(+1.740, +0.000, +1.810)` | None |
| 96 | `Medium Payloads (30kg - 70kg)\Archiv\KR60L30-3_C.vcmx` | 6 | 207,586 | Yes | `(+1.740, +0.000, +1.810)` | None |
| 97 | `Medium Payloads (30kg - 70kg)\Archiv\KR60L30-3_HA_C.vcmx` | 6 | 207,586 | Yes | `(+1.740, +0.000, +1.810)` | None |
| 98 | `Medium Payloads (30kg - 70kg)\Archiv\KR60L30-3_HA_CR.vcmx` | 6 | 207,586 | Yes | `(+1.740, +0.000, +1.810)` | None |
| 99 | `Medium Payloads (30kg - 70kg)\Archiv\KR60L30-4KS.vcmx` | 6 | 269,664 | Yes | `(+1.940, +0.000, +1.245)` | None |
| 100 | `Medium Payloads (30kg - 70kg)\Archiv\KR60L45-3.vcmx` | 6 | 213,295 | Yes | `(+1.540, +0.000, +1.810)` | None |
| 101 | `Medium Payloads (30kg - 70kg)\Archiv\KR60L45-3_C.vcmx` | 6 | 213,295 | Yes | `(+1.540, +0.000, +1.810)` | None |
| 102 | `Medium Payloads (30kg - 70kg)\Archiv\KR60L45-3_HA.vcmx` | 6 | 213,295 | Yes | `(+1.540, +0.000, +1.810)` | None |
| 103 | `Medium Payloads (30kg - 30kg)\Archiv\KR60L45-3_HA_C.vcmx` | 6 | 213,295 | Yes | `(+1.540, +0.000, +1.810)` | None |
| 104 | `Medium Payloads (30kg - 70kg)\Archiv\KR60L45-4_KS.vcmx` | 6 | 282,111 | Yes | `(+1.740, +0.000, +1.245)` | None |
| 105 | `Medium Payloads (30kg - 70kg)\Dummy_JET\KR 30 JET CLG.vcmx` | 6 | 18,534 | Yes | `(+0.990, -0.000, -0.995)` | None |
| 106 | `Medium Payloads (30kg - 70kg)\Dummy_JET\KR 30 JET WALL.vcmx` | 6 | 18,400 | Yes | `(+0.995, +0.000, -0.990)` | None |
| 107 | `Medium Payloads (30kg - 70kg)\Dummy_JET\KR 60 JET CLG.vcmx` | 6 | 18,534 | Yes | `(+0.990, -0.000, -0.995)` | None |
| 108 | `Medium Payloads (30kg - 70kg)\Dummy_JET\KR 60 JET WALL.vcmx` | 6 | 18,400 | Yes | `(+0.995, +0.000, -0.990)` | None |
| 109 | `Medium Payloads (30kg - 70kg)\Dummy_JET\KR 60L30 JET CLG.vcmx` | 6 | 18,534 | Yes | `(+1.390, -0.000, -0.995)` | None |
| 110 | `Medium Payloads (30kg - 70kg)\Dummy_JET\KR 60L30 JET WALL.vcmx` | 6 | 18,400 | Yes | `(+0.995, +0.000, -1.390)` | None |
| 111 | `Medium Payloads (30kg - 70kg)\Dummy_JET\KR 60L45 JET CLG.vcmx` | 6 | 18,534 | Yes | `(+1.190, -0.000, -0.995)` | None |
| 112 | `Medium Payloads (30kg - 70kg)\Dummy_JET\KR 60L45 JET WALL.vcmx` | 6 | 18,400 | Yes | `(+0.995, +0.000, -1.190)` | None |
| 113 | `Medium Payloads (30kg - 70kg)\IONTEC-2 Series\KR 50 R2100-2.vcmx` | 6 | 186,829 | No (rsc) | `(+1.385, +0.000, +1.597)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 114 | `Medium Payloads (30kg - 70kg)\IONTEC-2 Series\KR 50 R2500-2 F.vcmx` | 6 | 193,756 | No (rsc) | `(+1.585, +0.000, +1.797)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 115 | `Medium Payloads (30kg - 70kg)\IONTEC-2 Series\KR 50 R2500-2.vcmx` | 6 | 193,756 | No (rsc) | `(+1.585, +0.000, +1.797)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 116 | `Medium Payloads (30kg - 70kg)\IONTEC-2 Series\KR 70 R2100-2 F.vcmx` | 6 | 186,829 | No (rsc) | `(+1.385, +0.000, +1.597)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 117 | `Medium Payloads (30kg - 70kg)\IONTEC-2 Series\KR 70 R2100-2.vcmx` | 6 | 186,829 | No (rsc) | `(+1.385, +0.000, +1.597)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 118 | `Medium Payloads (30kg - 70kg)\Iontec-Series\KR 50 R2100 E.vcmx` | 6 | 186,829 | No (rsc) | `(+1.385, +0.000, +1.597)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 119 | `Medium Payloads (30kg - 70kg)\Iontec-Series\KR 50 R2500 E.vcmx` | 6 | 193,756 | No (rsc) | `(+1.585, +0.000, +1.797)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 120 | `Medium Payloads (30kg - 70kg)\Iontec-Series\KR 70 R2100 E.vcmx` | 6 | 186,829 | Yes | `(+1.385, +0.000, +1.597)` | None |
| 121 | `Medium Payloads (30kg - 70kg)\Iontec-Series\KR_20_R3100.vcmx` | 6 | 231,572 | No (rsc) | `(+1.963, +0.000, +1.915)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 122 | `Medium Payloads (30kg - 70kg)\Iontec-Series\KR_30_R2100.vcmx` | 6 | 313,916 | No (rsc) | `(+1.395, +0.000, +1.515)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 123 | `Medium Payloads (30kg - 70kg)\Iontec-Series\KR_50_R2100.vcmx` | 6 | 313,916 | No (rsc) | `(+1.395, +0.000, +1.515)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 124 | `Medium Payloads (30kg - 70kg)\Iontec-Series\KR_50_R2500.vcmx` | 6 | 300,195 | No (rsc) | `(+1.395, +0.000, +1.915)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 125 | `Medium Payloads (30kg - 70kg)\Iontec-Series\KR_50_R2500_F.vcmx` | 6 | 301,530 | No (rsc) | `(+1.395, +0.000, +1.915)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 126 | `Medium Payloads (30kg - 70kg)\Iontec-Series\KR_70_R2100.vcmx` | 6 | 316,170 | No (rsc) | `(+1.395, +0.000, +1.515)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
| 127 | `Medium Payloads (30kg - 70kg)\Iontec-Series\KR_70_R2100_F.vcmx` | 6 | 317,515 | No (rsc) | `(+1.395, +0.000, +1.515)` | No embedded _machine.dat/_robcor.dat (using component.rsc limits/defaults) |
