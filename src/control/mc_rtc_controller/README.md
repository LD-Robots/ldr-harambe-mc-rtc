# mc_rtc_controller — Harambe Robot

mc_rtc CoM + Posture controller for the Harambe humanoid (37 DOF: legs + waist + arms + hands).

## Quick Start

```bash
# 1. Generate static URDF from xacro (requires ROS workspace sourced)
source install/setup.bash
cd src/control/mc_rtc_controller/scripts
./generate_urdf.sh

# 2. Build controller
cd ..
cmake -S . -B build
cmake --build build

# 3. Install config symlinks
cd scripts
./install.sh          # user configs
sudo ./install.sh     # mc_mujoco robot config (needs sudo for /usr/local)

# 4. Run
mc_mujoco --sync
```

## Controller: HarambeCoMPosture

QP-based whole-body controller with:
- **PostureTask** — drives all 37 joints to reference positions
- **CoMTask** — tracks center of mass with configurable offset
- **TorsoOrientationTask** — stabilizes torso orientation
- **StabilizerTask** — LIPM balance with foot contacts

Config: `config/HarambeCoMPosture.yaml`

## Files

| File | Purpose |
|------|---------|
| `mc_com_posture_controller.cpp/h` | Controller implementation |
| `config/mc_rtc.yaml` | Main mc_rtc config |
| `config/HarambeCoMPosture.yaml` | Controller tuning parameters |
| `config/harambe.json` | Robot module (joints, sensors, surfaces) |
| `config/harambe.rsdf` | Foot contact surface geometry |
| `config/mc_mujoco.yaml` | MuJoCo visualization settings |
| `config/Harambe.yaml` | MuJoCo model path |
| `scripts/generate_urdf.sh` | Generate URDF from xacro |
| `scripts/install.sh` | Symlink configs to ~/.config/mc_rtc/ |
