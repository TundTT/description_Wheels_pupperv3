# Wheeled Pupper V3 - Description & RL Assets

This repository contains the description files (URDF, MJCF), scripts, and assets for the **Wheeled Pupper V3** robot. It represents a specialized fork of the original Pupper V3 description, modified to support **wheeled-legged locomotion** in Reinforcement Learning (RL) environments.

## 🛞 From Legged to Wheeled: Key Modifications
This repository implements the conversion of a standard quadruped into a wheeled-legged robot. The primary contributions and edits include:

### 1. Mechanical & URDF Overhaul
- **End-Effector Swap:** Replaced the standard point-contact feet with **wheel geometries** in the URDF/XML description.
- **Continuous Rotation:** Modified the distal joints (formerly ankles) to support **continuous rotation** (infinite joint limits), enabling the robot to drive via rolling rather than stepping.

### 2. Hybrid "Split" Control Architecture
Unlike the standard position-controlled quadruped, this robot uses a hybrid control scheme:
- **Legs:** Remain under **Position Control** (impedance) to maintain stance height and posture.
- **Wheels:** Converted to **Velocity/Torque Control**. This allows the RL policy to output direct driving velocities or torques, decoupling the drive-train dynamics from the postural dynamics.
- **Actuator Tuning:** Adjusted `ctrlrange` and gains (Kp/Kd) in `actuators.xml` and `Wheel_pupper.xml` to support higher-speed rolling behaviors and remove default position-holding damping on the wheels.

### 3. Simulation & RL Integration
- **Action Space Adaptation:** Updated the RL policy configuration (in `Colab_wheeled.ipynb`) to handle the mixed action space (Position targets for legs, Velocity targets for wheels).
- **Environment Updates:** Configured the MuJoCo environment/XMLs to handle the unique physics of wheeled locomotion, including friction adjustments and infinite rotation handling.

---

## 📂 Repository Structure

- **`description/`**: Core assets.
  - `urdf/`: URDF files exported from CAD and processed.
  - `mujoco_xml/`: Generated MuJoCo XML model files (e.g., `Wheel_pupper.xml`, `pupper_v3_complete.xml`).
  - `meshes/`: STL/OBJ mesh files.
- **`scripts/`**: Utilities related to model generation and asset management.
  - `fix_urdf.py`: Cleans up URDFs exported from CAD tools (e.g., Phobos/Blender).
  - `create_mujoco_xml.py`: Composes the final MuJoCo XML by combining the robot body with sensors, actuators, and environment settings.
- **`VIBE FILES AND NOTES/`**:
  - `car.xml`: Simplified car model used for isolating and testing wheel control logic.
  - `notes.txt`: Design notes on control strategies (Velocity vs Motor), action scaling, and implementation details.

## 🚀 Workflows

### 1. Generating MuJoCo Models
To generate a complete MuJoCo XML model from the base components:

```bash
python3 scripts/create_mujoco_xml.py --xml_dir description/mujoco_xml
```

**Key Arguments:**
- `--mjx`: Generate a model compatible with MJX (hardware-accelerated MuJoCo).
- `--fixed_base`: Generate a model with the torso fixed in space (debug mode).
- `--position_control`: (Optional) Flag to force position control on all joints (Note: Wheeled Pupper typically uses the custom split control defined in the XMLs).

### 2. Integration with RL (Colab)
The generated XML files (specifically `Wheel_pupper.xml`) are designed for the RL training pipeline (`Colab_wheeled.ipynb`).
- **Dependency Fixes:** This repo includes fixes for JAX/Orbax version compatibility issues found in the original pipelines.
- **Loading:** Ensure the `Colab_wheeled.ipynb` loads the local `Wheel_pupper.xml` to utilize the custom wheel actuators.

### 3. Usage with ROS 2 (Visualization)
- **Build:** `colcon build`
- **Visualize:** `ros2 launch pupper_v3_description display_model.launch.py`

## 🛠️ Modification Guide

### Adjusting Actuators
To tune the wheel speed or torque:
1.  Edit `description/mujoco_xml/actuators.xml` or the specific `<actuator>` tags in `Wheel_pupper.xml`.
2.  Modify `ctrlrange` to increase max speed/torque.
3.  Adjust `kp` (gain) and `limit` attributes to refine the split control behavior.