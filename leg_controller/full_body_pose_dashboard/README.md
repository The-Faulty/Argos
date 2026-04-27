# Argos Full-Body Pose Dashboard

Static pose workbench for Argos full-body positioning. The app reuses the existing dashboard bridge and shared kinematics, renders the provided URDF/STL model, keeps feet locked in world space while body position and attitude change, and sends an explicit full-body pose only when Apply Pose is pressed.

## Run

From `leg_controller/full_body_pose_dashboard`, use the launcher to start the existing serial bridge and this dashboard:

```powershell
.\launch.ps1
```

Or from `leg_controller`:

```powershell
.\launch_full_body_pose_dashboard.ps1
```

Manual startup still works if you prefer separate terminals. Start the existing bridge from `leg_controller/robot_dog_debug_dashboard`:

```bash
npm run bridge
```

Then start this app from `leg_controller/full_body_pose_dashboard`:

```bash
npm run dev
```

```text
http://localhost:5174
```

## Verify

```bash
npm test
npm run build
```
