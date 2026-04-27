// Print the joint angles the gait planner's IK produces at zero-twist neutral
// stance — both per-leg radians and degrees. Used to diagnose "the coxa
// starts in a weird position": if the IK output for the stance feet has a
// non-zero coxa, that's the position the leg snaps to as soon as the
// operator enters trot/crawl, even before pushing the joystick.

import { fourLegsInverseKinematics } from "../dashboard/shared/argos_kinematics.js";
import { LEG_IDS } from "../dashboard/shared/robot-config.js";

// Same constants as gait_planner.js makeStanceFeet().
const STAND_FRONT_FOOT_X =  0.1135;
const STAND_REAR_FOOT_X  = -0.1100;
const STAND_FOOT_Y       =  0.0733;
const STAND_FOOT_Z       = -0.1900;

const stanceFeet = LEG_IDS.map((id) => {
  const isFront = id === "FR" || id === "FL";
  const isLeft = id === "FL" || id === "RL";
  const x = isFront ? STAND_FRONT_FOOT_X : STAND_REAR_FOOT_X;
  const y = isLeft ? STAND_FOOT_Y : -STAND_FOOT_Y;
  return [x, y, STAND_FOOT_Z];
});

console.log("# Gait stance feet (body frame, m)");
for (let i = 0; i < LEG_IDS.length; i++) {
  const f = stanceFeet[i];
  console.log(`  ${LEG_IDS[i]}: (${f[0].toFixed(4)}, ${f[1].toFixed(4)}, ${f[2].toFixed(4)})`);
}

const ik = fourLegsInverseKinematics(stanceFeet);
if (!ik.ok) {
  console.error(`IK failed for ${ik.leg}: ${ik.reason}`);
  process.exit(1);
}

console.log("");
console.log("# IK-computed neutral joint angles");
console.log("#   leg     coxa(°)    femur(°)    tibia(°)");
for (let i = 0; i < LEG_IDS.length; i++) {
  const [c, f, t] = ik.angles[i];
  const r2d = 180 / Math.PI;
  console.log(`  ${LEG_IDS[i]}    ${(c * r2d).toFixed(3).padStart(7)}    ${(f * r2d).toFixed(3).padStart(7)}    ${(t * r2d).toFixed(3).padStart(7)}`);
}

console.log("");
console.log("# STAND_JOINTS_RAD (used by stand/extend mode, no IK):");
console.log("#   coxa=0.000°  femur=-28.080°  tibia=28.980°");
console.log("");
console.log("# If the IK coxa above is non-zero, switching from stand → trot");
console.log("# will jump every leg's coxa from 0 to that value.");
