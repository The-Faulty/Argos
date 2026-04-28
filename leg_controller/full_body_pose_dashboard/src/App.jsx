import React, { useCallback, useEffect, useMemo, useRef, useState } from "react";
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { TransformControls } from "three/examples/jsm/controls/TransformControls.js";
import URDFLoader from "urdf-loader";
import {
  createInterpolatedFullBodyCommandSequence,
  createNeutralStaticPose,
  createPoseApplyCommand,
  createRobotStateFullBodyCommand,
  createServoSpeedLimitCommand,
  getJointLimitsByLeg,
  getServoSpeedLimitsByLeg,
  getUrdfJointValues,
  LEG_MOUNTS_MM,
  legPlaneFootToWorld,
  parseStaticPose,
  POSE_LIBRARY_STORAGE_KEY,
  POSE_COMMAND_INTERVAL_MS,
  serializeStaticPose,
  solveStaticPose,
  updateBodyKeepingFeetLocked,
  updateFootWorld,
} from "../shared/pose-builder.js";
import { toCanvasPoint } from "../../robot_dog_debug_dashboard/shared/kinematics.js";
import { LEG_DRAWING, LEG_GEOMETRY, LEG_IDS, LEG_LABELS } from "../../robot_dog_debug_dashboard/shared/robot-config.js";

const BACKEND_DEV_PORT = 8787;
const EDIT_TARGET = {
  FOOT: "foot",
  BODY: "body",
};
const BODY_TOOL = {
  MOVE: "translate",
  ROTATE: "rotate",
};
const LEG_PREFIXES = {
  front_left: "FL",
  front_right: "FR",
  rear_left: "RL",
  rear_right: "RR",
};
const URDF_OVERLAY_OPACITY = 0.32;
const LEG_SCENE_COLORS = {
  front_left: 0x1d8cff,
  front_right: 0x21a264,
  rear_left: 0xff8a21,
  rear_right: 0x8256ff,
};
const CLAMPED_SCENE_COLOR = 0xffb156;
const LIMITED_SCENE_COLOR = 0xd83a2e;
const LIVE_UPDATE_INTERVAL_MS = POSE_COMMAND_INTERVAL_MS;
const POSE_HISTORY_LIMIT = 100;
const BODY_FRAME = (() => {
  const mounts = Object.values(LEG_MOUNTS_MM);
  const minX = Math.min(...mounts.map((mount) => mount.x));
  const maxX = Math.max(...mounts.map((mount) => mount.x));
  const minY = Math.min(...mounts.map((mount) => mount.y));
  const maxY = Math.max(...mounts.map((mount) => mount.y));
  const averageZ = mounts.reduce((sum, mount) => sum + mount.z, 0) / mounts.length;

  return {
    center: {
      x: (minX + maxX) / 2,
      y: (minY + maxY) / 2,
      z: averageZ + 30,
    },
    size: {
      x: (maxX - minX) + 96,
      y: (maxY - minY) + 78,
      z: 58,
    },
  };
})();
const DEFAULT_ROBOT_STATE = {
  connected: false,
  esp32Connected: false,
  connectedPort: null,
  motionMode: "idle",
  mode: "idle",
  ports: [],
  legs: {},
  lastAck: null,
  lastError: null,
};

function stripTrailingSlash(value) {
  return value.replace(/\/+$/, "");
}

function getBackendHttpBaseUrl() {
  const configured = import.meta.env.VITE_BACKEND_URL?.trim();
  if (configured) {
    return stripTrailingSlash(configured);
  }

  if (import.meta.env.DEV && window.location.port !== "5174") {
    return `${window.location.protocol}//${window.location.hostname}:${BACKEND_DEV_PORT}`;
  }

  return "";
}

function getBackendWsUrl() {
  const backendHttpBaseUrl = getBackendHttpBaseUrl();
  if (backendHttpBaseUrl) {
    return `${backendHttpBaseUrl.replace(/^http/i, "ws")}/telemetry`;
  }

  const protocol = window.location.protocol === "https:" ? "wss:" : "ws:";
  return `${protocol}//${window.location.host}/telemetry`;
}

function clone(value) {
  return JSON.parse(JSON.stringify(value));
}

function numberValue(value, fallback = 0) {
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : fallback;
}

function formatNumber(value, digits = 1) {
  return numberValue(value).toFixed(digits);
}

function formatDurationMs(value) {
  return `${(Math.max(0, numberValue(value)) / 1000).toFixed(2)} s`;
}

function sameSpeedLimit(a, b) {
  return (
    numberValue(a?.hipYaw) === numberValue(b?.hipYaw)
    && numberValue(a?.thigh) === numberValue(b?.thigh)
    && numberValue(a?.calf) === numberValue(b?.calf)
  );
}

function cloneSpeedLimitsByLeg(speedLimitsByLeg) {
  return Object.fromEntries(
    LEG_IDS.map((legId) => [
      legId,
      {
        hipYaw: numberValue(speedLimitsByLeg?.[legId]?.hipYaw),
        thigh: numberValue(speedLimitsByLeg?.[legId]?.thigh),
        calf: numberValue(speedLimitsByLeg?.[legId]?.calf),
      },
    ]),
  );
}

function speedLimitFromCommand(command) {
  return {
    hipYaw: command.hipYawDegPerSec,
    thigh: command.thighDegPerSec,
    calf: command.calfDegPerSec,
  };
}

function sleep(ms) {
  return new Promise((resolve) => {
    window.setTimeout(resolve, ms);
  });
}

function isEditableKeyboardTarget(target) {
  if (!(target instanceof HTMLElement)) {
    return false;
  }

  return target.isContentEditable || ["INPUT", "TEXTAREA", "SELECT"].includes(target.tagName);
}

function getLegIndicatorState(leg) {
  if (!leg?.commandReady) {
    return {
      pillClass: "count-bad",
      buttonClass: "limited",
      label: "Unavailable",
    };
  }

  if (leg.clamped) {
    return {
      pillClass: "count-warn",
      buttonClass: "clamped",
      label: "Clamped",
    };
  }

  return {
    pillClass: "count-good",
    buttonClass: "",
    label: "Ready",
  };
}

function getDisplayLegGeometry(leg) {
  if (!leg) {
    return null;
  }

  return leg.desired?.geometry ?? leg.preview?.geometry ?? null;
}

function getDisplayJointAngles(leg) {
  if (!leg) {
    return null;
  }

  return leg.desired?.jointAnglesDeg ?? leg.preview?.jointAnglesDeg ?? null;
}

function getDisplayFootWorldMm(leg) {
  return leg?.displayFootWorldMm ?? leg?.commandFootWorldMm ?? leg?.footWorldMm ?? null;
}

function downloadJson(filename, data) {
  const blob = new Blob([JSON.stringify(data, null, 2)], { type: "application/json" });
  const url = URL.createObjectURL(blob);
  const link = document.createElement("a");
  link.href = url;
  link.download = filename;
  link.click();
  URL.revokeObjectURL(url);
}

function loadPoseLibrary() {
  try {
    const raw = window.localStorage.getItem(POSE_LIBRARY_STORAGE_KEY);
    if (!raw) {
      return [];
    }
    const parsed = JSON.parse(raw);
    const poses = Array.isArray(parsed) ? parsed : parsed.poses;
    return Array.isArray(poses) ? poses.map((pose) => parseStaticPose(JSON.stringify(pose))) : [];
  } catch {
    return [];
  }
}

function savePoseLibrary(poses) {
  window.localStorage.setItem(POSE_LIBRARY_STORAGE_KEY, JSON.stringify(poses, null, 2));
}

function vectorToThree(point) {
  return new THREE.Vector3(point.x, point.y, point.z);
}

function threePositionToMm(position) {
  return {
    x: position.x,
    y: position.y,
    z: position.z,
  };
}

function threeEulerToBodyRotationDeg(rotation) {
  return {
    roll: THREE.MathUtils.radToDeg(rotation.x),
    pitch: THREE.MathUtils.radToDeg(rotation.y),
    yaw: THREE.MathUtils.radToDeg(rotation.z),
  };
}

function fromCanvasPoint(point, drawing = LEG_DRAWING) {
  return {
    x: (point.x - drawing.offset.x) / drawing.scale,
    y: -(point.y - drawing.offset.y) / drawing.scale,
  };
}

function toRelativeFoot(absolutePoint) {
  return {
    x: absolutePoint.x - LEG_GEOMETRY.footOriginOffset.x,
    y: absolutePoint.y - LEG_GEOMETRY.footOriginOffset.y,
  };
}

function segmentPath(a, b) {
  return `M ${a.x} ${a.y} L ${b.x} ${b.y}`;
}

function planePointToLegLocal(point, hip = LEG_GEOMETRY.hipPivot) {
  return new THREE.Vector3(point.x - hip.x, -(point.y - hip.y), 0);
}

function setSegmentBetween(mesh, start, end, radius = 4) {
  if (!mesh) {
    return;
  }

  const direction = new THREE.Vector3().subVectors(end, start);
  const length = Math.max(direction.length(), 0.001);
  mesh.visible = length > 0.001;
  mesh.position.copy(start).addScaledVector(direction, 0.5);
  mesh.scale.set(radius, length, radius);
  mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), direction.normalize());
}

function createLegVisual(legId) {
  const root = new THREE.Group();
  root.position.copy(vectorToThree(LEG_MOUNTS_MM[legId]));

  const plane = new THREE.Mesh(
    new THREE.PlaneGeometry(1, 1),
    new THREE.MeshBasicMaterial({
      color: LEG_SCENE_COLORS[legId],
      transparent: true,
      opacity: 0.08,
      depthWrite: false,
      side: THREE.DoubleSide,
    }),
  );
  plane.visible = false;
  plane.renderOrder = 2;
  root.add(plane);

  const jointMaterial = new THREE.MeshStandardMaterial({
    color: LEG_SCENE_COLORS[legId],
    roughness: 0.38,
    metalness: 0.1,
    depthTest: false,
  });
  const jointGeometry = new THREE.SphereGeometry(5.6, 18, 14);
  const hipJoint = new THREE.Mesh(jointGeometry, jointMaterial.clone());
  const kneeJoint = new THREE.Mesh(jointGeometry, jointMaterial.clone());
  const footJoint = new THREE.Mesh(
    new THREE.SphereGeometry(6.6, 20, 16),
    jointMaterial.clone(),
  );
  hipJoint.renderOrder = 6;
  kneeJoint.renderOrder = 6;
  footJoint.renderOrder = 7;
  root.add(hipJoint, kneeJoint, footJoint);

  const segmentGeometry = new THREE.CylinderGeometry(1, 1, 1, 12);
  const thighBone = new THREE.Mesh(
    segmentGeometry,
    new THREE.MeshStandardMaterial({
      color: LEG_SCENE_COLORS[legId],
      roughness: 0.32,
      metalness: 0.14,
      depthTest: false,
    }),
  );
  const calfBone = new THREE.Mesh(
    segmentGeometry,
    new THREE.MeshStandardMaterial({
      color: LEG_SCENE_COLORS[legId],
      roughness: 0.42,
      metalness: 0.08,
      depthTest: false,
    }),
  );
  thighBone.renderOrder = 5;
  calfBone.renderOrder = 5;
  root.add(thighBone, calfBone);

  return {
    root,
    plane,
    hipJoint,
    kneeJoint,
    footJoint,
    thighBone,
    calfBone,
  };
}

function applyUrdfOverlayPose(robot, solvedPose, showUrdfOverlay) {
  if (!robot?.urdf || !solvedPose) {
    return;
  }

  robot.urdf.visible = showUrdfOverlay;
  if (!showUrdfOverlay) {
    return;
  }

  const { positionMm, rotationDeg } = solvedPose.pose.body;
  robot.urdf.position.set(positionMm.x, positionMm.y, positionMm.z);
  robot.urdf.rotation.set(
    THREE.MathUtils.degToRad(rotationDeg.roll),
    THREE.MathUtils.degToRad(rotationDeg.pitch),
    THREE.MathUtils.degToRad(rotationDeg.yaw),
    "XYZ",
  );

  const jointValues = getUrdfJointValues(solvedPose);
  for (const [jointName, jointValue] of Object.entries(jointValues)) {
    robot.urdf.joints?.[jointName]?.setJointValue(jointValue);
  }
}

function applyScenePose(robot, solvedPose, selectedLegId, showPlanarRig, showUrdfOverlay) {
  if (!robot || !solvedPose) {
    return;
  }

  const { positionMm, rotationDeg } = solvedPose.pose.body;
  robot.root.position.set(positionMm.x, positionMm.y, positionMm.z);
  robot.root.rotation.set(
    THREE.MathUtils.degToRad(rotationDeg.roll),
    THREE.MathUtils.degToRad(rotationDeg.pitch),
    THREE.MathUtils.degToRad(rotationDeg.yaw),
    "XYZ",
  );
  robot.bodyShell.visible = showPlanarRig;
  robot.bodyOutline.visible = showPlanarRig;

  for (const legId of LEG_IDS) {
    const leg = solvedPose.legs[legId];
    const visual = robot.legs?.[legId];
    const geometry = getDisplayLegGeometry(leg);
    if (!visual || !geometry) {
      if (visual) {
        visual.root.visible = false;
      }
      continue;
    }

    visual.root.visible = showPlanarRig;
    visual.root.rotation.set(
      THREE.MathUtils.degToRad(leg.targetHipYawDeg ?? 0) - (Math.PI / 2),
      0,
      0,
      "XYZ",
    );

    const hipLocal = planePointToLegLocal(geometry.hip ?? LEG_GEOMETRY.hipPivot, geometry.hip ?? LEG_GEOMETRY.hipPivot);
    const kneeLocal = planePointToLegLocal(geometry.knee ?? geometry.hip ?? LEG_GEOMETRY.hipPivot, geometry.hip ?? LEG_GEOMETRY.hipPivot);
    const footLocal = planePointToLegLocal(geometry.foot ?? geometry.hip ?? LEG_GEOMETRY.hipPivot, geometry.hip ?? LEG_GEOMETRY.hipPivot);
    const isSelected = legId === selectedLegId;
    const baseColor = !leg.commandReady
      ? LIMITED_SCENE_COLOR
      : (leg.clamped ? CLAMPED_SCENE_COLOR : LEG_SCENE_COLORS[legId]);
    const accentColor = isSelected ? CLAMPED_SCENE_COLOR : baseColor;

    setSegmentBetween(visual.thighBone, hipLocal, kneeLocal, isSelected ? 4.6 : 4);
    setSegmentBetween(visual.calfBone, kneeLocal, footLocal, isSelected ? 4.2 : 3.6);

    visual.hipJoint.position.copy(hipLocal);
    visual.kneeJoint.position.copy(kneeLocal);
    visual.footJoint.position.copy(footLocal);

    visual.thighBone.material.color.set(accentColor);
    visual.calfBone.material.color.set(baseColor);
    visual.hipJoint.material.color.set(0x1a2430);
    visual.kneeJoint.material.color.set(accentColor);
    visual.footJoint.material.color.set(baseColor);

    const localPoints = [hipLocal, kneeLocal, footLocal];
    const minX = Math.min(...localPoints.map((point) => point.x));
    const maxX = Math.max(...localPoints.map((point) => point.x));
    const minY = Math.min(...localPoints.map((point) => point.y));
    const maxY = Math.max(...localPoints.map((point) => point.y));
    const planeWidth = Math.max(72, (maxX - minX) + 52);
    const planeHeight = Math.max(118, (maxY - minY) + 56);
    visual.plane.visible = showPlanarRig && isSelected;
    visual.plane.position.set((minX + maxX) / 2, (minY + maxY) / 2, -0.5);
    visual.plane.scale.set(planeWidth, planeHeight, 1);
    visual.plane.material.color.set(baseColor);
    visual.plane.material.opacity = isSelected ? 0.14 : 0.08;
  }

  applyUrdfOverlayPose(robot, solvedPose, showUrdfOverlay);
}

function RobotScene({
  solvedPose,
  selectedLegId,
  editTarget,
  setEditTarget,
  bodyTool,
  setSelectedLegId,
  showPlanarRig,
  showUrdfOverlay,
  onFootWorldChange,
  onBodyChange,
  onEditStart,
  onEditEnd,
  onStatus,
}) {
  const mountRef = useRef(null);
  const robotRef = useRef(null);
  const bodyHandleRef = useRef(null);
  const transformControlsRef = useRef(null);
  const footMarkersRef = useRef({});
  const draggingTransformRef = useRef(false);
  const syncingSceneRef = useRef(false);
  const pendingSceneUpdateRef = useRef(null);
  const sceneCommitFrameRef = useRef(0);
  const latestRef = useRef({
    solvedPose,
    selectedLegId,
    editTarget,
    bodyTool,
    showPlanarRig,
    showUrdfOverlay,
    onFootWorldChange,
    onBodyChange,
    onEditStart,
    onEditEnd,
  });

  useEffect(() => {
    latestRef.current = {
      solvedPose,
      selectedLegId,
      editTarget,
      bodyTool,
      showPlanarRig,
      showUrdfOverlay,
      onFootWorldChange,
      onBodyChange,
      onEditStart,
      onEditEnd,
    };
  }, [solvedPose, selectedLegId, editTarget, bodyTool, showPlanarRig, showUrdfOverlay, onFootWorldChange, onBodyChange, onEditStart, onEditEnd]);

  useEffect(() => {
    const mount = mountRef.current;
    if (!mount) {
      return undefined;
    }

    function flushSceneUpdate() {
      if (sceneCommitFrameRef.current) {
        window.cancelAnimationFrame(sceneCommitFrameRef.current);
        sceneCommitFrameRef.current = 0;
      }

      const pending = pendingSceneUpdateRef.current;
      pendingSceneUpdateRef.current = null;
      if (!pending) {
        return;
      }

      const latest = latestRef.current;
      if (pending.type === EDIT_TARGET.BODY) {
        latest.onBodyChange?.(pending.bodyPatch);
      } else {
        latest.onFootWorldChange?.(pending.legId, pending.footWorldMm);
      }
    }

    function scheduleSceneUpdate(update) {
      pendingSceneUpdateRef.current = update;
      if (sceneCommitFrameRef.current) {
        return;
      }

      sceneCommitFrameRef.current = window.requestAnimationFrame(() => {
        sceneCommitFrameRef.current = 0;
        flushSceneUpdate();
      });
    }

    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0xe9eef2);

    const camera = new THREE.PerspectiveCamera(45, 1, 1, 5000);
    camera.up.set(0, 0, 1);
    camera.position.set(420, -520, 300);

    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.outputColorSpace = THREE.SRGBColorSpace;
    mount.appendChild(renderer.domElement);

    const controls = new OrbitControls(camera, renderer.domElement);
    controls.target.set(45, -10, -90);
    controls.enableDamping = true;
    controls.dampingFactor = 0.08;

    scene.add(new THREE.HemisphereLight(0xffffff, 0x758190, 2.4));
    const keyLight = new THREE.DirectionalLight(0xffffff, 2.2);
    keyLight.position.set(180, -260, 420);
    scene.add(keyLight);
    const fillLight = new THREE.DirectionalLight(0x8fb8ff, 0.75);
    fillLight.position.set(-280, 200, 240);
    scene.add(fillLight);

    const grid = new THREE.GridHelper(620, 31, 0x8c98a4, 0xcbd3dc);
    grid.rotation.x = Math.PI / 2;
    grid.position.z = -165;
    scene.add(grid);

    const footGroup = new THREE.Group();
    scene.add(footGroup);

    for (const legId of LEG_IDS) {
      const markerGeometry = new THREE.SphereGeometry(8, 24, 16);
      const markerMaterial = new THREE.MeshStandardMaterial({
        color: 0x1d8cff,
        roughness: 0.44,
        metalness: 0.05,
        depthTest: false,
      });
      const marker = new THREE.Mesh(markerGeometry, markerMaterial);
      marker.userData.legId = legId;
      marker.renderOrder = 20;
      footMarkersRef.current[legId] = marker;
      footGroup.add(marker);
    }

    const robotRoot = new THREE.Group();
    const bodyShell = new THREE.Mesh(
      new THREE.BoxGeometry(BODY_FRAME.size.x, BODY_FRAME.size.y, BODY_FRAME.size.z),
      new THREE.MeshStandardMaterial({
        color: 0xf4f7fb,
        transparent: true,
        opacity: 0.78,
        roughness: 0.42,
        metalness: 0.08,
      }),
    );
    bodyShell.position.copy(vectorToThree(BODY_FRAME.center));
    const bodyOutline = new THREE.LineSegments(
      new THREE.EdgesGeometry(bodyShell.geometry),
      new THREE.LineBasicMaterial({ color: 0x2d3a48, transparent: true, opacity: 0.7 }),
    );
    bodyOutline.position.copy(bodyShell.position);
    robotRoot.add(bodyShell, bodyOutline);

    const legVisuals = {};
    for (const legId of LEG_IDS) {
      const legVisual = createLegVisual(legId);
      legVisuals[legId] = legVisual;
      robotRoot.add(legVisual.root);
    }
    robotRef.current = {
      root: robotRoot,
      bodyShell,
      bodyOutline,
      legs: legVisuals,
      urdf: null,
    };
    scene.add(robotRoot);

    const bodyMaterial = new THREE.MeshStandardMaterial({
      color: 0xff8a21,
      emissive: 0x6a2600,
      emissiveIntensity: 0.25,
      roughness: 0.38,
      metalness: 0.12,
      depthTest: false,
    });
    const bodyHandle = new THREE.Mesh(new THREE.OctahedronGeometry(16, 1), bodyMaterial);
    bodyHandle.userData.editTarget = EDIT_TARGET.BODY;
    bodyHandle.renderOrder = 30;
    bodyHandle.rotation.order = "XYZ";
    const bodyHalo = new THREE.Mesh(
      new THREE.TorusGeometry(24, 1.5, 8, 48),
      new THREE.MeshBasicMaterial({ color: 0xffb156, depthTest: false }),
    );
    bodyHalo.userData.editTarget = EDIT_TARGET.BODY;
    bodyHalo.rotation.x = Math.PI / 2;
    bodyHalo.renderOrder = 31;
    bodyHandle.add(bodyHalo);
    bodyHandleRef.current = bodyHandle;
    scene.add(bodyHandle);

    const transformControls = new TransformControls(camera, renderer.domElement);
    transformControls.setSize(0.82);
    transformControls.setTranslationSnap(1);
    transformControls.setRotationSnap(THREE.MathUtils.degToRad(1));
    transformControls.addEventListener("dragging-changed", (event) => {
      draggingTransformRef.current = event.value;
      controls.enabled = !event.value;
      if (event.value) {
        latestRef.current.onEditStart?.();
      }
      if (!event.value) {
        flushSceneUpdate();
        latestRef.current.onEditEnd?.();
      }
    });
    transformControls.addEventListener("objectChange", () => {
      if (syncingSceneRef.current) {
        return;
      }

      const latest = latestRef.current;
      if (latest.editTarget === EDIT_TARGET.BODY) {
        const handle = bodyHandleRef.current;
        if (handle) {
          scheduleSceneUpdate({
            type: EDIT_TARGET.BODY,
            bodyPatch: {
            positionMm: threePositionToMm(handle.position),
            rotationDeg: threeEulerToBodyRotationDeg(handle.rotation),
            },
          });
        }
        return;
      }

      const marker = footMarkersRef.current[latest.selectedLegId];
      if (marker) {
        scheduleSceneUpdate({
          type: EDIT_TARGET.FOOT,
          legId: latest.selectedLegId,
          footWorldMm: threePositionToMm(marker.position),
        });
      }
    });
    transformControlsRef.current = transformControls;
    scene.add(transformControls.getHelper());
    applyScenePose(
      robotRef.current,
      latestRef.current.solvedPose,
      latestRef.current.selectedLegId,
      latestRef.current.showPlanarRig,
      latestRef.current.showUrdfOverlay,
    );
    onStatus?.("Planar rig ready / loading URDF overlay");

    const loader = new URDFLoader(new THREE.LoadingManager());
    loader.packages = {
      Argos_description: "/argos_description",
    };
    loader.load(
      "/argos_description/urdf/Argos.urdf",
      (urdfRobot) => {
        urdfRobot.scale.setScalar(1000);
        urdfRobot.traverse((child) => {
          if (child.isMesh) {
            child.castShadow = false;
            child.receiveShadow = false;
            child.renderOrder = 18;
            child.material = child.material.clone();
            child.material.transparent = true;
            child.material.opacity = URDF_OVERLAY_OPACITY;
            child.material.depthWrite = false;
            child.material.depthTest = false;
            child.material.roughness = 0.74;
            child.material.metalness = 0.02;
            child.material.color.set(0x1f2730);
          }
        });
        robotRef.current.urdf = urdfRobot;
        scene.add(urdfRobot);
        applyUrdfOverlayPose(robotRef.current, latestRef.current.solvedPose, latestRef.current.showUrdfOverlay);
        onStatus?.("Planar rig + URDF overlay ready");
      },
      undefined,
      (error) => {
        onStatus?.(`Planar rig ready / URDF overlay error: ${error.message}`);
      },
    );

    const raycaster = new THREE.Raycaster();
    const pointer = new THREE.Vector2();

    function getPickData(object) {
      let current = object;
      while (current) {
        if (current.userData?.legId || current.userData?.editTarget) {
          return current.userData;
        }
        current = current.parent;
      }
      return null;
    }

    function resize() {
      const { width, height } = mount.getBoundingClientRect();
      camera.aspect = width / Math.max(1, height);
      camera.updateProjectionMatrix();
      renderer.setSize(width, height, false);
    }

    function handlePointerDown(event) {
      const rect = renderer.domElement.getBoundingClientRect();
      pointer.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
      pointer.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;
      raycaster.setFromCamera(pointer, camera);
      const pickables = [...Object.values(footMarkersRef.current), bodyHandleRef.current].filter(Boolean);
      const intersections = raycaster.intersectObjects(pickables, true);
      const pickData = intersections.map((intersection) => getPickData(intersection.object)).find(Boolean);

      if (pickData?.legId) {
        setSelectedLegId(pickData.legId);
        setEditTarget(EDIT_TARGET.FOOT);
      } else if (pickData?.editTarget === EDIT_TARGET.BODY) {
        setEditTarget(EDIT_TARGET.BODY);
      }
    }

    let animationFrame = 0;
    function animate() {
      controls.update();
      renderer.render(scene, camera);
      animationFrame = window.requestAnimationFrame(animate);
    }

    resize();
    animate();
    window.addEventListener("resize", resize);
    renderer.domElement.addEventListener("pointerdown", handlePointerDown);

    return () => {
      window.cancelAnimationFrame(animationFrame);
      if (sceneCommitFrameRef.current) {
        window.cancelAnimationFrame(sceneCommitFrameRef.current);
        sceneCommitFrameRef.current = 0;
      }
      pendingSceneUpdateRef.current = null;
      window.removeEventListener("resize", resize);
      renderer.domElement.removeEventListener("pointerdown", handlePointerDown);
      transformControls.detach();
      transformControls.dispose();
      controls.dispose();
      renderer.dispose();
      mount.removeChild(renderer.domElement);
    };
  }, [onStatus, setEditTarget, setSelectedLegId]);

  useEffect(() => {
    syncingSceneRef.current = true;
    applyScenePose(robotRef.current, solvedPose, selectedLegId, showPlanarRig, showUrdfOverlay);

    const robot = robotRef.current;
    if (robot?.bodyShell && robot?.bodyOutline) {
      robot.bodyShell.material.color.set(editTarget === EDIT_TARGET.BODY ? 0xfff0dd : 0xf4f7fb);
      robot.bodyOutline.material.color.set(editTarget === EDIT_TARGET.BODY ? 0xff8a21 : 0x2d3a48);
    }

    const bodyHandle = bodyHandleRef.current;
    if (bodyHandle && (!draggingTransformRef.current || editTarget !== EDIT_TARGET.BODY)) {
      const { positionMm, rotationDeg } = solvedPose.pose.body;
      bodyHandle.position.set(positionMm.x, positionMm.y, positionMm.z);
      bodyHandle.rotation.set(
        THREE.MathUtils.degToRad(rotationDeg.roll),
        THREE.MathUtils.degToRad(rotationDeg.pitch),
        THREE.MathUtils.degToRad(rotationDeg.yaw),
        "XYZ",
      );
      bodyHandle.material.color.set(editTarget === EDIT_TARGET.BODY ? 0xff8a21 : 0x3d4c5a);
    }

    for (const legId of LEG_IDS) {
      const leg = solvedPose.legs[legId];
      const marker = footMarkersRef.current[legId];
      const selected = legId === selectedLegId;
      const displayFootWorldMm = getDisplayFootWorldMm(leg);
      const markerColor = !leg.commandReady
        ? LIMITED_SCENE_COLOR
        : (leg.clamped ? CLAMPED_SCENE_COLOR : (selected ? 0x005ee8 : 0x1d8cff));

      if (marker) {
        if (
          !draggingTransformRef.current
          || editTarget !== EDIT_TARGET.FOOT
          || !selected
          || leg.clamped
          || !leg.commandReady
        ) {
          marker.position.copy(vectorToThree(displayFootWorldMm ?? leg.footWorldMm));
        }
        marker.scale.setScalar(selected ? 1.45 : 1);
        marker.material.color.set(markerColor);
      }
    }

    const transformControls = transformControlsRef.current;
    if (transformControls) {
      const targetObject = editTarget === EDIT_TARGET.BODY
        ? bodyHandleRef.current
        : footMarkersRef.current[selectedLegId];

      if (targetObject) {
        if (transformControls.object !== targetObject) {
          transformControls.attach(targetObject);
        }
        transformControls.setMode(editTarget === EDIT_TARGET.BODY ? bodyTool : BODY_TOOL.MOVE);
        transformControls.setSpace(editTarget === EDIT_TARGET.BODY && bodyTool === BODY_TOOL.ROTATE ? "local" : "world");
        transformControls.showX = true;
        transformControls.showY = true;
        transformControls.showZ = true;
        transformControls.enabled = true;
      } else {
        transformControls.detach();
      }
    }
    syncingSceneRef.current = false;
  }, [solvedPose, selectedLegId, editTarget, bodyTool, showPlanarRig, showUrdfOverlay]);

  return <div ref={mountRef} className="scene-viewport" />;
}

function NumberField({ label, value, onChange, step = 1, min, max }) {
  return (
    <label className="number-field">
      <span>{label}</span>
      <input
        type="number"
        step={step}
        min={min}
        max={max}
        value={Number.isFinite(value) ? value : 0}
        onChange={(event) => onChange(numberValue(event.target.value))}
      />
    </label>
  );
}

function AxisEditor({ title, values, labels = ["X", "Y", "Z"], onChange, step = 1 }) {
  const keys = ["x", "y", "z"];
  return (
    <section className="tool-block">
      <h3>{title}</h3>
      <div className="axis-grid">
        {keys.map((key, index) => (
          <NumberField
            key={key}
            label={labels[index]}
            value={values[key]}
            step={step}
            onChange={(value) => onChange(key, value)}
          />
        ))}
      </div>
    </section>
  );
}

function ConnectionBar({ robotState, selectedPort, setSelectedPort, connect, disconnect, refreshStatus, sceneStatus }) {
  const connected = Boolean(robotState.esp32Connected ?? robotState.connected);
  return (
    <header className="top-bar">
      <div>
        <p className="eyebrow">Argos</p>
        <h1>Full-Body Pose Builder</h1>
      </div>
      <div className="status-strip">
        <span className={`status-pill ${connected ? "status-good" : "status-warn"}`}>
          {connected ? "ESP32 online" : "ESP32 offline"}
        </span>
        <span className="status-pill">Mode {robotState.motionMode ?? robotState.mode ?? "idle"}</span>
        <span className="status-pill">{sceneStatus}</span>
      </div>
      <div className="connection-controls">
        <select value={selectedPort} onChange={(event) => setSelectedPort(event.target.value)}>
          <option value="">Serial port</option>
          {robotState.ports.map((port) => (
            <option key={port.path} value={port.path}>
              {port.path} {port.manufacturer ? `- ${port.manufacturer}` : ""}
            </option>
          ))}
        </select>
        <button className="secondary-button" onClick={refreshStatus}>Refresh</button>
        {connected ? (
          <button className="secondary-button" onClick={disconnect}>Disconnect</button>
        ) : (
          <button className="primary-button" onClick={connect} disabled={!selectedPort}>Connect</button>
        )}
      </div>
    </header>
  );
}

function PoseLibraryPanel({ pose, poseName, setPoseName, savedPoses, savePose, loadPose, deletePose, importPose, exportPose, exportLibrary }) {
  const importInputRef = useRef(null);

  return (
    <section className="panel pose-library-panel">
      <div className="panel-heading">
        <h2>Pose Library</h2>
        <span className="count-pill">{savedPoses.length}</span>
      </div>
      <label className="stacked-field">
        <span>Name</span>
        <input value={poseName} onChange={(event) => setPoseName(event.target.value)} />
      </label>
      <div className="button-row">
        <button className="primary-button" onClick={savePose}>Save</button>
        <button className="secondary-button" onClick={() => exportPose(pose)}>Export</button>
        <button className="secondary-button" onClick={exportLibrary}>Export All</button>
        <button className="secondary-button" onClick={() => importInputRef.current?.click()}>Import</button>
      </div>
      <input
        ref={importInputRef}
        className="hidden-input"
        type="file"
        accept="application/json,.json"
        onChange={importPose}
      />
      <div className="saved-list">
        {savedPoses.length === 0 ? (
          <p className="muted">No saved poses yet.</p>
        ) : (
          savedPoses.map((savedPose) => (
            <div key={savedPose.name} className="saved-row">
              <button className="text-button" onClick={() => loadPose(savedPose)}>{savedPose.name}</button>
              <button className="danger-text-button" onClick={() => deletePose(savedPose.name)}>Delete</button>
            </div>
          ))
        )}
      </div>
    </section>
  );
}

function LegSelector({ selectedLegId, setSelectedLegId, solvedPose }) {
  const overallPillClass = !solvedPose.commandReady
    ? "count-bad"
    : (solvedPose.reachable ? "count-good" : "count-warn");
  const overallLabel = !solvedPose.commandReady
    ? "fault"
    : (solvedPose.reachable ? "clear" : "clamped");

  return (
    <section className="panel">
      <div className="panel-heading">
        <h2>Legs</h2>
        <span className={`count-pill ${overallPillClass}`}>
          {overallLabel}
        </span>
      </div>
      <div className="leg-selector">
        {LEG_IDS.map((legId) => {
          const indicator = getLegIndicatorState(solvedPose.legs[legId]);
          return (
            <button
              key={legId}
              className={`leg-button ${legId === selectedLegId ? "selected" : ""} ${indicator.buttonClass}`}
              onClick={() => setSelectedLegId(legId)}
            >
              <span>{LEG_LABELS[legId]}</span>
              <strong>{indicator.label}</strong>
            </button>
          );
        })}
      </div>
    </section>
  );
}

function SceneHandlePanel({
  selectedLegId,
  editTarget,
  setEditTarget,
  bodyTool,
  setBodyTool,
  showPlanarRig,
  setShowPlanarRig,
  showUrdfOverlay,
  setShowUrdfOverlay,
}) {
  return (
    <section className="panel handle-panel">
      <div className="panel-heading">
        <h2>Scene Handles</h2>
        <span className="count-pill">{editTarget === EDIT_TARGET.BODY ? "body" : "foot"}</span>
      </div>
      <div className="segmented-control">
        <button
          className={editTarget === EDIT_TARGET.FOOT ? "selected" : ""}
          onClick={() => setEditTarget(EDIT_TARGET.FOOT)}
        >
          {LEG_LABELS[selectedLegId]} Foot
        </button>
        <button
          className={editTarget === EDIT_TARGET.BODY ? "selected" : ""}
          onClick={() => setEditTarget(EDIT_TARGET.BODY)}
        >
          Body Node
        </button>
      </div>
      <label className="toggle-row">
        <input
          type="checkbox"
          checked={showPlanarRig}
          onChange={(event) => setShowPlanarRig(event.target.checked)}
        />
        <span>Simplified 3D Rig</span>
      </label>
      <label className="toggle-row">
        <input
          type="checkbox"
          checked={showUrdfOverlay}
          onChange={(event) => setShowUrdfOverlay(event.target.checked)}
        />
        <span>URDF Overlay</span>
      </label>
      {editTarget === EDIT_TARGET.BODY ? (
        <div className="segmented-control">
          <button
            className={bodyTool === BODY_TOOL.MOVE ? "selected" : ""}
            onClick={() => setBodyTool(BODY_TOOL.MOVE)}
          >
            Move Body
          </button>
          <button
            className={bodyTool === BODY_TOOL.ROTATE ? "selected" : ""}
            onClick={() => setBodyTool(BODY_TOOL.ROTATE)}
          >
            Rotate Body
          </button>
        </div>
      ) : null}
    </section>
  );
}

function LimbKinematicsPanel({
  selectedLegId,
  selectedLeg,
  onPlaneFootChange,
  onEditStart,
  onEditEnd,
}) {
  const svgRef = useRef(null);
  const dragStateRef = useRef({ active: false, pointerId: null });
  const geometry = getDisplayLegGeometry(selectedLeg);
  const linkageGeometry = selectedLeg?.commandReady ? selectedLeg?.desired?.geometry : null;
  const indicator = getLegIndicatorState(selectedLeg);

  if (!geometry) {
    return (
      <section className="panel kinematics-panel">
        <div className="panel-heading">
          <h2>2D Kinematics</h2>
          <span className="count-pill">{LEG_LABELS[selectedLegId]}</span>
        </div>
        <p className="muted">No limb geometry available.</p>
      </section>
    );
  }

  const p = (point) => toCanvasPoint(point, LEG_DRAWING);
  const points = {
    hip: p(geometry.hip),
    knee: p(geometry.knee),
    foot: p(geometry.foot),
  };
  const linkagePoints = linkageGeometry
    ? {
        servoPivot: p(linkageGeometry.servoPivot),
        servoHornEnd: p(linkageGeometry.servoHornEnd),
        bellArmA: p(linkageGeometry.bellArmA),
        bellArmB: p(linkageGeometry.bellArmB),
        calfAttach: p(linkageGeometry.calfAttach),
      }
    : null;

  function pointerToFootCommand(event) {
    const svg = svgRef.current;
    if (!svg) {
      return null;
    }

    const rect = svg.getBoundingClientRect();
    const scaleX = LEG_DRAWING.stageWidth / rect.width;
    const scaleY = LEG_DRAWING.stageHeight / rect.height;
    const canvasPoint = {
      x: (event.clientX - rect.left) * scaleX,
      y: (event.clientY - rect.top) * scaleY,
    };
    return toRelativeFoot(fromCanvasPoint(canvasPoint));
  }

  function startDrag(event) {
    onEditStart?.();
    dragStateRef.current = { active: true, pointerId: event.pointerId };
    svgRef.current?.setPointerCapture(event.pointerId);
    const nextFoot = pointerToFootCommand(event);
    if (nextFoot) {
      onPlaneFootChange(selectedLegId, nextFoot);
    }
  }

  function moveDrag(event) {
    if (!dragStateRef.current.active) {
      return;
    }

    const nextFoot = pointerToFootCommand(event);
    if (nextFoot) {
      onPlaneFootChange(selectedLegId, nextFoot);
    }
  }

  function endDrag(event) {
    if (dragStateRef.current.active && dragStateRef.current.pointerId === event.pointerId) {
      dragStateRef.current = { active: false, pointerId: null };
      onEditEnd?.();
    }
  }

  return (
    <section className="panel kinematics-panel">
      <div className="panel-heading">
        <h2>2D Kinematics</h2>
        <span className={`count-pill ${indicator.pillClass}`}>
          {LEG_LABELS[selectedLegId]}
        </span>
      </div>
      <svg
        ref={svgRef}
        className="leg-svg leg-svg-interactive"
        viewBox={`0 0 ${LEG_DRAWING.stageWidth} ${LEG_DRAWING.stageHeight}`}
        onPointerMove={moveDrag}
        onPointerUp={endDrag}
        onPointerCancel={endDrag}
        onPointerLeave={endDrag}
      >
        <line x1="0" y1={LEG_DRAWING.offset.y} x2={LEG_DRAWING.stageWidth} y2={LEG_DRAWING.offset.y} className="svg-axis" />
        <line x1={LEG_DRAWING.offset.x} y1="0" x2={LEG_DRAWING.offset.x} y2={LEG_DRAWING.stageHeight} className="svg-axis" />
        <path d={segmentPath(points.hip, points.knee)} stroke="#17202a" strokeWidth="5" fill="none" strokeLinecap="round" />
        <path d={segmentPath(points.knee, points.foot)} stroke="#1d8cff" strokeWidth="5" fill="none" strokeLinecap="round" />
        {linkagePoints ? (
          <>
            <path d={segmentPath(points.hip, linkagePoints.bellArmA)} stroke="#21a264" strokeWidth="4" fill="none" strokeLinecap="round" />
            <path d={segmentPath(points.hip, linkagePoints.bellArmB)} stroke="#21a264" strokeWidth="4" fill="none" strokeLinecap="round" />
            <path d={segmentPath(linkagePoints.servoPivot, linkagePoints.servoHornEnd)} stroke="#d9432e" strokeWidth="4" fill="none" strokeLinecap="round" />
            <path d={segmentPath(linkagePoints.servoHornEnd, linkagePoints.bellArmA)} stroke="#c36a16" strokeWidth="4" fill="none" strokeLinecap="round" />
            <path d={segmentPath(linkagePoints.bellArmB, linkagePoints.calfAttach)} stroke="#6f4cff" strokeWidth="4" fill="none" strokeLinecap="round" />
          </>
        ) : null}
        {Object.entries(points).map(([key, point]) => (
          <circle
            key={key}
            cx={point.x}
            cy={point.y}
            r={key === "foot" ? 11 : 5}
            fill={key === "foot" ? "#1d8cff" : "#17202a"}
            className={key === "foot" ? "draggable-foot" : undefined}
            onPointerDown={key === "foot" ? startDrag : undefined}
          />
        ))}
      </svg>
      <div className="kinematics-readout">
        <div>
          <span>Plane target</span>
          <strong>{formatNumber(selectedLeg.footCommand.x)} / {formatNumber(selectedLeg.footCommand.y)}</strong>
        </div>
        <div>
          <span>Hip yaw</span>
          <strong>{formatNumber(selectedLeg.targetHipYawDeg)} deg</strong>
        </div>
      </div>
    </section>
  );
}

function PoseControls({ pose, setPose, selectedLegId, solvedPose }) {
  const selectedFoot = getDisplayFootWorldMm(solvedPose.legs[selectedLegId]) ?? pose.feetWorldMm[selectedLegId];
  const selectedLeg = solvedPose.legs[selectedLegId];
  const indicator = getLegIndicatorState(selectedLeg);

  function updateBodyVector(group, key, value) {
    setPose((current) => updateBodyKeepingFeetLocked(current, { [group]: { [key]: value } }));
  }

  function updateFoot(key, value) {
    setPose((current) => updateFootWorld(current, selectedLegId, { [key]: value }));
  }

  return (
    <section className="panel controls-panel">
      <div className="panel-heading">
        <h2>Pose Controls</h2>
        <span className={`count-pill ${indicator.pillClass}`}>
          {LEG_LABELS[selectedLegId]}
        </span>
      </div>
      <AxisEditor
        title="Body Position"
        values={pose.body.positionMm}
        labels={["X mm", "Y mm", "Z mm"]}
        onChange={(key, value) => updateBodyVector("positionMm", key, value)}
      />
      <AxisEditor
        title="Body Angle"
        values={{
          x: pose.body.rotationDeg.roll,
          y: pose.body.rotationDeg.pitch,
          z: pose.body.rotationDeg.yaw,
        }}
        labels={["Roll", "Pitch", "Yaw"]}
        step={0.5}
        onChange={(key, value) => {
          const map = { x: "roll", y: "pitch", z: "yaw" };
          updateBodyVector("rotationDeg", map[key], value);
        }}
      />
      <AxisEditor
        title={`${LEG_LABELS[selectedLegId]} Foot`}
        values={selectedFoot}
        labels={["X mm", "Y mm", "Z mm"]}
        onChange={updateFoot}
      />
    </section>
  );
}

function CommandMotionPanel({
  selectedLegId,
  speedLimitDraft,
  activeSpeedLimit,
  estimatedApplyDurationMs,
  estimatedApplyFrames,
  applyInFlight,
  onSpeedLimitChange,
  onUpdateSelectedSpeedLimit,
  onCopySpeedLimitToAll,
}) {
  const draft = speedLimitDraft ?? activeSpeedLimit;
  const active = activeSpeedLimit ?? draft;

  return (
    <section className="panel controls-panel">
      <div className="panel-heading">
        <h2>Command Motion</h2>
        <span className="count-pill">
          {estimatedApplyFrames}f
        </span>
      </div>
      <section className="tool-block">
        <h3>{LEG_LABELS[selectedLegId]} Speed Limit</h3>
        <div className="axis-grid">
          <NumberField
            label="Hip yaw deg/s"
            value={draft.hipYaw}
            min={1}
            onChange={(value) => onSpeedLimitChange(selectedLegId, "hipYaw", value)}
          />
          <NumberField
            label="Thigh deg/s"
            value={draft.thigh}
            min={1}
            onChange={(value) => onSpeedLimitChange(selectedLegId, "thigh", value)}
          />
          <NumberField
            label="Calf deg/s"
            value={draft.calf}
            min={1}
            onChange={(value) => onSpeedLimitChange(selectedLegId, "calf", value)}
          />
        </div>
        <div className="motion-readout-grid">
          <div>
            <span>Active</span>
            <strong>{formatNumber(active.hipYaw, 0)} / {formatNumber(active.thigh, 0)} / {formatNumber(active.calf, 0)}</strong>
          </div>
          <div>
            <span>Apply Time</span>
            <strong>{formatDurationMs(estimatedApplyDurationMs)}</strong>
          </div>
          <div>
            <span>Frame Cadence</span>
            <strong>{POSE_COMMAND_INTERVAL_MS} ms</strong>
          </div>
        </div>
        <div className="button-row">
          <button className="secondary-button" onClick={onUpdateSelectedSpeedLimit} disabled={applyInFlight}>Update Speed</button>
          <button className="secondary-button" onClick={onCopySpeedLimitToAll} disabled={applyInFlight}>Copy To All</button>
        </div>
      </section>
    </section>
  );
}

function ReadoutPanel({ solvedPose, selectedLegId, lastCommand, commandPreview }) {
  const leg = solvedPose.legs[selectedLegId];
  const commandJointAngles = leg.desired.jointAnglesDeg;
  const displayJointAngles = getDisplayJointAngles(leg) ?? commandJointAngles;
  const displayFootWorldMm = getDisplayFootWorldMm(leg) ?? leg.footWorldMm;
  const indicator = getLegIndicatorState(leg);
  const prefix = LEG_PREFIXES[selectedLegId];
  const servoAngles = commandPreview && prefix
    ? {
        hipYaw: commandPreview[`${prefix}HipYawDeg`],
        thigh: commandPreview[`${prefix}ThighDeg`],
        calf: commandPreview[`${prefix}CalfDeg`],
      }
    : leg.desired.servoAnglesDeg;

  return (
    <section className="panel readout-panel">
      <div className="panel-heading">
        <h2>Readout</h2>
        <span className={`count-pill ${indicator.pillClass}`}>
          {leg.status}
        </span>
      </div>
      <div className="readout-grid">
        <div>
          <span>Foot target</span>
          <strong>{formatNumber(leg.footWorldMm.x)} / {formatNumber(leg.footWorldMm.y)} / {formatNumber(leg.footWorldMm.z)}</strong>
        </div>
        <div>
          <span>Displayed foot</span>
          <strong>{formatNumber(displayFootWorldMm.x)} / {formatNumber(displayFootWorldMm.y)} / {formatNumber(displayFootWorldMm.z)}</strong>
        </div>
        <div>
          <span>Body-local delta</span>
          <strong>{formatNumber(leg.deltaBodyMm.x)} / {formatNumber(leg.deltaBodyMm.y)} / {formatNumber(leg.deltaBodyMm.z)}</strong>
        </div>
        <div>
          <span>Displayed joints deg</span>
          <strong>{formatNumber(displayJointAngles.hipYaw)} / {formatNumber(displayJointAngles.thigh)} / {formatNumber(displayJointAngles.calf)}</strong>
        </div>
        <div>
          <span>Command joints deg</span>
          <strong>{formatNumber(commandJointAngles.hipYaw)} / {formatNumber(commandJointAngles.thigh)} / {formatNumber(commandJointAngles.calf)}</strong>
        </div>
        <div>
          <span>Servos deg</span>
          <strong>{formatNumber(servoAngles.hipYaw)} / {formatNumber(servoAngles.thigh)} / {formatNumber(servoAngles.calf)}</strong>
        </div>
      </div>
      {lastCommand ? (
        <pre className="command-preview">{JSON.stringify(lastCommand, null, 2)}</pre>
      ) : null}
    </section>
  );
}

export default function App() {
  const [robotState, setRobotState] = useState(DEFAULT_ROBOT_STATE);
  const [selectedPort, setSelectedPort] = useState("");
  const [selectedLegId, setSelectedLegId] = useState("front_left");
  const [editTarget, setEditTarget] = useState(EDIT_TARGET.FOOT);
  const [bodyTool, setBodyTool] = useState(BODY_TOOL.ROTATE);
  const [showPlanarRig, setShowPlanarRig] = useState(true);
  const [showUrdfOverlay, setShowUrdfOverlay] = useState(true);
  const [liveUpdateEnabled, setLiveUpdateEnabled] = useState(false);
  const [applyInFlight, setApplyInFlight] = useState(false);
  const [pose, setPose] = useState(() => createNeutralStaticPose());
  const [poseName, setPoseName] = useState("neutral-stand");
  const [savedPoses, setSavedPoses] = useState(() => loadPoseLibrary());
  const [sceneStatus, setSceneStatus] = useState("Building planar rig");
  const [lastCommand, setLastCommand] = useState(null);
  const [notice, setNotice] = useState("");
  const [speedLimitDrafts, setSpeedLimitDrafts] = useState(() => getServoSpeedLimitsByLeg(DEFAULT_ROBOT_STATE));
  const poseRef = useRef(pose);
  const poseUndoStackRef = useRef([]);
  const poseEditSessionRef = useRef(null);
  const latestLiveCommandRef = useRef(null);
  const lastSentLiveCommandKeyRef = useRef("");
  const liveCommandInFlightRef = useRef(false);
  const applySequenceInFlightRef = useRef(false);
  const cancelApplySequenceRef = useRef(false);
  const syncedSpeedLimitsRef = useRef(getServoSpeedLimitsByLeg(DEFAULT_ROBOT_STATE));

  const jointLimitsByLeg = useMemo(() => getJointLimitsByLeg(robotState), [robotState]);
  const servoSpeedLimitsByLeg = useMemo(() => getServoSpeedLimitsByLeg(robotState), [robotState]);
  const solvedPose = useMemo(() => solveStaticPose(pose, { jointLimitsByLeg }), [pose, jointLimitsByLeg]);
  const connectedToRobot = Boolean(robotState.esp32Connected ?? robotState.connected);
  const liveCommand = useMemo(
    () => (solvedPose.commandReady ? createPoseApplyCommand(solvedPose, robotState) : null),
    [robotState, solvedPose],
  );
  const currentRobotCommand = useMemo(() => (
    createRobotStateFullBodyCommand(robotState, "current")
    ?? createRobotStateFullBodyCommand(robotState, "desired")
    ?? (lastCommand?.type === "apply_full_body_pose" ? lastCommand : null)
  ), [lastCommand, robotState]);
  const applySequence = useMemo(
    () => createInterpolatedFullBodyCommandSequence(
      currentRobotCommand,
      liveCommand,
      servoSpeedLimitsByLeg,
      { frameIntervalMs: POSE_COMMAND_INTERVAL_MS },
    ),
    [currentRobotCommand, liveCommand, servoSpeedLimitsByLeg],
  );
  const liveCommandKey = useMemo(
    () => (liveCommand ? JSON.stringify(liveCommand) : ""),
    [liveCommand],
  );

  useEffect(() => {
    poseRef.current = pose;
  }, [pose]);

  const clearPoseHistory = useCallback(() => {
    poseUndoStackRef.current = [];
    poseEditSessionRef.current = null;
  }, []);

  const endPoseEditSession = useCallback(() => {
    poseEditSessionRef.current = null;
  }, []);

  const beginPoseEditSession = useCallback(() => {
    if (poseEditSessionRef.current) {
      return;
    }

    poseEditSessionRef.current = {
      before: clone(poseRef.current),
      recorded: false,
    };
  }, []);

  const applyPoseEdit = useCallback((updater) => {
    setPose((current) => {
      const next = typeof updater === "function" ? updater(current) : updater;
      const currentSnapshot = serializeStaticPose(current);
      const nextSnapshot = serializeStaticPose(next);

      if (currentSnapshot === nextSnapshot) {
        return current;
      }

      const activeSession = poseEditSessionRef.current;
      if (activeSession) {
        if (!activeSession.recorded) {
          poseUndoStackRef.current.push(activeSession.before);
          activeSession.recorded = true;
        }
      } else {
        poseUndoStackRef.current.push(clone(current));
      }

      if (poseUndoStackRef.current.length > POSE_HISTORY_LIMIT) {
        poseUndoStackRef.current.splice(0, poseUndoStackRef.current.length - POSE_HISTORY_LIMIT);
      }

      return next;
    });
  }, []);

  const undoLastPoseEdit = useCallback(() => {
    if (applyInFlight) {
      return false;
    }

    endPoseEditSession();
    const previousPose = poseUndoStackRef.current.pop();
    if (!previousPose) {
      return false;
    }

    setPose(previousPose);
    return true;
  }, [applyInFlight, endPoseEditSession]);

  const selectLeg = useCallback((legId) => {
    setSelectedLegId(legId);
    setEditTarget(EDIT_TARGET.FOOT);
  }, []);

  const updateFootWorldFromScene = useCallback((legId, footWorldMm) => {
    applyPoseEdit((current) => updateFootWorld(current, legId, footWorldMm));
  }, [applyPoseEdit]);

  const updateBodyFromScene = useCallback((bodyPatch) => {
    applyPoseEdit((current) => updateBodyKeepingFeetLocked(current, bodyPatch));
  }, [applyPoseEdit]);

  const updateLegPlaneFoot = useCallback((legId, footCommand) => {
    applyPoseEdit((current) => {
      const currentSolved = solveStaticPose(current, { jointLimitsByLeg });
      const hipYawDeg = currentSolved.legs[legId]?.targetHipYawDeg ?? 0;
      return updateFootWorld(current, legId, legPlaneFootToWorld(current, legId, footCommand, hipYawDeg));
    });
  }, [applyPoseEdit, jointLimitsByLeg]);

  async function postJson(path, payload) {
    const response = await fetch(`${getBackendHttpBaseUrl()}${path}`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload),
    });
    const body = await response.json().catch(() => ({}));
    if (!response.ok) {
      throw new Error(body.error || `Request failed with ${response.status}`);
    }
    return body;
  }

  async function fetchStatus() {
    const response = await fetch(`${getBackendHttpBaseUrl()}/api/status`);
    const payload = await response.json();
    setRobotState((current) => ({ ...current, ...payload }));
    setSelectedPort((current) => current || payload.connectedPort || "");
  }

  function updateSpeedLimitDraft(legId, jointId, value) {
    setSpeedLimitDrafts((current) => ({
      ...current,
      [legId]: {
        ...current[legId],
        [jointId]: Math.max(1, numberValue(value, current[legId]?.[jointId] ?? 1)),
      },
    }));
  }

  async function sendServoSpeedLimit(legId, speedLimitDraft) {
    const command = createServoSpeedLimitCommand(legId, speedLimitDraft);
    const normalized = speedLimitFromCommand(command);

    setRobotState((current) => ({
      ...current,
      legs: {
        ...current.legs,
        [legId]: {
          ...current.legs?.[legId],
          servoSpeedLimitDegPerSec: normalized,
        },
      },
    }));
    setSpeedLimitDrafts((current) => ({
      ...current,
      [legId]: normalized,
    }));

    await postJson("/api/command", { command });
    setNotice(`Updated ${LEG_LABELS[legId]} speed limit`);
  }

  async function copySpeedLimitToAll() {
    const source = speedLimitDrafts[selectedLegId] ?? servoSpeedLimitsByLeg[selectedLegId];
    const commands = LEG_IDS.map((legId) => createServoSpeedLimitCommand(legId, source));
    const normalized = speedLimitFromCommand(commands[0]);

    setRobotState((current) => ({
      ...current,
      legs: Object.fromEntries(
        LEG_IDS.map((legId) => [
          legId,
          {
            ...current.legs?.[legId],
            servoSpeedLimitDegPerSec: normalized,
          },
        ]),
      ),
    }));
    setSpeedLimitDrafts((current) => ({
      ...current,
      ...Object.fromEntries(LEG_IDS.map((legId) => [legId, normalized])),
    }));

    await Promise.all(commands.map((command) => postJson("/api/command", { command })));
    setNotice(`Copied ${LEG_LABELS[selectedLegId]} speed limit to all legs`);
  }

  useEffect(() => {
    fetchStatus().catch((error) => setNotice(error.message));
    const socket = new WebSocket(getBackendWsUrl());

    socket.onmessage = (event) => {
      try {
        const message = JSON.parse(event.data);
        if (message.type === "status" || message.type === "state" || message.type === "hello_ack") {
          setRobotState((current) => ({ ...current, ...message.payload }));
          setSelectedPort((current) => current || message.payload?.connectedPort || "");
        }
        if (message.type === "ack") {
          setRobotState((current) => ({ ...current, lastAck: message.message ?? String(message.seq ?? "ack") }));
        }
        if (message.type === "error") {
          setNotice(message.message || "Controller error");
        }
      } catch (error) {
        setNotice(error.message);
      }
    };
    socket.onerror = () => setNotice("Bridge websocket unavailable");

    return () => socket.close();
  }, []);

  useEffect(() => {
    savePoseLibrary(savedPoses);
  }, [savedPoses]);

  useEffect(() => {
    setSpeedLimitDrafts((current) => {
      let changed = false;
      const next = { ...current };

      for (const legId of LEG_IDS) {
        const synced = syncedSpeedLimitsRef.current[legId];
        const incoming = servoSpeedLimitsByLeg[legId];
        const draft = current[legId];
        const shouldSync = !draft || sameSpeedLimit(draft, synced);
        if (shouldSync && !sameSpeedLimit(draft, incoming)) {
          next[legId] = { ...incoming };
          changed = true;
        }
      }

      syncedSpeedLimitsRef.current = cloneSpeedLimitsByLeg(servoSpeedLimitsByLeg);
      return changed ? next : current;
    });
  }, [servoSpeedLimitsByLeg]);

  useEffect(() => {
    function handleKeyDown(event) {
      if (
        !(event.ctrlKey || event.metaKey)
        || event.shiftKey
        || event.altKey
        || event.key.toLowerCase() !== "z"
        || isEditableKeyboardTarget(event.target)
      ) {
        return;
      }

      if (undoLastPoseEdit()) {
        event.preventDefault();
        setNotice("Undid pose change");
      }
    }

    window.addEventListener("keydown", handleKeyDown);
    return () => {
      window.removeEventListener("keydown", handleKeyDown);
    };
  }, [undoLastPoseEdit]);

  useEffect(() => {
    latestLiveCommandRef.current = {
      command: liveCommand,
      key: liveCommandKey,
      commandReady: solvedPose.commandReady,
      connected: connectedToRobot,
    };
  }, [connectedToRobot, liveCommand, liveCommandKey, solvedPose.commandReady]);

  useEffect(() => {
    if (!liveUpdateEnabled) {
      liveCommandInFlightRef.current = false;
      return undefined;
    }

    const intervalId = window.setInterval(() => {
      const latest = latestLiveCommandRef.current;
      if (!latest?.connected || !latest.commandReady || !latest.command) {
        return;
      }
      if (
        applySequenceInFlightRef.current
        || liveCommandInFlightRef.current
        || latest.key === lastSentLiveCommandKeyRef.current
      ) {
        return;
      }

      liveCommandInFlightRef.current = true;
      postJson("/api/command", { command: latest.command })
        .then(() => {
          setLastCommand(latest.command);
          lastSentLiveCommandKeyRef.current = latest.key;
        })
        .catch((error) => {
          setLiveUpdateEnabled(false);
          setNotice(`Live update stopped: ${error.message}`);
        })
        .finally(() => {
          liveCommandInFlightRef.current = false;
        });
    }, LIVE_UPDATE_INTERVAL_MS);

    return () => {
      window.clearInterval(intervalId);
    };
  }, [liveUpdateEnabled]);

  async function connect() {
    if (!selectedPort) {
      setNotice("Select a serial port first.");
      return;
    }
    await postJson("/api/connect", { path: selectedPort, baudRate: 921600 });
    await fetchStatus();
  }

  async function disconnect() {
    await postJson("/api/disconnect", {});
    await fetchStatus();
  }

  async function applyPose() {
    if (!liveCommand) {
      throw new Error("Pose is outside the available joint range.");
    }

    const commandPlan = createInterpolatedFullBodyCommandSequence(
      currentRobotCommand,
      liveCommand,
      servoSpeedLimitsByLeg,
      { frameIntervalMs: POSE_COMMAND_INTERVAL_MS },
    );

    cancelApplySequenceRef.current = false;
    applySequenceInFlightRef.current = true;
    setApplyInFlight(true);

    try {
      for (let index = 0; index < commandPlan.commands.length; index += 1) {
        if (cancelApplySequenceRef.current) {
          break;
        }

        await postJson("/api/command", { command: commandPlan.commands[index] });

        if (index < commandPlan.commands.length - 1) {
          await sleep(commandPlan.frameIntervalMs);
        }
      }

      if (cancelApplySequenceRef.current) {
        return;
      }

      setLastCommand(liveCommand);
      lastSentLiveCommandKeyRef.current = liveCommandKey;
      setNotice(
        commandPlan.commands.length > 1
          ? `Applied ${pose.name} over ${formatDurationMs(commandPlan.durationMs)}`
          : `Applied ${pose.name}`,
      );
    } finally {
      applySequenceInFlightRef.current = false;
      cancelApplySequenceRef.current = false;
      setApplyInFlight(false);
    }
  }

  async function panicRelease() {
    cancelApplySequenceRef.current = true;
    setLiveUpdateEnabled(false);
    lastSentLiveCommandKeyRef.current = "";
    await postJson("/api/command", { command: { type: "panic_release" } });
    setNotice("Servos released");
  }

  function withNotice(handler) {
    return (...args) => {
      Promise.resolve(handler(...args)).catch((error) => setNotice(error.message));
    };
  }

  function resetPose() {
    const neutral = createNeutralStaticPose();
    clearPoseHistory();
    setPose(neutral);
    setPoseName(neutral.name);
    setLastCommand(null);
  }

  function savePose() {
    const namedPose = { ...pose, name: poseName || pose.name };
    const normalized = parseStaticPose(serializeStaticPose(namedPose), pose);
    setPose(normalized);
    setPoseName(normalized.name);
    setSavedPoses((current) => [
      normalized,
      ...current.filter((savedPose) => savedPose.name !== normalized.name),
    ]);
    setNotice(`Saved ${normalized.name}`);
  }

  function loadPose(savedPose) {
    const nextPose = parseStaticPose(serializeStaticPose(savedPose), pose);
    clearPoseHistory();
    setPose(nextPose);
    setPoseName(nextPose.name);
    setLastCommand(null);
  }

  function deletePose(name) {
    setSavedPoses((current) => current.filter((savedPose) => savedPose.name !== name));
  }

  async function importPose(event) {
    const file = event.target.files?.[0];
    event.target.value = "";
    if (!file) {
      return;
    }

    const text = await file.text();
    const parsed = JSON.parse(text);
    const incoming = Array.isArray(parsed) ? parsed : parsed.poses ?? [parsed];
    const imported = incoming.map((item) => parseStaticPose(JSON.stringify(item), pose));
    setSavedPoses((current) => {
      const withoutImported = current.filter((savedPose) => !imported.some((item) => item.name === savedPose.name));
      return [...imported, ...withoutImported];
    });
    if (imported[0]) {
      clearPoseHistory();
      setPose(imported[0]);
      setPoseName(imported[0].name);
    }
    setNotice(`Imported ${imported.length} pose${imported.length === 1 ? "" : "s"}`);
  }

  function exportPose(exportedPose) {
    downloadJson(`${exportedPose.name || "argos-pose"}.json`, parseStaticPose(serializeStaticPose(exportedPose), pose));
  }

  function exportLibrary() {
    downloadJson("argos-pose-library.json", { version: 1, poses: savedPoses.map((savedPose) => clone(savedPose)) });
  }

  return (
    <main className="app-shell">
      <ConnectionBar
        robotState={robotState}
        selectedPort={selectedPort}
        setSelectedPort={setSelectedPort}
        connect={withNotice(connect)}
        disconnect={withNotice(disconnect)}
        refreshStatus={withNotice(fetchStatus)}
        sceneStatus={sceneStatus}
      />

      <section className="workspace">
        <RobotScene
          solvedPose={solvedPose}
          selectedLegId={selectedLegId}
          editTarget={editTarget}
          setEditTarget={setEditTarget}
          bodyTool={bodyTool}
          setSelectedLegId={selectLeg}
          showPlanarRig={showPlanarRig}
          showUrdfOverlay={showUrdfOverlay}
          onFootWorldChange={updateFootWorldFromScene}
          onBodyChange={updateBodyFromScene}
          onEditStart={beginPoseEditSession}
          onEditEnd={endPoseEditSession}
          onStatus={setSceneStatus}
        />

        <aside className="side-panel-stack">
          <div className="action-bar">
            <button
              className="primary-button"
              onClick={withNotice(applyPose)}
              disabled={!solvedPose.commandReady || applyInFlight}
            >
              {applyInFlight ? "Applying..." : "Apply Pose"}
            </button>
            <button className="secondary-button" onClick={resetPose}>Neutral</button>
            <button className="danger-button" onClick={withNotice(panicRelease)}>Panic Release</button>
            <label className="action-toggle">
              <input
                type="checkbox"
                checked={liveUpdateEnabled}
                disabled={applyInFlight}
                onChange={(event) => {
                  lastSentLiveCommandKeyRef.current = "";
                  setLiveUpdateEnabled(event.target.checked);
                  setNotice(
                    event.target.checked
                      ? (connectedToRobot ? "Live update enabled" : "Live update armed; connect the robot to stream poses")
                      : "Live update disabled",
                  );
                }}
              />
              <span>Live Update</span>
            </label>
          </div>
          {notice ? <div className="notice">{notice}</div> : null}
          <LegSelector selectedLegId={selectedLegId} setSelectedLegId={selectLeg} solvedPose={solvedPose} />
          <SceneHandlePanel
            selectedLegId={selectedLegId}
            editTarget={editTarget}
            setEditTarget={setEditTarget}
            bodyTool={bodyTool}
            setBodyTool={setBodyTool}
            showPlanarRig={showPlanarRig}
            setShowPlanarRig={setShowPlanarRig}
            showUrdfOverlay={showUrdfOverlay}
            setShowUrdfOverlay={setShowUrdfOverlay}
          />
          {editTarget === EDIT_TARGET.FOOT ? (
            <LimbKinematicsPanel
              selectedLegId={selectedLegId}
              selectedLeg={solvedPose.legs[selectedLegId]}
              onPlaneFootChange={updateLegPlaneFoot}
              onEditStart={beginPoseEditSession}
              onEditEnd={endPoseEditSession}
            />
          ) : null}
          <PoseControls pose={pose} setPose={applyPoseEdit} selectedLegId={selectedLegId} solvedPose={solvedPose} />
          <CommandMotionPanel
            selectedLegId={selectedLegId}
            speedLimitDraft={speedLimitDrafts[selectedLegId]}
            activeSpeedLimit={servoSpeedLimitsByLeg[selectedLegId]}
            estimatedApplyDurationMs={applySequence.durationMs}
            estimatedApplyFrames={applySequence.commands.length}
            applyInFlight={applyInFlight}
            onSpeedLimitChange={updateSpeedLimitDraft}
            onUpdateSelectedSpeedLimit={withNotice(() => sendServoSpeedLimit(
              selectedLegId,
              speedLimitDrafts[selectedLegId] ?? servoSpeedLimitsByLeg[selectedLegId],
            ))}
            onCopySpeedLimitToAll={withNotice(copySpeedLimitToAll)}
          />
          <ReadoutPanel
            solvedPose={solvedPose}
            selectedLegId={selectedLegId}
            lastCommand={lastCommand}
            commandPreview={liveCommand}
          />
          <PoseLibraryPanel
            pose={pose}
            poseName={poseName}
            setPoseName={setPoseName}
            savedPoses={savedPoses}
            savePose={savePose}
            loadPose={loadPose}
            deletePose={deletePose}
            importPose={withNotice(importPose)}
            exportPose={exportPose}
            exportLibrary={exportLibrary}
          />
        </aside>
      </section>
    </main>
  );
}
