import React, { useCallback, useEffect, useMemo, useRef, useState } from "react";
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { TransformControls } from "three/examples/jsm/controls/TransformControls.js";
import {
  createNeutralStaticPose,
  createPoseApplyCommand,
  getJointLimitsByLeg,
  LEG_MOUNTS_MM,
  legPlaneFootToWorld,
  parseStaticPose,
  POSE_LIBRARY_STORAGE_KEY,
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
const LEG_SCENE_COLORS = {
  front_left: 0x1d8cff,
  front_right: 0x21a264,
  rear_left: 0xff8a21,
  rear_right: 0x8256ff,
};
const LIMITED_SCENE_COLOR = 0xd83a2e;
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

function applyScenePose(robot, solvedPose, selectedLegId) {
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

  for (const legId of LEG_IDS) {
    const leg = solvedPose.legs[legId];
    const visual = robot.legs?.[legId];
    const geometry = leg.preview?.geometry ?? leg.desired?.geometry;
    if (!visual || !geometry) {
      if (visual) {
        visual.root.visible = false;
      }
      continue;
    }

    visual.root.visible = true;
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
    const baseColor = leg.reachable ? LEG_SCENE_COLORS[legId] : LIMITED_SCENE_COLOR;
    const accentColor = isSelected && leg.reachable ? 0xff9f43 : baseColor;

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
    visual.plane.visible = isSelected;
    visual.plane.position.set((minX + maxX) / 2, (minY + maxY) / 2, -0.5);
    visual.plane.scale.set(planeWidth, planeHeight, 1);
    visual.plane.material.color.set(baseColor);
    visual.plane.material.opacity = isSelected ? 0.14 : 0.08;
  }
}

function RobotScene({
  solvedPose,
  selectedLegId,
  editTarget,
  setEditTarget,
  bodyTool,
  setSelectedLegId,
  onFootWorldChange,
  onBodyChange,
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
  const latestRef = useRef({ solvedPose, selectedLegId, editTarget, bodyTool, onFootWorldChange, onBodyChange });

  useEffect(() => {
    latestRef.current = { solvedPose, selectedLegId, editTarget, bodyTool, onFootWorldChange, onBodyChange };
  }, [solvedPose, selectedLegId, editTarget, bodyTool, onFootWorldChange, onBodyChange]);

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
      if (!event.value) {
        flushSceneUpdate();
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
    applyScenePose(robotRef.current, latestRef.current.solvedPose, latestRef.current.selectedLegId);
    onStatus?.("Planar rig ready");

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
    applyScenePose(robotRef.current, solvedPose, selectedLegId);

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

      if (marker) {
        if (!draggingTransformRef.current || editTarget !== EDIT_TARGET.FOOT || !selected) {
          marker.position.copy(vectorToThree(leg.footWorldMm));
        }
        marker.scale.setScalar(selected ? 1.45 : 1);
        marker.material.color.set(leg.reachable ? (selected ? 0x005ee8 : 0x1d8cff) : 0xd83a2e);
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
  }, [solvedPose, selectedLegId, editTarget, bodyTool]);

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
  return (
    <section className="panel">
      <div className="panel-heading">
        <h2>Legs</h2>
        <span className={`count-pill ${solvedPose.reachable ? "count-good" : "count-bad"}`}>
          {solvedPose.reachable ? "clear" : "limited"}
        </span>
      </div>
      <div className="leg-selector">
        {LEG_IDS.map((legId) => (
          <button
            key={legId}
            className={`leg-button ${legId === selectedLegId ? "selected" : ""} ${solvedPose.legs[legId].reachable ? "" : "limited"}`}
            onClick={() => setSelectedLegId(legId)}
          >
            <span>{LEG_LABELS[legId]}</span>
            <strong>{solvedPose.legs[legId].reachable ? "Reachable" : "Limited"}</strong>
          </button>
        ))}
      </div>
    </section>
  );
}

function SceneHandlePanel({ selectedLegId, editTarget, setEditTarget, bodyTool, setBodyTool }) {
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

function LimbKinematicsPanel({ selectedLegId, selectedLeg, onPlaneFootChange }) {
  const svgRef = useRef(null);
  const dragStateRef = useRef({ active: false, pointerId: null });
  const geometry = selectedLeg?.preview?.geometry ?? selectedLeg?.desired?.geometry;
  const linkageGeometry = selectedLeg?.reachable ? selectedLeg?.desired?.geometry : null;

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
    }
  }

  return (
    <section className="panel kinematics-panel">
      <div className="panel-heading">
        <h2>2D Kinematics</h2>
        <span className={`count-pill ${selectedLeg.reachable ? "count-good" : "count-bad"}`}>
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
  const selectedFoot = pose.feetWorldMm[selectedLegId];
  const selectedLeg = solvedPose.legs[selectedLegId];

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
        <span className={`count-pill ${selectedLeg.reachable ? "count-good" : "count-bad"}`}>
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

function ReadoutPanel({ solvedPose, selectedLegId, lastCommand }) {
  const leg = solvedPose.legs[selectedLegId];
  const servoAngles = leg.desired.servoAnglesDeg;
  const commandJointAngles = leg.desired.jointAnglesDeg;
  const previewJointAngles = leg.preview?.jointAnglesDeg ?? commandJointAngles;

  return (
    <section className="panel readout-panel">
      <div className="panel-heading">
        <h2>Readout</h2>
        <span className={`count-pill ${leg.reachable ? "count-good" : "count-bad"}`}>
          {leg.status}
        </span>
      </div>
      <div className="readout-grid">
        <div>
          <span>Foot target</span>
          <strong>{formatNumber(leg.footWorldMm.x)} / {formatNumber(leg.footWorldMm.y)} / {formatNumber(leg.footWorldMm.z)}</strong>
        </div>
        <div>
          <span>Body-local delta</span>
          <strong>{formatNumber(leg.deltaBodyMm.x)} / {formatNumber(leg.deltaBodyMm.y)} / {formatNumber(leg.deltaBodyMm.z)}</strong>
        </div>
        <div>
          <span>Preview joints deg</span>
          <strong>{formatNumber(previewJointAngles.hipYaw)} / {formatNumber(previewJointAngles.thigh)} / {formatNumber(previewJointAngles.calf)}</strong>
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
  const [pose, setPose] = useState(() => createNeutralStaticPose());
  const [poseName, setPoseName] = useState("neutral-stand");
  const [savedPoses, setSavedPoses] = useState(() => loadPoseLibrary());
  const [sceneStatus, setSceneStatus] = useState("Building planar rig");
  const [lastCommand, setLastCommand] = useState(null);
  const [notice, setNotice] = useState("");

  const jointLimitsByLeg = useMemo(() => getJointLimitsByLeg(robotState), [robotState]);
  const solvedPose = useMemo(() => solveStaticPose(pose, { jointLimitsByLeg }), [pose, jointLimitsByLeg]);

  const selectLeg = useCallback((legId) => {
    setSelectedLegId(legId);
    setEditTarget(EDIT_TARGET.FOOT);
  }, []);

  const updateFootWorldFromScene = useCallback((legId, footWorldMm) => {
    setPose((current) => updateFootWorld(current, legId, footWorldMm));
  }, []);

  const updateBodyFromScene = useCallback((bodyPatch) => {
    setPose((current) => updateBodyKeepingFeetLocked(current, bodyPatch));
  }, []);

  const updateLegPlaneFoot = useCallback((legId, footCommand) => {
    setPose((current) => {
      const currentSolved = solveStaticPose(current, { jointLimitsByLeg });
      const hipYawDeg = currentSolved.legs[legId]?.targetHipYawDeg ?? 0;
      return updateFootWorld(current, legId, legPlaneFootToWorld(current, legId, footCommand, hipYawDeg));
    });
  }, [jointLimitsByLeg]);

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

  async function connect() {
    if (!selectedPort) {
      setNotice("Select a serial port first.");
      return;
    }
    await postJson("/api/connect", { path: selectedPort, baudRate: 460800 });
    await fetchStatus();
  }

  async function disconnect() {
    await postJson("/api/disconnect", {});
    await fetchStatus();
  }

  async function applyPose() {
    const command = createPoseApplyCommand(solvedPose, robotState);
    setLastCommand(command);
    await postJson("/api/command", { command });
    setNotice(`Applied ${pose.name}`);
  }

  async function panicRelease() {
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
          onFootWorldChange={updateFootWorldFromScene}
          onBodyChange={updateBodyFromScene}
          onStatus={setSceneStatus}
        />

        <aside className="side-panel-stack">
          <div className="action-bar">
            <button className="primary-button" onClick={withNotice(applyPose)} disabled={!solvedPose.reachable}>Apply Pose</button>
            <button className="secondary-button" onClick={resetPose}>Neutral</button>
            <button className="danger-button" onClick={withNotice(panicRelease)}>Panic Release</button>
          </div>
          {notice ? <div className="notice">{notice}</div> : null}
          <LegSelector selectedLegId={selectedLegId} setSelectedLegId={selectLeg} solvedPose={solvedPose} />
          <SceneHandlePanel
            selectedLegId={selectedLegId}
            editTarget={editTarget}
            setEditTarget={setEditTarget}
            bodyTool={bodyTool}
            setBodyTool={setBodyTool}
          />
          {editTarget === EDIT_TARGET.FOOT ? (
            <LimbKinematicsPanel
              selectedLegId={selectedLegId}
              selectedLeg={solvedPose.legs[selectedLegId]}
              onPlaneFootChange={updateLegPlaneFoot}
            />
          ) : null}
          <PoseControls pose={pose} setPose={setPose} selectedLegId={selectedLegId} solvedPose={solvedPose} />
          <ReadoutPanel solvedPose={solvedPose} selectedLegId={selectedLegId} lastCommand={lastCommand} />
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
