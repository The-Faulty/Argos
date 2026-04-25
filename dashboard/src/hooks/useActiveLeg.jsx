// Selected-leg state — which of FR/FL/RR/RL is the "active" leg in the UI.
//
// Every per-leg panel (LegDetail, JointPanel, FootXYZPanel, ServoPanel)
// reads from the same state so the main SVG and the slider stacks stay in
// sync without per-component prop drilling. Keep it dead-simple: a React
// Context wrapping a useState of the current leg id.
//
// Implementation note: we persist the choice in sessionStorage so a refresh
// mid-debug keeps focus on the leg you were working on.

import { createContext, useContext, useEffect, useMemo, useState } from "react";

const DEFAULT_LEG = "FR";
const STORAGE_KEY = "argos.activeLeg";

const ActiveLegContext = createContext(null);

function readInitial() {
  if (typeof window === "undefined") return DEFAULT_LEG;
  try {
    const stored = window.sessionStorage.getItem(STORAGE_KEY);
    if (stored && ["FR", "FL", "RR", "RL"].includes(stored)) return stored;
  } catch {
    /* sessionStorage may be unavailable in private mode */
  }
  return DEFAULT_LEG;
}

export function ActiveLegProvider({ children }) {
  const [activeLeg, setActiveLeg] = useState(readInitial);

  useEffect(() => {
    try {
      window.sessionStorage.setItem(STORAGE_KEY, activeLeg);
    } catch {
      /* ignore */
    }
  }, [activeLeg]);

  const value = useMemo(() => ({ activeLeg, setActiveLeg }), [activeLeg]);
  return <ActiveLegContext.Provider value={value}>{children}</ActiveLegContext.Provider>;
}

export function useActiveLeg() {
  const ctx = useContext(ActiveLegContext);
  if (!ctx) throw new Error("useActiveLeg must be used inside <ActiveLegProvider>");
  return ctx;
}
