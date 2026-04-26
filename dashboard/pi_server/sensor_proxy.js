// Sensor proxy — bridges the Python sidecar (pi_sensors/sensor_server.py) to
// the dashboard's same-origin WS + HTTP API. The sidecar exposes:
//   ws://localhost:8788/sensors  (IMU @100 Hz + thermal @8 Hz, JSON frames)
//   http://localhost:8788/camera/stream  (multipart MJPEG)
//
// We hold a long-lived WS to the sidecar and re-emit each frame as an event
// for the server.js broadcaster. The MJPEG endpoint is proxied via a thin
// pipe in `proxyCameraStream`.

import { EventEmitter } from "node:events";
import http from "node:http";
import { WebSocket } from "ws";

const SIDECAR_HOST = process.env.ARGOS_SIDECAR_HOST || "127.0.0.1";
const SIDECAR_PORT = Number(process.env.ARGOS_SIDECAR_PORT || 8788);

export class SensorProxy extends EventEmitter {
  constructor({ host = SIDECAR_HOST, port = SIDECAR_PORT } = {}) {
    super();
    this.host = host;
    this.port = port;
    this.ws = null;
    this.connected = false;
    this.lastImu = null;
    this.lastThermal = null;
    this._reconnectTimer = null;
  }

  connect() {
    if (this.ws) return;
    const url = `ws://${this.host}:${this.port}/sensors`;
    let ws;
    try {
      ws = new WebSocket(url);
    } catch (err) {
      this.emit("error", err);
      this._scheduleReconnect();
      return;
    }
    this.ws = ws;
    ws.on("open", () => {
      this.connected = true;
      this.emit("open");
    });
    ws.on("close", () => {
      this.connected = false;
      this.ws = null;
      this.emit("close");
      this._scheduleReconnect();
    });
    ws.on("error", (err) => this.emit("error", err));
    ws.on("message", (raw) => this._onMessage(raw));
  }

  disconnect() {
    if (this._reconnectTimer) {
      clearTimeout(this._reconnectTimer);
      this._reconnectTimer = null;
    }
    if (this.ws) {
      try {
        this.ws.close();
      } catch {
        /* already closed */
      }
      this.ws = null;
    }
  }

  _scheduleReconnect() {
    if (this._reconnectTimer) return;
    this._reconnectTimer = setTimeout(() => {
      this._reconnectTimer = null;
      this.connect();
    }, 2000);
  }

  _onMessage(raw) {
    let msg;
    try {
      msg = JSON.parse(raw.toString());
    } catch {
      return;
    }
    if (msg.type === "imu") {
      this.lastImu = msg;
      this.emit("imu", msg);
    } else if (msg.type === "thermal") {
      this.lastThermal = msg;
      this.emit("thermal", msg);
    } else {
      this.emit("message", msg);
    }
  }

  // Express handler for the camera proxy. Pipes the sidecar's multipart MJPEG
  // straight through to the browser. Fails open with 502 if the sidecar is
  // unreachable so the dashboard can show a placeholder instead of hanging.
  proxyCameraStream(req, res) {
    const upstream = http.request(
      {
        host: this.host,
        port: this.port,
        path: "/camera/stream",
        method: "GET",
      },
      (upstreamRes) => {
        res.writeHead(upstreamRes.statusCode || 502, upstreamRes.headers);
        upstreamRes.pipe(res);
      },
    );
    upstream.on("error", (err) => {
      if (!res.headersSent) {
        res.writeHead(502, { "content-type": "text/plain" });
        res.end(`sensor sidecar unavailable: ${err.message}`);
      } else {
        res.end();
      }
    });
    req.on("close", () => upstream.destroy());
    upstream.end();
  }
}
