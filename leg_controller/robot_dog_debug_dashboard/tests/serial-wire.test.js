import test from "node:test";
import assert from "node:assert/strict";
import { extractJsonMessageCandidate } from "../shared/serial-wire.js";

test("extractJsonMessageCandidate ignores non-json serial noise", () => {
  assert.equal(extractJsonMessageCandidate("Starting up"), null);
  assert.equal(extractJsonMessageCandidate("\u0008\u00ff\u00a5"), null);
});

test("extractJsonMessageCandidate keeps clean JSON frames", () => {
  assert.equal(
    extractJsonMessageCandidate('{"type":"state","payload":{"ok":true}}'),
    '{"type":"state","payload":{"ok":true}}',
  );
});

test("extractJsonMessageCandidate trims boot garbage before JSON", () => {
  assert.equal(
    extractJsonMessageCandidate('ets Jul 29 2019 {"type":"hello_ack","payload":{"mode":"idle"}}'),
    '{"type":"hello_ack","payload":{"mode":"idle"}}',
  );
});

test("extractJsonMessageCandidate ignores brace noise without a type field", () => {
  assert.equal(extractJsonMessageCandidate("{\u00ff\u0000garbage"), null);
  assert.equal(extractJsonMessageCandidate("boot {ready}"), null);
});
