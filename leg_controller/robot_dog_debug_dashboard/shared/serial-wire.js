export function extractJsonMessageCandidate(line) {
  const trimmed = String(line ?? "").trim();
  if (!trimmed) {
    return null;
  }

  const firstBrace = trimmed.indexOf("{");
  if (firstBrace < 0) {
    return null;
  }

  const lastBrace = trimmed.lastIndexOf("}");
  const candidate = lastBrace >= firstBrace
    ? trimmed.slice(firstBrace, lastBrace + 1)
    : trimmed.slice(firstBrace);

  // Only treat frames as protocol messages if they contain the expected top-level key.
  if (!candidate.includes("\"type\"")) {
    return null;
  }

  return candidate;
}
