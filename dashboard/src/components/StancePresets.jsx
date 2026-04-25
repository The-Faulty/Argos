// Save / recall the three bench stances (stand, crouch, extend).
//
// "Save" snapshots the CURRENT /joint_states into ~/.argos/stances.json.
// "Play" tells the bridge to smoothly interpolate to the saved pose. Each
// button shows a timestamp tooltip so the operator knows how stale the
// saved pose is.

const STANCE_NAMES = ["stand", "crouch", "extend"];

export default function StancePresets({ stances, onSave, onPlay, enabled }) {
  return (
    <section className="stance-presets">
      <header><h3>Stance presets</h3></header>
      <div className="stance-presets__grid">
        {STANCE_NAMES.map((name) => {
          const saved = stances?.[name];
          return (
            <div key={name} className="stance-presets__row">
              <button type="button" className="stance-play"
                      onClick={() => onPlay?.(name)} disabled={!enabled || !saved}>
                {name.charAt(0).toUpperCase() + name.slice(1)}
              </button>
              <button type="button" className="stance-save"
                      onClick={() => onSave?.(name)} disabled={!enabled}
                      title={saved ? `saved ${saved.saved_at}` : "not saved yet"}>
                Save current as {name}
              </button>
            </div>
          );
        })}
      </div>
    </section>
  );
}
