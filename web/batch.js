// batch.js: buttons -> Web Worker -> POST /save into results/ (falls back to downloads without the server).
const IDS = ['V', 'A', 'CAL', 'B0', 'B', 'C', 'D', 'S'];
const logEl = document.getElementById('log');
const say = (s) => { logEl.textContent += s + '\n'; };

async function save(folder, name, text) {
  try {
    const r = await fetch(`/save?path=results/${folder}/${name}`, { method: 'POST', body: text });
    if (!r.ok) throw new Error(r.status);
    return `saved results/${folder}/${name}`;
  } catch {
    const a = Object.assign(document.createElement('a'), { href: URL.createObjectURL(new Blob([text])), download: `${folder}__${name}` });
    a.click();
    return `downloaded ${folder}__${name}`;
  }
}

// Exposed for automation: await window.runAndSave('V')
window.runAndSave = (id) => new Promise((resolve, reject) => {
  const w = new Worker(new URL('../sim/batch_worker.js', import.meta.url), { type: 'module' });
  const t0 = performance.now();
  say(`[${id}] started`);
  w.onmessage = async (e) => {
    const m = e.data;
    if (m.type === 'progress') { if (m.done === m.total || m.done % 500 === 1) say(`[${id}] ${m.done}/${m.total}`); return; }
    if (m.type === 'error') { say(`[${id}] ERROR ${m.message}`); w.terminate(); reject(new Error(m.message)); return; }
    const saved = [];
    for (const out of m.outputs) for (const [name, text] of Object.entries(out.files)) saved.push(await save(out.folder, name, text));
    const secs = ((performance.now() - t0) / 1000).toFixed(1);
    say(`[${id}] done in ${secs} s\n  ` + saved.join('\n  '));
    w.terminate();
    resolve({ id, secs, saved });
  };
  w.postMessage({ id });
});

const box = document.getElementById('buttons');
for (const id of IDS) {
  const b = document.createElement('button');
  b.textContent = id === 'V' ? 'Run V (verification)' : `Run ${id}`;
  b.onclick = () => window.runAndSave(id).catch(() => {});
  box.appendChild(b);
}
