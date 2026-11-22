// util.js: seeded random streams, angle helpers, small linear algebra, hashing.

export const clamp = (a, lo, hi) => (a < lo ? lo : a > hi ? hi : a);
export const wrap = (a) => { a = (a + Math.PI) % (2 * Math.PI); if (a < 0) a += 2 * Math.PI; return a - Math.PI; };

// mulberry32 uniform + Box-Muller normal. Each sensor gets its own stream so the same seed gives
// the same noise for every arm (common random numbers, PLAN §4.1).
export function makeRng(seed) {
  let a = seed >>> 0, spare = null;
  const u = () => {
    a = (a + 0x6D2B79F5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
  const n = () => {
    if (spare !== null) { const s = spare; spare = null; return s; }
    const u1 = Math.max(u(), 1e-12), u2 = u(), r = Math.sqrt(-2 * Math.log(u1));
    spare = r * Math.sin(2 * Math.PI * u2);
    return r * Math.cos(2 * Math.PI * u2);
  };
  return { u, n };
}
export const STREAM = Object.freeze({ gyro: 1, camera: 2, channel: 4 });
export const streamSeed = (seed, stream) => (seed * 7919 + stream) >>> 0;

export function inv3(m) {
  const [a, b, c] = m[0], [d, e, f] = m[1], [g, h, i] = m[2];
  const A = e * i - f * h, B = -(d * i - f * g), C = d * h - e * g, det = a * A + b * B + c * C;
  return [[A / det, -(b * i - c * h) / det, (b * f - c * e) / det],
          [B / det, (a * i - c * g) / det, -(a * f - c * d) / det],
          [C / det, -(a * h - b * g) / det, (a * e - b * d) / det]];
}

// FNV-1a over a canonical JSON string (for meta.json parameter hashes)
export function hashObj(o) {
  const s = JSON.stringify(o, Object.keys(o).sort());
  let h = 0x811c9dc5;
  for (let i = 0; i < s.length; i++) { h ^= s.charCodeAt(i); h = Math.imul(h, 0x01000193); }
  return (h >>> 0).toString(16).padStart(8, '0');
}
