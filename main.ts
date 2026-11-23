// Sokoban solver — account 4 (lathixbt).
// Angle: per-room push-optimal A* with Manhattan-assignment heuristic and
// provably-safe deadlock pruning (dead squares, wall freeze, 2x2 freeze).
// Rooms are solved independently (boxes never leave their room), then the
// room visiting order is chosen to minimise walking. Deterministic: fixed
// iteration orders, no randomness, no clock, no wall-clock budgets — only
// expansion caps.
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const DIRS = [
  { dx: 0, dy: -1, ch: 'u' },
  { dx: 0, dy: 1, ch: 'd' },
  { dx: -1, dy: 0, ch: 'l' },
  { dx: 1, dy: 0, ch: 'r' },
];

// ---------------------------------------------------------------------------
// Small deterministic binary heap (min-heap on a comparator).
// ---------------------------------------------------------------------------
class Heap {
  private a: any[] = [];
  private cmp: (x: any, y: any) => number;
  constructor(cmp: (x: any, y: any) => number) {
    this.cmp = cmp;
  }
  get size(): number {
    return this.a.length;
  }
  push(v: any): void {
    const a = this.a;
    a.push(v);
    let i = a.length - 1;
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (this.cmp(a[i], a[p]) < 0) {
        const t = a[i];
        a[i] = a[p];
        a[p] = t;
        i = p;
      } else break;
    }
  }
  pop(): any {
    const a = this.a;
    const top = a[0];
    const last = a.pop();
    if (a.length > 0) {
      a[0] = last;
      let i = 0;
      for (;;) {
        const l = i * 2 + 1;
        const r = l + 1;
        let m = i;
        if (l < a.length && this.cmp(a[l], a[m]) < 0) m = l;
        if (r < a.length && this.cmp(a[r], a[m]) < 0) m = r;
        if (m === i) break;
        const t = a[i];
        a[i] = a[m];
        a[m] = t;
        i = m;
      }
    }
    return top;
  }
}

// ---------------------------------------------------------------------------
// Level parsing. The grid is padded with a 1-cell wall border so neighbour
// lookups never need bounds checks.
// ---------------------------------------------------------------------------
function parseLevel(text: string): any {
  const raw = text.replace(/\r/g, '').split('\n');
  while (raw.length > 0 && raw[raw.length - 1] === '') raw.pop();
  const h0 = raw.length;
  let w0 = 0;
  for (const l of raw) if (l.length > w0) w0 = l.length;
  const W = w0 + 2;
  const H = h0 + 2;
  const wall = new Array<boolean>(W * H).fill(true);
  const goal = new Array<boolean>(W * H).fill(false);
  const boxes: number[] = [];
  let player = -1;
  for (let y = 0; y < h0; y++) {
    const line = raw[y];
    for (let x = 0; x < line.length; x++) {
      const c = line[x];
      const idx = (y + 1) * W + (x + 1);
      if (c === '#') continue;
      wall[idx] = false;
      if (c === '.' || c === '*' || c === '+') goal[idx] = true;
      if (c === '$' || c === '*') boxes.push(idx);
      if (c === '@' || c === '+') player = idx;
    }
  }
  boxes.sort((a, b) => a - b);
  return { W, H, wall, goal, boxes, player };
}

// BFS over walkable cells avoiding blocked cells. Returns seen flags, parent
// links and the step character used to reach each cell (fixed neighbour
// order u,d,l,r, so paths are deterministic).
function bfs(walk: boolean[], blocked: boolean[] | null, start: number, W: number, H: number): any {
  const seen = new Array<boolean>(W * H).fill(false);
  const parent = new Int32Array(W * H).fill(-1);
  const pch = new Array<string>(W * H).fill('');
  const q: number[] = [start];
  seen[start] = true;
  parent[start] = -2;
  let head = 0;
  while (head < q.length) {
    const cur = q[head++];
    for (const d of DIRS) {
      const n = cur + d.dy * W + d.dx;
      if (!walk[n] || seen[n]) continue;
      if (blocked && blocked[n]) continue;
      seen[n] = true;
      parent[n] = cur;
      pch[n] = d.ch;
      q.push(n);
    }
  }
  return { seen, parent, pch };
}

function pathTo(b: any, target: number): string {
  const chars: string[] = [];
  let cur = target;
  while (b.parent[cur] >= 0) {
    chars.push(b.pch[cur]);
    cur = b.parent[cur];
  }
  chars.reverse();
  return chars.join('');
}

function regionMin(b: any, W: number, H: number): number {
  for (let i = 0; i < W * H; i++) if (b.seen[i]) return i;
  return -1;
}

// ---------------------------------------------------------------------------
// Dead squares: reverse-pull BFS from the goals over box positions. A box
// position is live when a goal can be reached from it by pushes (ignoring
// other boxes, so this is an over-approximation: a cell NOT live is truly
// dead and a box on it can never reach a goal).
// ---------------------------------------------------------------------------
function computeLive(walk: boolean[], target: boolean[], goals: number[], W: number, H: number): boolean[] {
  const live = new Array<boolean>(W * H).fill(false);
  const q: number[] = [];
  for (const g of goals) {
    live[g] = true;
    q.push(g);
  }
  let head = 0;
  while (head < q.length) {
    const cur = q[head++];
    for (const d of DIRS) {
      const p = cur - (d.dy * W + d.dx);
      if (!target[p] || live[p]) continue;
      const behind = p - (d.dy * W + d.dx);
      if (!walk[behind]) continue;
      live[p] = true;
      q.push(p);
    }
  }
  return live;
}

// Provably-safe deadlock test for a box configuration.
// - box on a dead square (not a goal) -> dead
// - box against a wall that cannot slide along it (both slide directions
//   blocked by walls alone) and is not on a goal -> dead
// - solid 2x2 block of boxes, none on a goal -> dead (no box in it can move)
function boxesDead(boxes: number[], live: boolean[], goalSet: Set<number>, walk: boolean[], target: boolean[], W: number): boolean {
  const boxSet = new Set(boxes);
  for (const b of boxes) {
    if (goalSet.has(b)) continue;
    if (target[b] && !live[b]) return true;
    // Freeze against a wall: a box against a wall can only slide along it.
    // Sliding east needs a walkable stance west and a target cell east;
    // sliding west needs a walkable stance east and a target cell west.
    // Both impossible by walls alone -> the box can never move (pushes
    // into the wall need a stance inside it; pushes off the wall need a
    // stance inside it; pushes into the hallway only strand it).
    const up = walk[b - W];
    const down = walk[b + W];
    const left = walk[b - 1];
    const right = walk[b + 1];
    // For a wall on one side, the two slide directions are along the
    // perpendicular axis; each entry below is [stanceSide, destSide] for
    // one slide direction. A slide is possible only when its stance cell
    // is walkable and its destination cell is a push target.
    const walls: { stanceA: boolean; destA: boolean; stanceB: boolean; destB: boolean }[] = [];
    if (!up) walls.push({ stanceA: left, destA: right && target[b + 1], stanceB: right, destB: left && target[b - 1] });
    if (!down) walls.push({ stanceA: left, destA: right && target[b + 1], stanceB: right, destB: left && target[b - 1] });
    if (!left) walls.push({ stanceA: up, destA: down && target[b + W], stanceB: down, destB: up && target[b - W] });
    if (!right) walls.push({ stanceA: up, destA: down && target[b + W], stanceB: down, destB: up && target[b - W] });
    for (const w of walls) {
      const canA = w.stanceA && w.destA;
      const canB = w.stanceB && w.destB;
      if (!canA && !canB) return true;
    }
  }
  // 2x2 freeze: every push of every box in a solid 2x2 block is blocked by
  // another box of the same block (as stance or destination), so no box in
  // it can ever move first and the block is permanent.
  for (const b of boxes) {
    const tls = [b, b - 1, b - W, b - W - 1];
    for (const tl of tls) {
      const cells = [tl, tl + 1, tl + W, tl + W + 1];
      if (cells.every((c) => boxSet.has(c)) && !cells.some((c) => goalSet.has(c))) return true;
    }
  }
  return false;
}

// ---------------------------------------------------------------------------
// Admissible heuristic: minimum-cost assignment of boxes to goals by
// Manhattan distance (brute-force permutations, <= 6 boxes). For larger
// sets, sum of per-box nearest-goal Manhattan distances (also admissible).
// ---------------------------------------------------------------------------
function heuristic(boxes: number[], goals: number[], W: number, cache: Map<string, number>): number {
  const key = boxes.join(',');
  const hit = cache.get(key);
  if (hit !== undefined) return hit;
  const n = boxes.length;
  let h: number;
  if (n <= 6) {
    const bx = boxes.map((b) => b % W);
    const by = boxes.map((b) => (b / W) | 0);
    const gx = goals.map((g) => g % W);
    const gy = goals.map((g) => (g / W) | 0);
    const d: number[][] = [];
    for (let i = 0; i < n; i++) {
      d.push([]);
      for (let j = 0; j < n; j++) d[i].push(Math.abs(bx[i] - gx[j]) + Math.abs(by[i] - gy[j]));
    }
    let best = Infinity;
    const perm = Array.from({ length: n }, (_, i) => i);
    const rec = (k: number): void => {
      if (k === n) {
        let s = 0;
        for (let i = 0; i < n; i++) s += d[i][perm[i]];
        if (s < best) best = s;
        return;
      }
      for (let i = k; i < n; i++) {
        const t = perm[k];
        perm[k] = perm[i];
        perm[i] = t;
        rec(k + 1);
        const t2 = perm[k];
        perm[k] = perm[i];
        perm[i] = t2;
      }
    };
    rec(0);
    h = best;
  } else {
    let s = 0;
    for (const b of boxes) {
      let m = Infinity;
      for (const g of goals) {
        const dd = Math.abs((b % W) - (g % W)) + Math.abs(((b / W) | 0) - ((g / W) | 0));
        if (dd < m) m = dd;
      }
      s += m;
    }
    h = s;
  }
  cache.set(key, h);
  return h;
}

// ---------------------------------------------------------------------------
// Push-optimal A* over (sorted boxes, canonical player region). One step is
// one push; the walk to each push stance is filled in by BFS. Cost is pushes
// first, then total moves, then a deterministic key order, so the result is
// push-optimal and byte-deterministic. weight > 1 gives weighted A* (a fast
// non-optimal fallback); greedy uses f = h only.
// ---------------------------------------------------------------------------
function pushSearch(cfg: any): any {
  const { walk, target, goals, W, H } = cfg;
  const goalSet = new Set<number>(goals);
  const live = computeLive(walk, target, goals, W, H);
  const hCache = new Map<string, number>();
  let seq = 0;
  const cmp = (a: any, b: any): number => {
    if (a.f !== b.f) return a.f - b.f;
    if (a.pushes !== b.pushes) return a.pushes - b.pushes;
    if (a.moves !== b.moves) return a.moves - b.moves;
    if (a.key !== b.key) return a.key < b.key ? -1 : 1;
    return a.seq - b.seq;
  };
  const heap = new Heap(cmp);
  const visited = new Map<string, { pushes: number; moves: number }>();

  const occFor = (boxes: number[]): boolean[] => {
    const occ = new Array<boolean>(W * H).fill(false);
    for (const b of boxes) occ[b] = true;
    return occ;
  };

  const makeNode = (boxes: number[], player: number, pushes: number, moves: number, parent: any, seg: string): any => {
    const occ = occFor(boxes);
    const rb = bfs(walk, occ, player, W, H);
    const rmin = regionMin(rb, W, H);
    const key = boxes.join(',') + '|' + rmin;
    const h = heuristic(boxes, goals, W, hCache);
    const f = cfg.greedy ? h : pushes + cfg.weight * h;
    return { key, boxes, player, pushes, moves, parent, seg, f, seq: seq++ };
  };

  const startBoxes = cfg.boxes.slice().sort((a: number, b: number) => a - b);
  if (boxesDead(startBoxes, live, goalSet, walk, target, W)) return null;
  const start = makeNode(startBoxes, cfg.player, 0, 0, null, '');
  visited.set(start.key, { pushes: 0, moves: 0 });
  heap.push(start);

  let expansions = 0;
  while (heap.size > 0) {
    const node = heap.pop();
    const v = visited.get(node.key);
    if (v && (v.pushes < node.pushes || (v.pushes === node.pushes && v.moves < node.moves))) continue;
    // Goal test on pop: with a consistent heuristic this is push-optimal.
    if (node.boxes.every((b: number) => goalSet.has(b))) {
      if (!cfg.requireExit) {
        return { moves: reconstruct(node), pushes: node.pushes, endPlayer: node.player };
      }
      const occ = occFor(node.boxes);
      const eb = bfs(walk, occ, node.player, W, H);
      if (eb.seen[cfg.exitCell]) {
        return { moves: reconstruct(node), pushes: node.pushes, endPlayer: node.player };
      }
    }
    if (++expansions > cfg.maxExpansions) return null;
    const occ = occFor(node.boxes);
    const b = bfs(walk, occ, node.player, W, H);
    const boxSet = new Set<number>(node.boxes);
    for (const box of node.boxes) {
      for (const d of DIRS) {
        const dest = box + d.dy * W + d.dx;
        const stance = box - (d.dy * W + d.dx);
        if (!target[dest] || boxSet.has(dest)) continue;
        if (!walk[stance] || boxSet.has(stance) || !b.seen[stance]) continue;
        const nb = node.boxes.map((x: number) => (x === box ? dest : x)).sort((x: number, y: number) => x - y);
        if (boxesDead(nb, live, goalSet, walk, target, W)) continue;
        const seg = pathTo(b, stance) + d.ch;
        const child = makeNode(nb, box, node.pushes + 1, node.moves + seg.length, node, seg);
        const cv = visited.get(child.key);
        if (cv && (cv.pushes < child.pushes || (cv.pushes === child.pushes && cv.moves <= child.moves))) continue;
        visited.set(child.key, { pushes: child.pushes, moves: child.moves });
        heap.push(child);
      }
    }
  }
  return null;
}

function reconstruct(node: any): string {
  const parts: string[] = [];
  let cur = node;
  while (cur.parent) {
    parts.push(cur.seg);
    cur = cur.parent;
  }
  parts.reverse();
  return parts.join('');
}

// ---------------------------------------------------------------------------
// Room detection: the hallway is the player's row. Each room is the flood
// fill from a floor cell just above/below a hallway cell, excluding the
// hallway row. Rooms sharing cells are merged (one room, several doors).
// ---------------------------------------------------------------------------
function detectRooms(L: any): any {
  const hy = (L.player / L.W) | 0;
  const hallway: number[] = [];
  for (let x = 0; x < L.W; x++) {
    const c = hy * L.W + x;
    if (!L.wall[c]) hallway.push(c);
  }
  const hallwaySet = new Set<number>(hallway);
  const found: any[] = [];
  const seen = new Set<number>();
  for (const c of hallway) {
    for (const dy of [-1, 1]) {
      const n = c + dy * L.W;
      if (L.wall[n] || hallwaySet.has(n) || seen.has(n)) continue;
      const cells = new Set<number>();
      const q = [n];
      cells.add(n);
      seen.add(n);
      let head = 0;
      while (head < q.length) {
        const cur = q[head++];
        for (const d of DIRS) {
          const m = cur + d.dy * L.W + d.dx;
          if (L.wall[m] || hallwaySet.has(m) || cells.has(m)) continue;
          cells.add(m);
          seen.add(m);
          q.push(m);
        }
      }
      found.push({ cells, doors: [n], approaches: [c] });
    }
  }
  // Merge rooms that share cells.
  const merged: any[] = [];
  for (const r of found) {
    let targetRoom: any = null;
    for (const m of merged) {
      for (const c of r.cells) {
        if (m.cells.has(c)) {
          targetRoom = m;
          break;
        }
      }
      if (targetRoom) break;
    }
    if (!targetRoom) {
      merged.push({ cells: new Set(r.cells), doors: r.doors.slice(), approaches: r.approaches.slice() });
    } else {
      for (const c of r.cells) targetRoom.cells.add(c);
      for (const d of r.doors) if (!targetRoom.doors.includes(d)) targetRoom.doors.push(d);
      for (const a of r.approaches) if (!targetRoom.approaches.includes(a)) targetRoom.approaches.push(a);
    }
  }
  const rooms = merged.map((m) => {
    const boxes: number[] = [];
    const goals: number[] = [];
    for (const c of m.cells) {
      if (L.boxes.includes(c)) boxes.push(c);
      if (L.goal[c]) goals.push(c);
    }
    boxes.sort((a, b) => a - b);
    goals.sort((a, b) => a - b);
    return { cells: m.cells, boxes, goals, approach: Math.min(...m.approaches) };
  });
  return { rooms, hallway };
}

function verifyRooms(L: any, rooms: any[]): boolean {
  if (rooms.length === 0) return false;
  const boxCovered = new Set<number>();
  const goalCovered = new Set<number>();
  for (const r of rooms) {
    if (r.boxes.length === 0) return false;
    if (r.boxes.length !== r.goals.length) return false;
    for (const b of r.boxes) {
      if (boxCovered.has(b)) return false;
      boxCovered.add(b);
    }
    for (const g of r.goals) {
      if (goalCovered.has(g)) return false;
      goalCovered.add(g);
    }
  }
  if (boxCovered.size !== L.boxes.length) return false;
  let goalCount = 0;
  for (let i = 0; i < L.W * L.H; i++) if (L.goal[i]) goalCount++;
  if (goalCovered.size !== goalCount) return false;
  return true;
}

// Solve one room from its approach (hallway) cell. Returns the push-optimal
// solution (a) and, when (a) leaves the player unable to walk back out to
// the hallway, the push-optimal solution among exitable ones (b).
function solveRoom(L: any, room: any): any {
  const walk = new Array<boolean>(L.W * L.H).fill(false);
  for (const c of room.cells) walk[c] = true;
  for (const c of L.hallwayAll) walk[c] = true;
  const target = new Array<boolean>(L.W * L.H).fill(false);
  for (const c of room.cells) target[c] = true;
  const base: any = {
    walk,
    target,
    goals: room.goals,
    boxes: room.boxes,
    player: room.approach,
    W: L.W,
    H: L.H,
    exitCell: room.approach,
  };
  let a = pushSearch({ ...base, maxExpansions: 400000, weight: 1, greedy: false, requireExit: false });
  if (!a) a = pushSearch({ ...base, maxExpansions: 400000, weight: 2, greedy: false, requireExit: false });
  if (!a) a = pushSearch({ ...base, maxExpansions: 600000, weight: 1, greedy: true, requireExit: false });
  if (!a) return { a: null, b: null };
  // Is (a) exitable? Player must be able to walk back to the approach cell
  // with the room's boxes on their goals.
  const occ = new Array<boolean>(L.W * L.H).fill(false);
  for (const g of room.goals) occ[g] = true;
  const eb = bfs(walk, occ, a.endPlayer, L.W, L.H);
  if (eb.seen[room.approach]) return { a, b: a };
  let b = pushSearch({ ...base, maxExpansions: 400000, weight: 1, greedy: false, requireExit: true });
  if (!b) b = pushSearch({ ...base, maxExpansions: 400000, weight: 2, greedy: false, requireExit: true });
  if (!b) b = pushSearch({ ...base, maxExpansions: 600000, weight: 1, greedy: true, requireExit: true });
  return { a, b };
}

// Whole-level fallback: the same A* on the full level (used only when the
// hallway/room structure does not hold).
function solveWhole(L: any): any {
  const walk = L.wall.map((w: boolean) => !w);
  const target = walk.slice();
  const goals: number[] = [];
  for (let i = 0; i < L.W * L.H; i++) if (L.goal[i]) goals.push(i);
  const base: any = {
    walk,
    target,
    goals,
    boxes: L.boxes,
    player: L.player,
    W: L.W,
    H: L.H,
    exitCell: L.player,
  };
  let r = pushSearch({ ...base, maxExpansions: 100000, weight: 1, greedy: false, requireExit: false });
  if (!r) r = pushSearch({ ...base, maxExpansions: 200000, weight: 2, greedy: false, requireExit: false });
  if (!r) r = pushSearch({ ...base, maxExpansions: 500000, weight: 1, greedy: true, requireExit: false });
  return r;
}

// All room orders (lexicographic), pick the one with fewest pushes, then
// least walking. A room with no exitable solution may only go last.
function assemble(L: any, rooms: any[], solved: any[]): any {
  const n = rooms.length;
  const idx = Array.from({ length: n }, (_, i) => i);
  const perms: number[][] = [];
  const gen = (k: number): void => {
    if (k === n) {
      perms.push(idx.slice());
      return;
    }
    for (let i = k; i < n; i++) {
      const t = idx[k];
      idx[k] = idx[i];
      idx[i] = t;
      gen(k + 1);
      const t2 = idx[k];
      idx[k] = idx[i];
      idx[i] = t2;
    }
  };
  gen(0);
  const walkAll = L.wall.map((w: boolean) => !w);
  let best: any = null;
  for (const perm of perms) {
    let ok = true;
    let pushes = 0;
    let walkMoves = 0;
    let moves = '';
    let curPlayer = L.player;
    const curBoxes = rooms.map((r) => r.boxes.slice());
    for (let pi = 0; pi < perm.length; pi++) {
      const ri = perm[pi];
      const last = pi === perm.length - 1;
      const sol = last ? solved[ri].a : solved[ri].b;
      if (!sol) {
        ok = false;
        break;
      }
      const occ = new Array<boolean>(L.W * L.H).fill(false);
      for (const bs of curBoxes) for (const b of bs) occ[b] = true;
      const bf = bfs(walkAll, occ, curPlayer, L.W, L.H);
      if (!bf.seen[rooms[ri].approach]) {
        ok = false;
        break;
      }
      const leg = pathTo(bf, rooms[ri].approach);
      walkMoves += leg.length;
      moves += leg + sol.moves;
      curPlayer = sol.endPlayer;
      curBoxes[ri] = rooms[ri].goals.slice();
      pushes += sol.pushes;
    }
    if (!ok) continue;
    if (!best || pushes < best.pushes || (pushes === best.pushes && walkMoves < best.walk)) {
      best = { pushes, walk: walkMoves, moves };
    }
  }
  return best;
}

// Replay a move string on the level: returns solved? and push count.
function simulate(L: any, moves: string): any {
  const boxes = new Set<number>(L.boxes);
  let player = L.player;
  let pushes = 0;
  for (const ch of moves) {
    const d = DIRS.find((x) => x.ch === ch);
    if (!d) return { solved: false, pushes: -1 };
    const dest = player + d.dy * L.W + d.dx;
    if (L.wall[dest]) return { solved: false, pushes: -1 };
    if (boxes.has(dest)) {
      const beyond = dest + d.dy * L.W + d.dx;
      if (L.wall[beyond] || boxes.has(beyond)) return { solved: false, pushes: -1 };
      boxes.delete(dest);
      boxes.add(beyond);
      pushes++;
    }
    player = dest;
  }
  let solved = true;
  for (const b of boxes) if (!L.goal[b]) solved = false;
  return { solved, pushes };
}

function solveLevel(text: string): string {
  const L = parseLevel(text);
  if (L.player < 0 || L.boxes.length === 0) return '';
  const { rooms, hallway } = detectRooms(L);
  L.hallwayAll = hallway;
  if (rooms.length > 0 && verifyRooms(L, rooms)) {
    const solved = rooms.map((r: any) => solveRoom(L, r));
    if (solved.every((s: any) => s.a)) {
      const best = assemble(L, rooms, solved);
      if (best) {
        const v = simulate(L, best.moves);
        if (v.solved) return best.moves;
      }
    }
  }
  const whole = solveWhole(L);
  if (whole) {
    const v = simulate(L, whole.moves);
    if (v.solved) return whole.moves;
  }
  return '';
}

const I = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const O = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';
for (const id of readdirSync(I).sort()) {
  let moves = '';
  try {
    moves = solveLevel(readFileSync(join(I, id, 'level.txt'), 'utf8'));
  } catch {
    moves = '';
  }
  mkdirSync(join(O, id), { recursive: true });
  writeFileSync(join(O, id, 'solution.txt'), `${moves}\n`);
}
