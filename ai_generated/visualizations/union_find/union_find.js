(function () {
  "use strict";
  const model = window.UnionFindModel, byId = id => document.getElementById(id);
  let weighted = location.hash === "#potential", history = [], position = 0, timer = null;
  const current = () => history[position];
  function stop() {
    if (timer) clearInterval(timer);
    timer = null; byId("play").textContent = "再生";
  }
  function reset(n) {
    stop();
    history = [{ state: model.initial(n, weighted), title: "各頂点が1つずつの成分です", description: "操作を試すか、解説シナリオを読み込んでください。代表元は緑の頂点です。", nodes: [], edges: [], tone: "normal" }];
    position = 0; byId("vertex-count").value = n;
    for (const id of ["vertex-u", "vertex-v"]) {
      byId(id).replaceChildren(...Array.from({ length: n }, (_, v) => {
        const option = document.createElement("option"); option.value = v; option.textContent = v; return option;
      }));
    }
    byId("vertex-v").value = 1; byId("difference").value = 5; byId("input-error").textContent = "";
    render();
  }
  function setMode(value) {
    weighted = value;
    byId("ordinary-tab").setAttribute("aria-pressed", !weighted);
    byId("potential-tab").setAttribute("aria-pressed", weighted);
    for (const id of ["weight-field", "diff", "potential-notes", "potential-heading"]) byId(id).hidden = !weighted;
    byId("ordinary-notes").hidden = weighted;
    byId("merge").textContent = weighted ? "merge(u, v, w)" : "merge(u, v)";
    byId("weight-heading").textContent = weighted ? "親への差分" : "根のサイズ";
    byId("edge-legend").textContent = weighted ? "辺ラベル：p[子] − p[親]" : "矢印：子 → 親";
    byId("page-title").textContent = weighted ? "差分を保って、つなぐ。" : "つながりを、ひとつずつ。";
    byId("intro").textContent = weighted ? "p[v] − p[u] = w を満たすように成分を併合します。根間の差分、経路圧縮、矛盾する制約の拒否を確認します。" : "併合と経路圧縮で、親ポインタがどう変わるかを観察します。図の矢印は子から親へ向き、根は緑で表示します。";
    const name = weighted ? "potentialized_disjoint_set_union" : "disjoint_set_union";
    byId("docs-link").href = `../../docs/union_find/${name}.md`;
    byId("source-link").href = `../../union_find/${name}.hpp`;
    byId("scenario-hint").textContent = weighted ? "接続方向の反転、既知の差分、整合する制約と矛盾する制約を比較します。" : "サイズによる併合を重ね、最後に頂点7の経路を圧縮します。";
    window.history.replaceState(null, "", weighted ? "#potential" : "#ordinary");
    reset(weighted ? 6 : 8);
  }
  function addOperations(operations) {
    stop(); history = history.slice(0, position + 1);
    let state = current().state;
    for (const operation of operations) {
      const next = model.trace(state, operation); history.push(...next.frames); state = next.state;
    }
    if (position < history.length - 1) ++position;
    render();
  }
  function input(kind) {
    const u = Number(byId("vertex-u").value), v = Number(byId("vertex-v").value);
    const field = byId("difference"), w = Number(field.value);
    if (weighted && kind === "merge" && (field.value.trim() === "" || !Number.isSafeInteger(w) || Math.abs(w) > 10000)) {
      byId("input-error").textContent = "w は −10000〜10000 の整数で指定してください。"; return;
    }
    byId("input-error").textContent = ""; addOperations([{ kind, u, v, w }]);
  }
  function positions(state) {
    const n = state.parent.length, children = Array.from({ length: n }, () => []), roots = [];
    state.parent.forEach((p, v) => p === -1 ? roots.push(v) : children[p].push(v));
    const result = Array(n), depth = Array(n).fill(0);
    let cursor = 0, maxDepth = 0;
    function visit(v, level) {
      depth[v] = level; maxDepth = Math.max(maxDepth, level);
      if (!children[v].length) result[v] = cursor++;
      else {
        for (const child of children[v]) visit(child, level + 1);
        result[v] = (result[children[v][0]] + result[children[v][children[v].length-1]]) / 2;
      }
    }
    roots.forEach((root, i) => { if (i) cursor += .4; visit(root, 0); });
    const height = Math.max(330, 140 + maxDepth * 100), extent = Math.max(1, cursor - 1);
    return { points: result.map((x, v) => ({ x: cursor <= 1 ? 500 : 55 + x * 890 / extent, y: 75 + depth[v]*100 })), height };
  }
  function draw(frame) {
    const svg = byId("forest"), state = frame.state, { points, height } = positions(state);
    svg.setAttribute("viewBox", `0 0 1000 ${height}`);
    const parts = ['<defs><marker id="arrow" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse"><path d="M 0 0 L 10 5 L 0 10 z" fill="#9bada5"/></marker><marker id="active-arrow" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse"><path d="M 0 0 L 10 5 L 0 10 z" fill="#bd6935"/></marker></defs>'];
    state.parent.forEach((p, v) => {
      if (p === -1) return;
      const from = points[v], to = points[p], dx = to.x-from.x, dy = to.y-from.y, length = Math.hypot(dx,dy);
      const x1 = from.x + dx*27/length, y1 = from.y + dy*27/length, x2 = to.x - dx*32/length, y2 = to.y - dy*32/length;
      const active = frame.edges.includes(v);
      parts.push(`<path class="edge${active ? " active" : ""}" d="M${x1},${y1} L${x2},${y2}" marker-end="url(#${active ? "active-arrow" : "arrow"})"/>`);
      if (weighted) parts.push(`<text class="edge-label" x="${(x1+x2)/2 + 9}" y="${(y1+y2)/2}">${state.weight[v]}</text>`);
    });
    points.forEach((point, v) => {
      const root = state.parent[v] === -1;
      parts.push(`<g class="node${root ? " root" : ""}${frame.nodes.includes(v) ? " active" : ""}" data-vertex="${v}" transform="translate(${point.x},${point.y})" tabindex="0" role="button" aria-label="頂点 ${v} を u に設定"><circle r="26"/><text y="7">${v}</text>${root ? `<text class="root-label" y="-38">size ${state.size[v]}</text>` : ""}</g>`);
    });
    svg.innerHTML = parts.join("");
    svg.querySelectorAll("[data-vertex]").forEach(node => {
      const choose = () => { byId("vertex-u").value = node.dataset.vertex; render(); };
      node.addEventListener("click", choose);
      node.addEventListener("keydown", event => { if (event.key === "Enter" || event.key === " ") { event.preventDefault(); choose(); } });
    });
  }
  function render() {
    const frame = current(), state = frame.state, u = Number(byId("vertex-u").value), info = model.inspect(state,u);
    byId("component-count").textContent = state.count; byId("component-size").textContent = info.size;
    byId("representative").textContent = info.root; byId("step-title").textContent = frame.title;
    byId("step-description").textContent = frame.description; byId("explanation").dataset.tone = frame.tone;
    byId("progress").textContent = `ステップ ${position} / ${history.length-1}`;
    byId("previous").disabled = position === 0;
    for (const id of ["next", "finish", "play"]) byId(id).disabled = position === history.length-1;
    draw(frame);
    byId("values").innerHTML = state.parent.map((p, v) => {
      const entry = model.inspect(state,v);
      return `<tr${frame.nodes.includes(v) ? ' class="active"' : ""}><td>${v}</td><td>${p === -1 ? "根" : p}</td><td>${entry.root}</td><td>${weighted ? state.weight[v] : p === -1 ? state.size[v] : "—"}</td>${weighted ? `<td>${entry.potential}</td>` : ""}</tr>`;
    }).join("");
  }
  byId("ordinary-tab").addEventListener("click", () => setMode(false));
  byId("potential-tab").addEventListener("click", () => setMode(true));
  byId("reset").addEventListener("click", () => {
    const n = Number(byId("vertex-count").value);
    if (!Number.isInteger(n) || n < 2 || n > 12) { byId("input-error").textContent = "頂点数は2〜12の整数で指定してください。"; return; }
    reset(n);
  });
  byId("merge").addEventListener("click", () => input("merge"));
  byId("find").addEventListener("click", () => input("leader"));
  byId("diff").addEventListener("click", () => input("diff"));
  byId("vertex-u").addEventListener("change", render);
  byId("previous").addEventListener("click", () => { stop(); if (position) --position; render(); });
  byId("next").addEventListener("click", () => { stop(); if (position < history.length-1) ++position; render(); });
  byId("finish").addEventListener("click", () => { stop(); position = history.length-1; render(); });
  byId("play").addEventListener("click", () => {
    if (timer) { stop(); return; }
    byId("play").textContent = "一時停止";
    timer = setInterval(() => { if (position < history.length-1) ++position; if (position === history.length-1) stop(); render(); }, 800);
  });
  byId("scenario").addEventListener("click", () => {
    reset(weighted ? 6 : 8);
    const operations = weighted ? [
      {kind:"merge",u:0,v:1,w:5}, {kind:"merge",u:2,v:3,w:-2}, {kind:"merge",u:2,v:4,w:3},
      {kind:"merge",u:0,v:2,w:7}, {kind:"diff",u:1,v:4}, {kind:"merge",u:1,v:4,w:5},
      {kind:"merge",u:1,v:4,w:99}, {kind:"diff",u:0,v:5}
    ] : [[0,1],[2,3],[4,5],[6,7],[0,2],[4,6],[0,4]].map(([u,v]) => ({kind:"merge",u,v,w:0})).concat({kind:"leader",u:7});
    addOperations(operations);
  });
  window.addEventListener("hashchange", () => setMode(location.hash === "#potential"));
  setMode(weighted);
})();
