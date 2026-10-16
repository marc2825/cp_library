// 描画から独立した教材用モデル。ブラウザと Node.js の両方で検証する。
(function (root) {
  "use strict";
  const copy = state => ({ ...state, parent: [...state.parent], size: [...state.size], weight: [...state.weight] });
  function initial(n, weighted) {
    return { parent: Array(n).fill(-1), size: Array(n).fill(1), weight: Array(n).fill(0), count: n, weighted };
  }
  function inspect(state, v) {
    let potential = 0;
    while (state.parent[v] !== -1) { potential += state.weight[v]; v = state.parent[v]; }
    return { root: v, potential, size: state.size[v] };
  }
  function trace(base, operation) {
    const state = copy(base), frames = [];
    const emit = (title, description, nodes = [], edges = [], tone = "normal") => {
      frames.push({ state: copy(state), title, description, nodes, edges, tone });
    };
    function find(v) {
      const path = [v];
      let current = v;
      while (state.parent[current] !== -1) {
        const next = state.parent[current];
        emit(`頂点 ${current} の親をたどる`, `${current} → ${next}。代表元に到達するまで親ポインタをたどります。`, [current, next], [current]);
        current = next; path.push(current);
      }
      emit(`代表元は ${current}`, `頂点 ${v} は代表元 ${current} の成分に属します。`, path);
      for (let i = path.length - 2; i >= 0; --i) {
        const node = path[i], old = state.parent[node];
        if (old === current) continue;
        const before = state.weight[node];
        state.weight[node] += state.weight[old]; state.parent[node] = current;
        emit(`頂点 ${node} の経路を圧縮`, state.weighted
          ? `親を ${old} から ${current} へ変更。差分も ${before} + ${state.weight[old]} = ${state.weight[node]} に更新します。`
          : `親を ${old} から代表元 ${current} へ直接つなぎ直します。成分サイズと成分数は変わりません。`, [node, old, current], [node]);
      }
      return current;
    }
    const { u, v, kind } = operation;
    let result;
    if (kind === "leader") {
      result = find(u);
      emit(`leader(${u}) = ${result}`, `この成分のサイズは ${state.size[result]}、成分数は ${state.count} です。`, [u, result]);
    } else {
      let a = find(u), b = find(v);
      if (kind === "diff") {
        result = a === b ? state.weight[v] - state.weight[u] : null;
        emit(result === null ? "差分はまだ分かりません" : `diff(${u}, ${v}) = ${result}`,
          result === null ? "別の成分に属するため、2頂点の相対的な位置は決まっていません。C++ 版では nullopt を返します。"
            : `p[${v}] − p[${u}] = ${state.weight[v]} − (${state.weight[u]}) = ${result}。`, [u, v]);
      } else if (a === b) {
        const known = state.weight[v] - state.weight[u];
        const valid = !state.weighted || known === operation.w;
        result = valid ? "AlreadyConsistent" : "Contradiction";
        emit(valid ? "既に同じ成分です" : "矛盾する制約を拒否します",
          !state.weighted ? "新しい併合は起きません。merge は false、成分数は変わりません。"
            : valid ? `p[${v}] − p[${u}] = ${known} と指定した w = ${operation.w} は一致します。AlreadyConsistent を返します。`
              : `p[${v}] − p[${u}] = ${known} と w = ${operation.w} は異なります。Contradiction を返し、以前の差分関係を保持します。`, [u, v], [], valid ? "normal" : "danger");
      } else {
        let shift = state.weighted ? state.weight[u] + operation.w - state.weight[v] : 0;
        emit("2つの成分を比較します", state.weighted
          ? `根 a = ${a}, b = ${b}。U = ${state.weight[u]}, V = ${state.weight[v]}。\np[b] − p[a] = U + w − V = ${state.weight[u]} + (${operation.w}) − (${state.weight[v]}) = ${shift}。`
          : `代表元 ${a} のサイズは ${state.size[a]}、代表元 ${b} のサイズは ${state.size[b]} です。`, [a, b]);
        if (state.size[a] < state.size[b]) {
          [a, b] = [b, a]; shift = shift === 0 ? 0 : -shift;
          emit("小さい成分を下につなぐ向きに反転", state.weighted
            ? `根 ${b} を根 ${a} の下につなぎます。向きが逆になるので、根間の差分も符号を反転して ${shift} にします。`
            : `根 ${b} を、より大きい成分の根 ${a} の下につなぎます。`, [a, b]);
        }
        state.parent[b] = a; state.weight[b] = shift; state.size[a] += state.size[b]; --state.count;
        result = "Merged";
        emit("併合が完了しました", `根 ${b} の親は ${a} になりました。成分サイズは ${state.size[a]}、成分数は ${state.count} です。`, [a, b], [b]);
      }
    }
    return { frames, state: copy(state), result };
  }
  const api = { initial, inspect, trace };
  if (typeof module !== "undefined" && module.exports) module.exports = api;
  else root.UnionFindModel = api;
})(typeof globalThis !== "undefined" ? globalThis : this);
