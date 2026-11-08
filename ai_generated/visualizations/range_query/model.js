/* Offline teaching model. The diagrams trace recursive interval decomposition. */
(function (root, factory) {
  const api = factory();
  if (typeof module === "object" && module.exports) module.exports = api;
  else root.RangeQueryModel = api;
})(typeof globalThis !== "undefined" ? globalThis : this, function () {
  "use strict";
  const modes = ["fenwick", "segtree", "lazy", "dual", "range-bit"];
  const clone = x => JSON.parse(JSON.stringify(x));
  const id = () => ({a: 1, b: 0});
  function safe(x) {
    if (!Number.isSafeInteger(x)) throw new Error("数値が安全に表示できる整数の範囲を超えました。初期化して値を小さくしてください。");
    return x === 0 ? 0 : x;
  }
  const compose = (f, g) => ({a: safe(f.a * g.a), b: safe(f.a * g.b + f.b)});
  const mapping = (f, x) => safe(f.a * x + f.b);
  function bitBuild(a) {
    const b = [0, ...a];
    for (let i = 1; i < b.length; ++i) {
      const p = i + (i & -i);
      if (p < b.length) b[p] = safe(b[p] + b[i]);
    }
    return b;
  }
  function prefix(b, r) {
    let value = 0;
    for (; r > 0; r -= r & -r) value = safe(value + b[r]);
    return value;
  }
  function create(mode, values) {
    if (!modes.includes(mode)) throw new Error("モードが不正です。");
    if (!Array.isArray(values) || values.length > 16) throw new Error("要素数は 0〜16 個です。");
    values.forEach(safe);
    let base = 1;
    while (base < values.length) base *= 2;
    const s = {mode, n: values.length, base};
    if (mode === "fenwick") s.bit = bitBuild(values);
    else if (mode === "range-bit") {
      const d = values.map((x, i) => safe(x - (i ? values[i - 1] : 0)));
      s.bit = bitBuild(d);
      s.weighted = bitBuild(d.map((x, i) => safe(x * i)));
    } else {
      s.nodes = Array(base * 2).fill(null);
      function build(k, l, r) {
        s.nodes[k] = {l, r, sum: k < base && mode === "dual" ? null : 0, tag: id()};
        if (k >= base) s.nodes[k].sum = l < values.length ? values[l] : 0;
        else {
          build(k * 2, l, (l + r) / 2); build(k * 2 + 1, (l + r) / 2, r);
          if (mode !== "dual") s.nodes[k].sum = safe(s.nodes[k * 2].sum + s.nodes[k * 2 + 1].sum);
        }
      }
      build(1, 0, base);
    }
    return s;
  }
  function values(s) {
    if (s.mode === "fenwick") return Array.from({length: s.n}, (_, i) => safe(prefix(s.bit, i + 1) - prefix(s.bit, i)));
    if (s.mode === "range-bit") return Array.from({length: s.n}, (_, i) => prefix(s.bit, i + 1));
    const a = [];
    function visit(k, inherited) {
      const node = s.nodes[k];
      if (k >= s.base) { if (node.l < s.n) a.push(mapping(inherited, node.sum)); }
      else {
        const next = compose(inherited, node.tag);
        visit(k * 2, next); visit(k * 2 + 1, next);
      }
    }
    visit(1, id());
    return a;
  }
  function run(initial, operation) {
    const s = clone(initial), op = clone(operation), frames = [];
    const emit = (title, detail, active = [], result = null) => frames.push({state: clone(s), title, detail, active, result});
    const integer = x => { if (!Number.isSafeInteger(x)) throw new Error("整数を入力してください。"); return x; };
    const point = p => { integer(p); if (p < 0 || p >= s.n) throw new Error("添字 p は 0 以上、要素数未満です。"); };
    const interval = (l, r) => {
      integer(l); integer(r);
      if (l < 0 || l > r || r > s.n) throw new Error("0 ≤ l ≤ r ≤ n を満たしてください。");
    };
    const allowed = {fenwick: ["add", "set", "sum", "lower"], segtree: ["set", "sum"], lazy: ["apply", "set", "sum", "get"], dual: ["apply", "set", "get"], "range-bit": ["add", "sum"]};
    if (!allowed[s.mode].includes(op.type)) throw new Error("このモードでは使えない操作です。");
    if (["set", "get"].includes(op.type) || (op.type === "add" && s.mode === "fenwick")) point(op.p);
    else if (op.type !== "lower") interval(op.l, op.r);
    if (["add", "set"].includes(op.type)) integer(op.x);
    if (op.type === "apply") { integer(op.a); integer(op.b); }
    if (op.type === "lower") {
      integer(op.w);
      if (values(s).some(x => x < 0)) throw new Error("lower_bound は全要素が非負の場合に使えます。");
    }
    emit("操作を開始", "オレンジ色は、このステップで参照・変更するノードです。戻ると過去の状態を確認できます。");
    frames[0].stable = true;
    function addBit(which, p, x) {
      const bit = which === "B2" ? s.weighted : s.bit;
      for (let k = p + 1; k <= s.n; k += k & -k) {
        bit[k] = safe(bit[k] + x);
        emit(`${which}[${k}] += ${x}`, `区間 [${k - (k & -k)}, ${k}) の和を変更。次は i += lowbit(i) で上位の区間へ進みます。`, [k]);
      }
    }
    function sumBit(which, r) {
      const bit = which === "B2" ? s.weighted : s.bit;
      let value = 0;
      for (let k = r; k > 0; k -= k & -k) {
        value = safe(value + bit[k]);
        emit(`${which} prefix(${r}): 累計 ${value}`, `[${k - (k & -k)}, ${k}) の ${bit[k]} を加えます。次は i -= lowbit(i) で未集計の左側へ進みます。`, [k]);
      }
      return value;
    }
    function applyNode(k, f) {
      const node = s.nodes[k], len = Math.max(0, Math.min(s.n, node.r) - node.l);
      if (s.mode !== "dual") node.sum = safe(f.a * node.sum + f.b * len);
      else if (k >= s.base) node.sum = mapping(f, node.sum);
      if (k < s.base) node.tag = compose(f, node.tag);
    }
    function push(k) {
      const node = s.nodes[k], f = node.tag;
      if (k >= s.base || (f.a === 1 && f.b === 0)) return;
      applyNode(k * 2, f); applyNode(k * 2 + 1, f); node.tag = id();
      emit(`push: [${node.l}, ${node.r}) → 子`, `親に保留していた (${f.a})x + (${f.b}) を子へ適用し、親の保留を id に戻します。子では古い更新の後に合成します。`, [k, k * 2, k * 2 + 1]);
    }
    function update(k, l, r, f) {
      const node = s.nodes[k];
      if (r <= node.l || node.r <= l || l === r) return;
      if (l <= node.l && node.r <= r) {
        applyNode(k, f);
        emit(`区間 [${node.l}, ${node.r}) に更新`, `x → (${f.a})x + (${f.b})。${s.mode === "dual" && k < s.base ? "値の集約を持たず、更新だけを保留します。" : "このノードを更新し、内部ノードなら子への適用を保留します。"}`, [k]);
        return;
      }
      push(k); update(k * 2, l, r, f); update(k * 2 + 1, l, r, f);
      if (s.mode !== "dual") {
        node.sum = safe(s.nodes[k * 2].sum + s.nodes[k * 2 + 1].sum);
        emit(`親 [${node.l}, ${node.r}) を再計算`, `左右の和をまとめて ${node.sum} に戻します。`, [k]);
      }
    }
    function query(k, l, r) {
      const node = s.nodes[k];
      if (r <= node.l || node.r <= l || l === r) return 0;
      if (l <= node.l && node.r <= r) {
        emit(`区間 [${node.l}, ${node.r}) を採用`, `格納している和 ${node.sum} を使います。完全に含まれる区間では子まで降りません。`, [k]);
        return node.sum;
      }
      push(k);
      return safe(query(k * 2, l, r) + query(k * 2 + 1, l, r));
    }
    function setPoint(k, p, x) {
      const node = s.nodes[k];
      if (k >= s.base) {
        node.sum = x;
        emit(`a[${p}] = ${x}`, "祖先の保留を反映してから葉を代入します。", [k]); return;
      }
      push(k);
      setPoint(p < s.nodes[k * 2].r ? k * 2 : k * 2 + 1, p, x);
      if (s.mode !== "dual") {
        node.sum = safe(s.nodes[k * 2].sum + s.nodes[k * 2 + 1].sum);
        emit(`親 [${node.l}, ${node.r}) を再計算`, `左右の和は ${node.sum}。`, [k]);
      }
    }
    let result = null, label = "更新完了";
    if (op.type === "lower") {
      let p = 0, remaining = op.w, step = 1;
      while (step <= s.n / 2) step *= 2;
      if (remaining > 0) for (; step > 0; step = Math.floor(step / 2)) {
        const next = p + step;
        if (next > s.n) { emit(`幅 ${step}: 配列外`, `候補 ${next} は n=${s.n} より大きいので飛ばします。`); continue; }
        const take = s.bit[next] < remaining;
        emit(`幅 ${step}: 候補 ${next} を${take ? "採用" : "保留"}`, `残り目標 ${remaining} と BIT[${next}]=${s.bit[next]} を比較。${take ? "目標未満なのでこのブロックを飛ばします。" : "ここまで進むと目標以上なので、幅を半分にします。"}`, [next]);
        if (take) { remaining = safe(remaining - s.bit[next]); p = next; }
      }
      result = p; label = `lower_bound(${op.w}) = ${p}`;
      emit(label, p === s.n ? "該当なし。要素数 n を返します。" : op.w <= 0 ? "非正の目標は 0 を返します。" : `最初に累積和が目標以上になる要素の添字は ${p} です。`, [], result);
    } else if (s.mode === "fenwick") {
      if (op.type === "add") addBit("BIT", op.p, op.x);
      else if (op.type === "set") addBit("BIT", op.p, safe(op.x - (prefix(s.bit, op.p + 1) - prefix(s.bit, op.p))));
      else { result = safe(sumBit("BIT", op.r) - sumBit("BIT", op.l)); label = `sum(${op.l}, ${op.r}) = ${result}`; }
    } else if (s.mode === "range-bit") {
      if (op.type === "add" && op.l !== op.r) {
        addBit("B1", op.l, op.x); addBit("B2", op.l, safe(op.l * op.x));
        if (op.r < s.n) { addBit("B1", op.r, -op.x); addBit("B2", op.r, safe(-op.r * op.x)); }
      } else if (op.type === "sum") {
        function fullPrefix(r) {
          const b1 = sumBit("B1", r), b2 = sumBit("B2", r), value = safe(r * b1 - b2);
          emit(`prefix(${r}) = ${r} × ${b1} − (${b2}) = ${value}`, "B1 は差分の和、B2 は添字×差分の和。差分が何要素に寄与するかを数えています。");
          return value;
        }
        result = safe(fullPrefix(op.r) - fullPrefix(op.l)); label = `sum(${op.l}, ${op.r}) = ${result}`;
      }
    } else if (op.type === "apply") update(1, op.l, op.r, {a: op.a, b: op.b});
    else if (op.type === "set") setPoint(1, op.p, op.x);
    else {
      const l = op.type === "get" ? op.p : op.l, r = op.type === "get" ? op.p + 1 : op.r;
      result = query(1, l, r); label = op.type === "get" ? `get(${op.p}) = ${result}` : `prod(${l}, ${r}) = ${result}`;
    }
    values(s).forEach(safe);
    if (op.type !== "lower") emit(label, result === null ? "更新を反映しました。続けて問い合わせると、保留がいつ伝播するか確認できます。" : "区間の分解と集約が完了しました。", [], result);
    frames[frames.length - 1].stable = true;
    return {state: s, frames, result};
  }
  return {modes, create, run, values, compose, mapping};
});
