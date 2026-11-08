(function (root) {
  'use strict';
  const MODES = ['rooted', 'lca', 'euler', 'diameter', 'rerooting', 'lifting', 'dp'];
  const clone = value => JSON.parse(JSON.stringify(value));
  const integer = (value, min, max, label) => {
    const n = Number(value);
    if (String(value).trim() === '' || !Number.isSafeInteger(n) || n < min || n > max) throw new Error(`${label}は ${min}〜${max} の整数にしてください。`);
    return n;
  };
  function normalize(input) {
    const mode = input.mode || 'rooted';
    if (!MODES.includes(mode)) throw new Error('未対応のモードです。');
    const n = integer(input.n, 0, 12, '頂点数');
    const start = n && !['diameter', 'lifting'].includes(mode) ? integer(input.root ?? 0, 0, n - 1, '根') : 0;
    const s = n && ['lca', 'lifting'].includes(mode) ? integer(input.s ?? 0, 0, n - 1, '始点') : 0;
    const t = n && mode === 'lca' ? integer(input.t ?? n - 1, 0, n - 1, '終点') : 0;
    let k;
    try { if (!/^\d+$/.test(String(input.k ?? 0).trim())) throw new Error(); k = BigInt(String(input.k ?? 0).trim()); }
    catch { throw new Error('移動回数は0以上の整数にしてください。'); }
    if (k > (1n << 64n) - 1n) throw new Error('移動回数は uint64 の範囲内にしてください。');
    if (mode === 'lifting') {
      const next = typeof input.next === 'string' ? input.next.trim().split(/[\s,]+/).filter(Boolean) : input.next || [];
      if (next.length !== n) throw new Error('遷移先を頂点数と同じ個数だけ入力してください。');
      return {mode, n, root: start, s, t, k: String(k), next: next.map(v => integer(v, -1, n - 1, '遷移先')), edges: []};
    }
    let edges = input.edges || [];
    if (typeof edges === 'string') edges = edges.trim().split('\n').filter(line => line.trim()).map(line => {
      const values = line.trim().split(/\s+/);
      if (values.length !== 2 && values.length !== 3) throw new Error('辺は u v または u v w の2個／3個で入力してください。');
      return values;
    });
    const component = Array.from({length:n}, (_, v) => v);
    edges = edges.map((edge, id) => {
      const u = integer(edge[0], 0, n - 1, '辺の頂点'), v = integer(edge[1], 0, n - 1, '辺の頂点');
      const w = integer(edge[2] ?? 1, -50, 50, '重み');
      if (component[u] === component[v]) throw new Error('閉路があります。自己ループ・多重辺も木には使えません。');
      const old = component[v], replacement = component[u];
      for (let x = 0; x < n; ++x) if (component[x] === old) component[x] = replacement;
      if (mode === 'diameter' && w < 0) throw new Error('直径の2回走査には非負重みが必要です。');
      return [u, v, w, id];
    });
    if (mode === 'diameter' && n && edges.length !== n - 1) throw new Error('直径は連結な木にしてください。');
    return {mode, n, root:start, s, t, k:String(k), edges};
  }
  function orient(config) {
    const {n, edges, root: start} = config;
    const adj = Array.from({length:n}, () => []);
    for (const [u, v, w, id] of edges) { adj[u].push({to:v,w,id}); adj[v].push({to:u,w,id}); }
    const tree = {adj, roots:[], parent:Array(n).fill(-1), pe:Array(n).fill(-1), depth:Array(n).fill(0), dist:Array(n).fill(0), component:Array(n).fill(-1), sub:Array(n).fill(1), tin:Array(n).fill(-1), tout:Array(n).fill(-1), pre:[], post:[], walk:[], edgeWalk:[], first:Array(n).fill(-1), last:Array(n).fill(-1)};
    const events = [];
    const enter = v => { tree.tin[v] = tree.pre.length; tree.pre.push(v); tree.first[v] = tree.walk.length; tree.walk.push(v); events.push({kind:'enter', v}); };
    for (const r of n ? [start, ...Array.from({length:n}, (_, v) => v)] : []) {
      if (tree.component[r] !== -1) continue;
      tree.component[r] = tree.roots.length; tree.roots.push(r); enter(r);
      const stack = [[r, 0]];
      while (stack.length) {
        const top = stack[stack.length - 1], v = top[0];
        if (top[1] < adj[v].length) {
          const e = adj[v][top[1]++];
          if (e.id === tree.pe[v]) continue;
          tree.parent[e.to] = v; tree.pe[e.to] = e.id; tree.component[e.to] = tree.component[v];
          tree.depth[e.to] = tree.depth[v] + 1; tree.dist[e.to] = tree.dist[v] + e.w;
          tree.edgeWalk.push(e.id); enter(e.to); stack.push([e.to, 0]);
        } else {
          tree.last[v] = tree.walk.length - 1; tree.tout[v] = tree.pre.length; tree.post.push(v);
          events.push({kind:'exit', v}); stack.pop();
          if (tree.parent[v] !== -1) { const p = tree.parent[v]; tree.sub[p] += tree.sub[v]; tree.walk.push(p); tree.edgeWalk.push(tree.pe[v]); events.push({kind:'return', v:p, child:v, edge:tree.pe[v]}); }
        }
      }
    }
    return {tree, events};
  }
  function run(input) {
    const config = normalize(input), {mode,n} = config;
    const frames = [];
    let state = {phase:'準備', focus:[], activeEdges:[], visited:[], result:null, rule:'', extra:'', values:Array(n).fill(null), highlights:[]};
    const save = (title, description, patch={}) => { Object.assign(state, {focus:[], activeEdges:[], highlights:[], rule:'', extra:''}, patch); frames.push(clone({...state,title,description})); };
    if (mode === 'lifting') {
      const steps = BigInt(config.k), levels = Math.max(1, steps.toString(2).length);
      const up = [config.next.slice()]; state.up = up; state.at = config.s; state.remaining = config.k;
      save('1回先を登録', '-1 は遷移先なし。閉路もそのまま扱えます。', {phase:'前計算'});
      for (let b = 1; b < levels; ++b) {
        up.push(config.next.map((_,v) => up[b-1][v] === -1 ? -1 : up[b-1][up[b-1][v]]));
        save(`2^${b} 回先を構築`, '半分の移動を2回つなぎます。', {highlights:Array.from({length:n}, (_,v) => [b,v]), rule:`up[${b}][v] = up[${b-1}][up[${b-1}][v]]`});
      }
      let at = config.s, remaining = steps;
      if (n) for (let b = 0; remaining && at !== -1; ++b, remaining >>= 1n) if (remaining & 1n) {
        const before = at; at = up[b][at]; state.at = at; state.remaining = String((remaining - 1n) << BigInt(b));
        save(`2^${b} 回進む`, `${before} → ${at}。立っているビットだけを使います。`, {phase:'問い合わせ', focus:[before,at].filter(v=>v>=0), highlights:[[b,before]], rule:`k の bit ${b} = 1`});
      }
      save('移動完了', n ? 'k=0 なら始点自身。途中で遷移が尽きると -1 です。' : '空の遷移表には問い合わせを行いません。', {phase:'完了', result:n ? at : '空'});
      return {config, tree:null, frames};
    }
    const {tree,events} = orient(config);
    Object.assign(state, {parent:tree.parent, depth:tree.depth, sub:tree.sub, tin:tree.tin, tout:tree.tout, dist:tree.dist, pre:tree.pre, post:tree.post});
    if (!n) { save('空の森', '前計算結果とDPの答えは空列。直径は長さ0・端点なしです。', {phase:'完了',result:mode==='diameter'?0:'空'}); return {config,tree,frames}; }
    if (mode === 'rooted' || mode === 'euler') {
      Object.assign(state, {parent:Array(n).fill(null), depth:Array(n).fill(null), sub:Array(n).fill(null), tin:Array(n).fill(null), tout:Array(n).fill(null), dist:Array(n).fill(null), pre:[], post:[], walk:[], edgeWalk:[]});
      save('根から探索を始める', '指定した根の成分から始め、残る成分も頂点番号順に処理します。');
      for (const event of events) {
        const v = event.v;
        if (event.kind === 'enter') {
          state.parent[v] = tree.parent[v]; state.depth[v] = tree.depth[v]; state.dist[v] = tree.dist[v]; state.tin[v] = state.pre.length; state.pre.push(v); state.walk.push(v);
          if (tree.pe[v] !== -1) state.edgeWalk.push(tree.pe[v]);
          state.visited.push(v);
          save(`${v} に入る`, '行きがけ位置 tin と深さを確定します。根の親は -1 です。', {phase:'行きがけ',focus:[v],activeEdges:tree.pe[v] === -1 ? [] : [tree.pe[v]],rule:`tin[${v}] = ${state.tin[v]}`});
        } else if (event.kind === 'exit') {
          state.sub[v] = tree.sub[v]; state.tout[v] = state.pre.length; state.post.push(v);
          save(`${v} の部分木が完了`, '部分木は行きがけ列の連続区間。子の結果が先に揃うのが帰りがけ順です。', {phase:'帰りがけ',focus:[v],rule:`[tin,tout) = [${state.tin[v]},${state.tout[v]}) / size = ${state.sub[v]}`});
        } else {
          state.walk.push(v); state.edgeWalk.push(event.edge);
          save(`${event.child} から ${v} に戻る`, '往復の頂点列には親をもう一度追加し、同じ辺IDを復路にも記録します。', {phase:'復路',focus:[event.child,v],activeEdges:[event.edge]});
        }
      }
      save('探索完了', 'preorder は各頂点1回、walk は成分ごとに 2N−1 回。部分木クエリには preorder を使います。', {phase:'完了',result:mode==='rooted'?`${tree.roots.length} 成分`:`${tree.walk.length} 項`});
    } else if (mode === 'lca') {
      const levels = Math.max(1, Math.floor(Math.log2(n)) + 1), up = [tree.parent.slice()]; state.up = up;
      save('親と深さを準備', '各成分の根の祖先は -1。表の列は頂点、行は2のべき乗です。');
      for (let b=1;b<levels;++b) { up.push(up[b-1].map(v=>v===-1?-1:up[b-1][v])); save(`祖先表 2^${b}`, '2^(b−1) 個上を2回たどります。', {phase:'前計算',highlights:Array.from({length:n},(_,v)=>[b,v])}); }
      let a=config.s,b=config.t;
      if (tree.component[a] !== tree.component[b]) save('別の成分', '共通祖先と木上のパスはありません。', {phase:'完了',result:-1});
      else {
        if (tree.depth[a] < tree.depth[b]) [a,b]=[b,a];
        let difference=tree.depth[a]-tree.depth[b];
        for (let bit=0;difference;difference>>=1,++bit) if (difference&1) { const old=a; a=up[bit][a]; save('深さを揃える', `${old} → ${a}。深い側を 2^${bit} 個上げます。`, {phase:'深さ調整',focus:[a,b],highlights:[[bit,old]]}); }
        if (a!==b) {
          for (let bit=levels-1;bit>=0;--bit) if (up[bit][a]!==up[bit][b]) { const aa=a,bb=b; a=up[bit][a]; b=up[bit][b]; save('異なる祖先まで一緒に上がる', '祖先が異なる最大の段だけ進み、LCAを飛び越さないようにします。', {phase:'同時移動',focus:[a,b],highlights:[[bit,aa],[bit,bb]]}); }
          a=tree.parent[a];
        }
        const length=tree.dist[config.s]+tree.dist[config.t]-2*tree.dist[a];
        const upward=[];for(let v=config.s;v!==a;v=tree.parent[v])upward.push(v);upward.push(a);
        const downward=[];for(let v=config.t;v!==a;v=tree.parent[v])downward.push(v);
        state.path=upward.concat(downward.reverse());
        const k=BigInt(config.k), jump=k<BigInt(state.path.length)?state.path[Number(k)]:-1;
        save('LCA とパスを確定', `重み付き距離 ${length} / 辺数 ${state.path.length-1} / k=${config.k} 番目は ${jump}。`, {phase:'完了',focus:[a],result:a,extra:`path: ${state.path.join(' → ')}`,rule:'dist(s,t) = dist(root,s) + dist(root,t) − 2 dist(root,LCA)'});
      }
    } else if (mode === 'diameter') {
      let start=0, best=0, finalPath=[];
      for (let pass=0;pass<2;++pass) {
        const queue=[start], parent=Array(n).fill(-1), seen=new Set([start]), distances=Array(n).fill(null);distances[start]=0;
        save(`第${pass+1}走査を開始`, `${start} から全頂点への距離を求めます。木なので一意なパスをたどれば十分です。`, {phase:`第${pass+1}走査`,scanStart:start,dist:distances,focus:[start]});
        for (let i=0;i<queue.length;++i) { const v=queue[i]; for(const e of tree.adj[v])if(!seen.has(e.to)){seen.add(e.to);parent[e.to]=v;distances[e.to]=distances[v]+e.w;queue.push(e.to);save(`${e.to} まで到達`, `距離 ${distances[e.to]} を確定。`, {dist:distances,focus:[e.to],activeEdges:[e.id]});} }
        best=0;for(let v=0;v<n;++v)if(distances[v]>distances[best])best=v;
        if(!pass){start=best;save('直径の端点を一つ選ぶ', `最遠の ${best} を次の始点にします。`, {focus:[best]});}
        else { for(let v=best;v!==-1;v=parent[v])finalPath.push(v);finalPath.reverse();state.path=finalPath;save('直径を復元', '2回目の最遠点から親を逆にたどります。同じ長さの端点は一意とは限りません。', {phase:'完了',result:distances[best],focus:finalPath,extra:`endpoints: ${start}, ${best} / path: ${finalPath.join(' → ')}`}); }
      }
    } else if (mode === 'dp') {
      const values=Array(n).fill(null);state.values=values;
      save('独立集合の数を木DPで数える', '白は子を白／黒から選べます。黒は子を白に限定します。図解の12頂点以内では整数をそのまま保持します。');
      for(const v of tree.post) {
        let white=1,black=1;
        for(const e of tree.adj[v])if(e.id!==tree.pe[v]) { white*=values[e.to].white+values[e.to].black;black*=values[e.to].white;save(`子 ${e.to} の寄与を併合`, `頂点 ${v}: 白=${white}、黒=${black}。`, {phase:'子の併合',focus:[v,e.to],activeEdges:[e.id],rule:'white *= child.white + child.black; black *= child.white'}); }
        values[v]={white,black};save(`${v} を確定`, '葉は白／黒それぞれ1通り。子の状態を使い終えてから親へ渡します。', {phase:'帰りがけDP',focus:[v]});
      }
      const total=tree.roots.reduce((a,v)=>a*(values[v].white+values[v].black),1);
      save('森全体の数を計算', '各成分は独立なので、根で得られた場合の数を掛け合わせます。', {phase:'完了',result:total});
    } else if (mode === 'rerooting') {
      const down=Array(n).fill(null), fromParent=Array.from({length:n},()=>({count:0,sum:0})), answers=Array(n).fill(null);
      const merge=(a,b)=>({count:a.count+b.count,sum:a.sum+b.sum}), vertex=a=>({count:a.count+1,sum:a.sum}), lift=(a,w)=>({count:a.count,sum:a.sum+a.count*w}), zero=()=>({count:0,sum:0});
      state.values=down;state.answers=answers;state.messages=[];
      save('距離の総和を全ての根で求める', '寄与は {個数, 距離和}。辺を越えると、その先の全頂点に辺重みが加わります。');
      for(const v of tree.post){let aggregate=zero();for(const e of tree.adj[v])if(e.id!==tree.pe[v])aggregate=merge(aggregate,lift(down[e.to],e.w));down[v]=vertex(aggregate);save(`${v} の下向きDP`, `count=${down[v].count}, sum=${down[v].sum}。`, {phase:'下から上へ',focus:[v]});}
      for(const v of tree.pre){
        const neighbors=tree.adj[v], incoming=neighbors.map(e=>e.id===tree.pe[v]?fromParent[v]:lift(down[e.to],e.w)), prefix=[zero()], suffix=Array(neighbors.length+1);suffix[neighbors.length]=zero();
        for(const message of incoming)prefix.push(merge(prefix[prefix.length-1],message));
        for(let i=neighbors.length-1;i>=0;--i)suffix[i]=merge(incoming[i],suffix[i+1]);
        state.neighbors=neighbors.map(e=>e.to);state.incoming=incoming;state.prefix=prefix;state.suffix=suffix;state.excluded=null;state.center=v;
        answers[v]=vertex(prefix[neighbors.length]);
        save(`${v} を根とした答え`, '親側の寄与を元の隣接位置に入れて、全ての寄与をまとめます。', {phase:'全方位の答え',focus:[v],rule:`sum[${v}] = ${answers[v].sum}`});
        for(let i=0;i<neighbors.length;++i){const e=neighbors[i];if(e.id===tree.pe[v])continue;
          const excluded=merge(prefix[i],suffix[i+1]);fromParent[e.to]=lift(vertex(excluded),e.w);state.excluded=i;state.messages.push({from:v,to:e.to,...fromParent[e.to]});
          save(`子 ${e.to} を除いて渡す`, `prefix[${i}] と suffix[${i+1}] を結合し、${v} 自身と辺重みを加えます。`, {phase:'上から下へ',focus:[v,e.to],activeEdges:[e.id],rule:`exclude = prefix[${i}] ⊕ suffix[${i+1}]`,extra:`${v} → ${e.to}: count=${fromParent[e.to].count}, sum=${fromParent[e.to].sum}`});
        }
      }
      state.excluded=null;
      save('全頂点の答えが揃った', '森では同じ成分の頂点への距離和を返します。各辺の向きごとに1回ずつ情報を渡すので線形時間です。', {phase:'完了',result:answers.map(a=>a.sum).join(', ')});
    }
    return {config,tree,frames};
  }
  const api={MODES,normalize,orient,run};
  if(typeof module!=='undefined'&&module.exports)module.exports=api;
  else root.TreeModel=api;
})(typeof globalThis!=='undefined'?globalThis:this);
