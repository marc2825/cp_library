(function () {
  'use strict';
  const $=id=>document.getElementById(id), Model=window.TreeModel;
  const descriptions={
    rooted:{file:'rooted_tree',title:'部分木を連続区間にする',text:'親・深さは行きがけに、部分木サイズは帰りがけに確定します。再帰の代わりに明示的なスタックを使います。',formula:'subtree(v) = preorder[tin[v] .. tout[v])\nsize[v] = tout[v] − tin[v]',pitfall:'depth は辺の本数。dist は重みの和です。根を変えると、親・深さ・部分木も変わります。'},
    lca:{file:'doubling_lca',title:'2のべき乗で共通祖先へ',text:'深さを揃え、祖先が異なる最大の段から一緒に上げます。最後の1個上が最も近い共通祖先です。',formula:'up[b][v] = up[b−1][up[b−1][v]]\nlca / distance / jump: O(log N)',pitfall:'別成分の LCA・jump は -1。重みが0や負でも、パス上の位置は辺数で決まります。'},
    euler:{file:'euler_tour',title:'行きがけ列と往復列を使い分ける',text:'部分木の集約には各頂点1回の preorder。LCA の RMQ には親へ戻るたびに追加する walk。辺も往路と復路に記録します。',formula:'preorder: N 項\nwalk: 2N − 成分数\nedge_walk: 2E 項',pitfall:'walk の区間を単純に足すと頂点を重複して数えます。last は walk の最後の位置で、排他的な tout とは別です。'},
    diameter:{file:'diameter',title:'最遠点から、もう一度',text:'任意の頂点からの最遠点を直径の端点に選び、そこから2回目の走査を行います。木には経路が一つなので優先度付きキューは不要です。',formula:'任意の点 → 最遠点 s → 最遠点 t\n距離 dist(s,t) が直径 / O(N)',pitfall:'この2回走査は非負重みの連結な木用です。0重みの木では同じ頂点を両端にする場合もあります。'},
    rerooting:{file:'rerooting',title:'子を一つ除いて、親側の情報を返す',text:'下から集めたDPに親側の寄与を加え、全ての根の答えを求めます。prefix と suffix で、ある子の寄与だけを除きます。',formula:'exclude(i) = prefix[i] ⊕ suffix[i+1]\nlift({count,sum}, w) = {count, sum + count×w}',pitfall:'C++ の merge は結合的で単位元を持つ必要があります。可換性・逆元は不要です。親側も元の隣接位置に置き、順序を保ちます。'},
    lifting:{file:'binary_lifting',title:'一意な遷移を何回も適用する',text:'2^b 回先を前計算し、k の立っているビットだけを使います。木の祖先に限らず、閉路を含む遷移にも使えます。',formula:'jump(v,k): k を2進数で分解\n構築 O(N log(K+1)) / 問合せ O(log(k+1))',pitfall:'-1 は遷移先なし。C++ の既定上限は uint64 の最大値（64段）。図解は入力 k に必要な段だけを表示します。'},
    dp:{file:'tree_dp',title:'子が揃ってから親を計算する',text:'独立集合の個数を例に、各頂点を白／黒にする場合を分けます。隣接する頂点を両方黒にしないよう、子の状態を組み合わせます。',formula:'white[v] = ∏(white[c] + black[c])\nblack[v] = ∏white[c]',pitfall:'空の積は1。単位元は問題ごとに変わります。C++ のテンプレートでは、子の集約 S と頂点の状態 R を別の型にできます。'}
  };
  let mode='rooted', trace, index=0, timer=null;
  const format=value=>value===null||value===undefined?'—':typeof value==='object'?('white'in value?`${value.white} / ${value.black}`:`${value.count} / ${value.sum}`):String(value);
  const stop=()=>{if(timer)clearInterval(timer);timer=null;$('play').textContent='再生 ▶';};
  function read(){return {mode,n:$('vertices').value,edges:$('edges').value,next:$('successors').value,root:$('root').value,s:$('source').value,t:$('target').value,k:$('steps').value};}
  function execute(){stop();try{const next=Model.run(read());trace=next;index=0;$('input-error').textContent='';render();}catch(error){$('input-error').textContent=error.message;}}
  function scenario(type){
    let n=7, edges=[[0,1,2],[0,2,1],[1,3,3],[1,4,0],[2,5,4],[2,6,2]], next=[-1,0,0,1,1,2,2];
    if(type==='path'){edges=Array.from({length:6},(_,i)=>[i,i+1,1]);next=[1,2,3,4,5,6,-1];}
    if(type==='star'){edges=Array.from({length:6},(_,i)=>[0,i+1,1]);next=[-1,0,0,0,0,0,0];}
    if(type==='forest'){edges=[[0,1,2],[1,2,3],[3,4,1]];next=[-1,0,1,-1,3,-1,-1];}
    if(type==='zero')edges=edges.map(([u,v])=>[u,v,0]);
    if(type==='negative')edges=edges.map(([u,v,w],i)=>[u,v,i%2?-w:w]);
    if(type==='empty'){n=0;edges=[];next=[];}
    if(type==='cycle')next=[1,2,0,2,3,6,5];
    $('vertices').value=n;$('edges').value=edges.map(e=>e.join(' ')).join('\n');$('successors').value=next.join(' ');$('root').value=0;$('source').value=n?3:0;$('target').value=n?5:0;$('steps').value=type==='cycle'?'18446744073709551615':'2';execute();
  }
  function setMode(next){mode=Model.MODES.includes(next)?next:'rooted';stop();document.querySelectorAll('[data-mode]').forEach(button=>button.setAttribute('aria-pressed',String(button.dataset.mode===mode)));
    for(const option of $('scenario-type').options){const irrelevant=mode==='lifting'?['zero','negative'].includes(option.value):option.value==='cycle';option.hidden=irrelevant;option.disabled=irrelevant;}
    $('edges-field').hidden=mode==='lifting';$('next-field').hidden=mode!=='lifting';$('root-field').hidden=['diameter','lifting'].includes(mode);$('query-fields').hidden=!['lca','lifting'].includes(mode);$('target-field').hidden=mode==='lifting';
    const info=descriptions[mode];$('principle-title').textContent=info.title;$('principle').textContent=info.text;$('formula').textContent=info.formula;$('pitfall').textContent=info.pitfall;$('docs-link').href=`../../docs/tree/${info.file}.md`;$('code-link').href=`../../tree/${info.file}.hpp`;
    $('result-label').textContent={lca:'LCA',diameter:'直径の長さ',lifting:'移動先',dp:'独立集合の個数'}[mode]||'結果';$('scenario-type').value='balanced';scenario('balanced');
  }
  const svgNS='http://www.w3.org/2000/svg';
  function element(tag,attributes={},text){const node=document.createElementNS(svgNS,tag);for(const [key,value]of Object.entries(attributes))node.setAttribute(key,value);if(text!==undefined)node.textContent=text;return node;}
  function draw(frame){
    const svg=$('diagram');svg.replaceChildren();const {config,tree}=trace,n=config.n;
    const defs=element('defs'),marker=element('marker',{id:'arrow',viewBox:'0 0 10 10',refX:26,refY:5,markerWidth:6,markerHeight:6,orient:'auto-start-reverse'});marker.append(element('path',{d:'M 0 0 L 10 5 L 0 10 z',fill:'#66877b'}));defs.append(marker);svg.append(defs);
    if(!n){svg.append(element('text',{x:420,y:230,'text-anchor':'middle',class:'caption'},'空の入力'));return;}
    const positions=Array(n);
    if(mode==='lifting')for(let v=0;v<n;++v){const angle=2*Math.PI*v/n-Math.PI/2;positions[v]=[420+180*Math.cos(angle),225+155*Math.sin(angle)];}
    else{
      let leaves=0;const raw=Array(n);
      for(const v of tree.post){const children=tree.adj[v].filter(e=>e.id!==tree.pe[v]);raw[v]=children.length?children.reduce((s,e)=>s+raw[e.to],0)/children.length:leaves++;}
      const maxDepth=Math.max(...tree.depth);
      for(let v=0;v<n;++v)positions[v]=[leaves===1?420:60+raw[v]*720/(leaves-1),65+tree.depth[v]*300/Math.max(1,maxDepth)];
    }
    const pathEdges=new Set();if(frame.path)for(let i=1;i<frame.path.length;++i)for(const e of tree.adj[frame.path[i-1]])if(e.to===frame.path[i])pathEdges.add(e.id);
    const edges=mode==='lifting'?config.next.flatMap((v,u)=>v===-1?[]:[[u,v,1,u]]):config.edges;
    for(const[u,v,w,id]of edges){const[a,b]=positions[u],[c,d]=positions[v];let curve=`M ${a} ${b} L ${c} ${d}`;
      if(mode==='lifting')curve=u===v?`M ${a-12} ${b-15} C ${a-65} ${b-80},${a+65} ${b-80},${a+12} ${b-15}`:`M ${a} ${b} Q ${(a+c)/2+(d-b)*.12} ${(b+d)/2-(c-a)*.12} ${c} ${d}`;
      const active=frame.activeEdges.includes(id)||(mode==='lifting'&&frame.focus.includes(u));const edge=element('path',{d:curve,class:`graph-edge${pathEdges.has(id)?' selected':''}${active?' active':''}`});if(mode==='lifting')edge.setAttribute('marker-end','url(#arrow)');svg.append(edge);
      if(mode!=='lifting')svg.append(element('text',{x:(a+c)/2+10,y:(b+d)/2-4,'text-anchor':'middle',class:'edge-label'},`${id} / ${w}`));
    }
    for(let v=0;v<n;++v){const[x,y]=positions[v],active=frame.focus.includes(v),done=mode==='lifting'?frame.at===v:frame.values[v]!==null||frame.sub[v]!==null&&frame.visited.includes(v);svg.append(element('circle',{cx:x,cy:y,r:23,fill:active?'#fff0de':done?'#eaf3ed':'white',stroke:active?'#bd6935':'#66877b','stroke-width':active?3:2}));svg.append(element('text',{x,y:y+6,'text-anchor':'middle',class:'node-value'},v));
      let info='';if(mode==='rooted'||mode==='euler')info=`depth ${format(frame.depth[v])} / size ${format(frame.sub[v])}`;if(mode==='lca')info=`depth ${tree.depth[v]}`;if(mode==='diameter')info=`dist ${format(frame.dist[v])}`;if(mode==='dp')info=`白 / 黒: ${format(frame.values[v])}`;if(mode==='rerooting')info=`count / sum: ${format(frame.answers?.[v]||frame.values[v])}`;
      svg.append(element('text',{x,y:y+42,'text-anchor':'middle',class:'node-info'},info));}
    const caption=mode==='lifting'?`現在位置: ${format(frame.at)} / k: ${config.k}`:mode==='diameter'?`今回の走査の始点: ${frame.scanStart}`:`${mode==='rerooting'?'初回DPの根':'根'}: ${tree.roots.join(', ')} / 成分数: ${tree.roots.length}`;
    svg.append(element('text',{x:22,y:443,class:'caption'},caption));
  }
  function row(values,highlight=false){const tr=document.createElement('tr');if(highlight)tr.className='active';for(const value of values){const td=document.createElement('td');td.textContent=format(value);tr.append(td);}return tr;}
  function table(frame){
    const head=$('table-head'),body=$('values');head.replaceChildren();body.replaceChildren();
    if(frame.up){const tr=document.createElement('tr');for(const value of ['b / 2^b',...Array.from({length:trace.config.n},(_,v)=>v)]){const th=document.createElement('th');th.textContent=value;tr.append(th);}head.append(tr);frame.up.forEach((values,b)=>{const entry=row([`${b} / ${String(1n<<BigInt(b))}`,...values]);for(const[bit,v]of frame.highlights)if(bit===b)entry.children[v+1].classList.add('highlight');body.append(entry);});$('table-title').textContent='2のべき乗の遷移表';$('table-note').textContent='列は頂点番号。橙色は今回参照／構築したセルです。';}
    else{const labels=['v','parent','depth','size','tin','tout','dist'];if(['dp','rerooting'].includes(mode))labels.push(mode==='dp'?'白 / 黒':'down: 個数 / 和');if(mode==='rerooting')labels.push('答え: 個数 / 和');const tr=document.createElement('tr');for(const label of labels){const th=document.createElement('th');th.textContent=label;tr.append(th);}head.append(tr);for(let v=0;v<trace.config.n;++v){const values=[v,frame.parent[v],frame.depth[v],frame.sub[v],frame.tin[v],frame.tout[v],frame.dist[v]];if(['dp','rerooting'].includes(mode))values.push(frame.values[v]);if(mode==='rerooting')values.push(frame.answers?.[v]);body.append(row(values,frame.focus.includes(v)));}$('table-title').textContent='頂点ごとの状態';$('table-note').textContent='— は未確定。木DPの基礎情報は前計算済みの値を表示します。';}
    $('sequences').replaceChildren();const sequences=mode==='euler'?['pre','walk','edgeWalk','post']:['rooted','dp'].includes(mode)?['pre','post']:[];
    for(const key of sequences){const label=document.createElement('p');label.className='sequence-label';label.textContent={pre:'preorder（行きがけ・部分木区間用）',post:'postorder（帰りがけ・木DP用）',walk:'walk（往復の頂点列）',edgeWalk:'edge_walk（往復の辺ID）'}[key];const list=document.createElement('div');list.className='sequence';(frame[key]||[]).forEach((v,i)=>{const chip=document.createElement('span');chip.textContent=`${i}: ${v}`;if(frame.focus.includes(v)&&key!=='edgeWalk')chip.className='selected';list.append(chip);});$('sequences').append(label,list);}
    $('prefix-wrap').hidden=!frame.prefix;$('prefix-values').replaceChildren();
    if(frame.prefix){$('prefix-note').textContent=`中心 ${frame.center} / prefix[i] は [0,i)、suffix[i] は [i,degree)。表示値は 個数 / 距離和。`;for(let i=0;i<frame.prefix.length;++i){const entry=row([i,frame.neighbors[i],frame.incoming[i],frame.prefix[i],frame.suffix[i]]);if(i===frame.excluded)entry.className='excluded';$('prefix-values').append(entry);}}
  }
  function render(){if(!trace)return;const frame=trace.frames[index];$('phase-value').textContent=frame.phase;$('result').textContent=format(frame.result);$('focus-value').textContent=frame.focus.join(', ')||'—';$('step-title').textContent=frame.title;$('step-description').textContent=frame.description;$('rule').textContent=frame.rule;$('rule').hidden=!frame.rule;$('extra').textContent=frame.extra;$('progress').textContent=`${index+1} / ${trace.frames.length}`;$('previous').disabled=index===0;$('next').disabled=index===trace.frames.length-1;$('milestone').disabled=$('next').disabled;$('finish').disabled=$('next').disabled;draw(frame);table(frame);}
  $('run').onclick=execute;$('scenario').onclick=()=>scenario($('scenario-type').value);$('previous').onclick=()=>{stop();index=Math.max(0,index-1);render();};$('next').onclick=()=>{stop();index=Math.min(trace.frames.length-1,index+1);render();};$('finish').onclick=()=>{stop();index=trace.frames.length-1;render();};$('milestone').onclick=()=>{stop();const phase=trace.frames[index].phase;do{index++;}while(index<trace.frames.length-1&&trace.frames[index].phase===phase);render();};$('play').onclick=()=>{if(timer){stop();return;}if(index===trace.frames.length-1)index=0;timer=setInterval(()=>{index=Math.min(trace.frames.length-1,index+1);render();if(index===trace.frames.length-1)stop();},500);$('play').textContent='停止 ▮▮';};
  document.querySelectorAll('[data-mode]').forEach(button=>button.onclick=()=>{if(location.hash.slice(1)===button.dataset.mode)setMode(button.dataset.mode);else location.hash=button.dataset.mode;});window.addEventListener('hashchange',()=>setMode(location.hash.slice(1)));setMode(location.hash.slice(1));
})();
