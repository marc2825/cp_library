(function(root){
'use strict';
const specs={},paths={};
function spec(mode,title,path,sample,note){specs[mode]=[title,sample,note];paths[mode]=path;}
spec('cyclic_sequence','循環列の操作','basic_utility/cyclic_sequence',{a:[1,2,3,4],shift:-1,start:3,count:6},'負の添字を正規化し、回転と循環区間を追います。');
spec('interval_operations','区間のマージ・分割','basic_utility/interval_operations',{intervals:[[1,4],[3,6],[8,10]],cuts:[2,5]},'重なる区間の併合と、内部の点による分割を追います。');
spec('quotient_enumeration','商列挙','basic_utility/quotient_enumeration',{n:23},'floor(n/i)の同じ閉区間を一度に進めます。');
spec('multiple_enumeration','範囲内の倍数列挙','basic_utility/multiple_enumeration',{l:0,r:26,step:5},'最初の倍数と、加算前の残り距離を確認します。');
spec('gray_code','Gray Code','basic_utility/gray_code',{bits:4},'通常の二進数との対応と、変化した1bitを確認します。');
spec('subset_pairs','部分集合の部分集合列挙','basic_utility/subset_pairs',{n:3},'各Sに含まれるTを降順で列挙します。空集合も含みます。');
spec('permutation_rank','順列のランク・アンランク','basic_utility/permutation_rank',{a:[2,0,3,1],rank:17},'未使用の候補数からLehmer符号を作り、順位からも復元します。');
const common={n:6,ops:[['merge',0,1],['merge',2,3],['merge',1,3]]};
spec('partially_persistent_disjoint_set_union','部分永続Union-Find','union_find/partially_persistent_disjoint_set_union',{n:5,ops:[['merge',0,1],['merge',1,2],['same',0,0,1],['size',1,0]]},'親を付けた時刻と、過去のサイズを確認します。');
spec('persistent_disjoint_set_union','完全永続Union-Find','union_find/persistent_disjoint_set_union',{n:5,ops:[['merge',0,0,1],['merge',0,1,2],['same',1,0,1],['same',2,0,1]]},'任意の版から併合を分岐します。図では連番の版IDを使います。');
spec('aggregate_disjoint_set_union','連結成分のモノイド集約','union_find/aggregate_disjoint_set_union',{n:5,values:[1,2,3,4,5],ops:[['merge',0,1],['add',1,8],['merge',1,2],['prod',0]]},'根が保持する成分和と、寄与の追加を追います。');
spec('bipartite_disjoint_set_union','二部グラフ判定付きUnion-Find','union_find/bipartite_disjoint_set_union',{n:5,ops:[['edge',0,1],['edge',1,2],['edge',2,0],['edge',3,4],['edge',2,3]]},'親との色差をXORで保持し、奇閉路を記録します。');
spec('interval_disjoint_set_union','区間Union-Find','union_find/interval_disjoint_set_union',{n:8,ops:[['merge',1,5],['merge',3,7],['merge',2,6]]},'処理済みの隣接境界をnextで飛ばします。');
spec('merge_tree','マージ履歴木','union_find/merge_tree',common,'成功した併合ごとに二つの旧根を子にします。');
for(const [m,t] of [['dynamic_segtree','Dynamic Segtree'],['dynamic_lazy_segtree','Dynamic Lazy Segtree'],['range_actions','Add/Assign/Affineと区間和モノイド']])spec(m,t,'range_query/'+m,{n:16,ops:m==='dynamic_segtree'?[['set',3,7],['set',12,5],['sum',2,14]]:[['affine',2,13,2,3],['assign',5,9,1],['add',0,7,2],['sum',3,12]]},'区間の分割・集約と作用の合成を追います。');
for(const [m,t,c] of [['fenwick_tree_2d','二次元Fenwick Tree','range_query'],['segtree_2d','二次元Segtree','range_query'],['compressed_fenwick_tree_2d','二次元BIT（座圧版）','offline_query']])spec(m,t,c+'/'+m,{h:4,w:5,points:[[1,2],[3,4],[2,1]],ops:[['add',1,2,7],['add',3,4,9],['sum',0,0,3,4]]},'点の更新先と、長方形に寄与する内部セルを確認します。');
spec('disjoint_sparse_table','Disjoint Sparse Table','static_query/disjoint_sparse_table',{a:[2,3,5,7,11,13],queries:[[1,5],[2,2],[3,4]]},'分割点の左右のsuffix/prefixを構築し、重複なしで結合します。');
spec('binary_trie','Binary Trie','static_query/binary_trie',{bits:4,ops:[['insert',1],['insert',6],['insert',6],['xor',3],['kth',1],['xor_min',7],['erase',5]]},'bitごとの個数、XORマスクと検索分岐を追います。');
spec('fast_set','Fast Set / Predecessor Set','static_query/fast_set',{n:130,ops:[['insert',3],['insert',64],['insert',129],['next',4],['prev',128],['erase',64]]},'下位の非空ワードを上位ビットで索引化します。');
for(const [m,t]of [['treap','Treap'],['rbst','RBST'],['ordered_set','Ordered Set']])spec(m,t,'ordered_structure/'+m,{seed:7,ops:[['insert',5],['insert',1],['insert',3],['insert',3],['kth',2],['rank',5],['erase',3]]},'木の形・部分木サイズ・重複個数と、順位検索の道を追います。');
spec('lazy_reversible_treap','Lazy Reversible Treap','ordered_structure/lazy_reversible_treap',{a:[1,2,3,4],ops:[['affine',0,3,2,1],['reverse',0,4],['insert',2,9],['sum',1,4]]},'列の区間を切り出し、反転とアフィン作用を合成します。図は論理列です。');
for(const [m,t]of [['persistent_stack','永続Stack'],['persistent_queue','永続Queue']])spec(m,t,'ordered_structure/'+m,{ops:[['push',0,4],['push',1,9],['pop',2],['push',1,7],['front',3]]},'版を共有して分岐します。根ハンドルは図では連番の版IDです。');
spec('range_set','Range Set','offline_query/range_set',{ops:[['insert',1,8],['erase',3,5],['insert',7,11],['contains',4],['covers',5,10]]},'半開区間の追加で併合し、削除で分割します。');
spec('offline_rmq','Offline RMQ','offline_query/offline_rmq',{a:[4,2,2,7,1],queries:[[0,4],[1,3],[2,5],[2,2]]},'単調スタックと次の小さい添字への親リンクを追います。');
spec('rollback_mo','Rollback Mo','offline_query/rollback_mo',{a:[2,3,5,7,11,13],queries:[[0,2],[1,6],[0,5],[3,6]],block:2},'右側を単調に伸ばし、左側の一時追加だけsnapshotまで戻します。');
spec('mo_on_tree','Mo on Tree','offline_query/mo_on_tree',{n:6,edges:[[0,1],[1,2],[1,3],[3,4],[3,5]],queries:[[2,4],[4,5],[0,2]]},'往復Euler列の偶奇で頂点をtoggleし、必要なLCAを一時追加します。');
function integer(x,l=-100000,r=100000){if(!Number.isSafeInteger(x)||x<l||x>r)throw Error(`整数は${l}〜${r}の範囲で指定してください`);return x;}
function array(x,max=40){if(!Array.isArray(x)||x.length>max)throw Error(`配列は${max}項目以内で指定してください`);return x;}
function values(x,max=16){return array(x,max).map(v=>integer(v));}
function build(mode,input){
 if(!Object.hasOwn(specs,mode)||!input||typeof input!=='object'||Array.isArray(input))throw Error('入力はJSONオブジェクトで指定してください');
 const frames=[];let result=null;
 function emit(title,note,columns,rows,extra={}){for(const row of rows)for(const v of row)if(typeof v==='number'&&!Number.isSafeInteger(v))throw Error('計算結果が図解の安全な整数範囲を超えました');if(frames.length>=2500)throw Error('状態数が上限を超えました');frames.push({title,note,columns,rows:rows.length?structuredClone(rows):[['—']],...structuredClone(extra)});}
 function range(l,r,n){integer(l,0,n);integer(r,l,n);}
 function ops(){return array(input.ops).map(op=>{array(op,8);if(typeof op[0]!=='string')throw Error('操作名が必要です');return op;});}
 const initial=(rows,cols=['位置','値'],extra={})=>emit('初期状態',specs[mode][2],cols,rows,extra);
 if(mode==='cyclic_sequence'){
  const a=values(input.a),s=integer(input.shift),start=integer(input.start),count=integer(input.count,0,40);if(!a.length&&count)throw Error('切出す列が空です');
  initial(a.map((v,i)=>[i,v]),undefined,{values:a});const norm=x=>((x%a.length)+a.length)%a.length,rotation=a.map((_,i)=>a[(i+norm(s))%a.length]);
  emit('左回転',`shift=${s} → ${a.length?norm(s):0}`,['新位置','元位置','値'],rotation.map((v,i)=>[i,(i+(a.length?norm(s):0))%a.length,v]),{values:rotation});
  const slice=[];for(let i=0;i<count;i++){slice.push(a[norm(start+i)]);emit('循環切出し',`元の列の位置${norm(start+i)}を追加`,['出力位置','値'],slice.map((v,j)=>[j,v]),{values:slice});}result={rotation,slice};
 }else if(mode==='interval_operations'){
  const a=array(input.intervals,16).map(p=>{array(p,2);const l=integer(p[0]),r=integer(p[1],l);return[l,r];}).sort((a,b)=>a[0]-b[0]||a[1]-b[1]),cuts=values(input.cuts).sort((a,b)=>a-b),merged=[];
  initial(a,['左端','右端']);for(const [l,r]of a){if(l===r)continue;if(!merged.length||merged.at(-1)[1]<l)merged.push([l,r]);else merged.at(-1)[1]=Math.max(merged.at(-1)[1],r);emit('併合',`[${l},${r})を追加`,['左端','右端'],merged);}
  const split=[];for(const [l,r]of merged){let prev=l;for(const c of [...new Set(cuts),r])if(prev<c&&c<=r){split.push([prev,c]);prev=c;emit('分割',`内部の点${c}まで`,['左端','右端'],split);}}result={merged,split};
 }else if(mode==='quotient_enumeration'){
  const n=integer(input.n,0,10000),a=[];initial([[n]],['n']);for(let l=1;l<=n;){const q=Math.floor(n/l),r=Math.floor(n/q);a.push([l,r,q]);emit('同じ商の最大区間',`l=${l}, q=${q}, r=floor(n/q)=${r}`,['l（含む）','r（含む）','商'],a);l=r+1;}result=a;
 }else if(mode==='multiple_enumeration'){
  const l=integer(input.l,0),r=integer(input.r,l),step=integer(input.step,1);if(r-l>200)throw Error('図解の範囲長は200以下です');const a=[];initial([[l,r,step]],['l','r（含まない）','step']);let x=l+(step-l%step)%step;for(;x<r;x+=step){a.push(x);emit('倍数の追加',`次への加算前の残り距離=${r-x}`,['値'],a.map(x=>[x]),{values:a});}result=a;
 }else if(mode==='gray_code'){
  const bits=integer(input.bits,0,6),a=[];initial([[bits]],['bits']);for(let i=0;i<2**bits;i++){const g=i^(i>>1);a.push(g);emit('1bitの変化',`i=${i}、Gray=i XOR (i>>1)`,['通常','二進数','Gray','二進数'],a.map((v,j)=>[j,j.toString(2).padStart(bits,'0'),v,v.toString(2).padStart(bits,'0')]));}result=a;
 }else if(mode==='subset_pairs'){
  const n=integer(input.n,0,5),a=[];initial([[n]],['n']);for(let s=0;s<2**n;s++){for(let t=s;;t=(t-1)&s){a.push([s,t]);emit('部分集合の部分集合',`S=${s}、T=${t}`,['S','T'],a);if(!t)break;}}result=a;
 }else if(mode==='permutation_rank'){
  const a=values(input.a,10),n=a.length;let f=[1];for(let i=1;i<=n;i++)f.push(f.at(-1)*i);if([...a].sort((a,b)=>a-b).some((x,i)=>x!==i))throw Error('0..n-1の順列を指定してください');
  let candidates=Array.from({length:n},(_,i)=>i),rank=0;initial(a.map((v,i)=>[i,v]));for(let i=0;i<n;i++){const k=candidates.indexOf(a[i]);rank+=k*f[n-1-i];candidates.splice(k,1);emit('順位への寄与',`位置${i}の小さい未使用値は${k}個`,['位置','値','寄与','累積順位','残り'],[[i,a[i],k*f[n-1-i],rank,candidates.join(',')]]);}
  let r=integer(input.rank,0,f[n]-1);candidates=Array.from({length:n},(_,i)=>i);const p=[];for(let i=n-1;i>=0;i--){const k=Math.floor(r/f[i]);r%=f[i];p.push(candidates.splice(k,1)[0]);emit('順位から復元',`次の候補の${k}番目を選択`,['復元済み','残り順位','候補'],[[p.join(','),r,candidates.join(',')]],{values:p});}result={rank,permutation:p};
 }else if(mode.endsWith('disjoint_set_union')||mode==='merge_tree'){
  const n=integer(input.n,0,16),p=Array(n).fill(-1),parity=Array(n).fill(0),bad=Array(n).fill(false),next=Array.from({length:n+1},(_,i)=>i),versions=[p.slice()],top=Array.from({length:n},(_,i)=>i),history=Array(n).fill(-1),children=Array.from({length:n},()=>[-1,-1]);let weight=input.values?values(input.values):Array(n).fill(0),time=0;const outputs=[];if(weight.length!==n)throw Error('valuesの長さが違います');
  function find(a,v){integer(v,0,n-1);while(a[v]>=0)v=a[v];return v;}
  function join(a,u,v){let x=find(a,u),y=find(a,v);if(x===y)return false;if(a[x]>a[y])[x,y]=[y,x];a[x]+=a[y];a[y]=x;return[x,y];}
  function show(label,extra=''){const rows=p.map((x,i)=>[i,x,parity[i],weight[i],bad[i]?'奇閉路':'可',next[i]]);emit(label,extra,['頂点','親/負サイズ','親との色差','成分和','二部性','next'],rows,{edges:p.flatMap((x,i)=>x>=0?[[i,x]]:[])});}
  show('初期状態');for(const op of ops()){
   if(mode==='persistent_disjoint_set_union'){
    const base=integer(op[1],0,versions.length-1),a=versions[base].slice();if(op[0]==='merge'){join(a,op[2],op[3]);versions.push(a);emit('版を分岐',`版${base}から版${versions.length-1}`,['版','親/負サイズ'],versions.map((v,i)=>[i,v.join(',')]));}else if(op[0]==='same')outputs.push(find(a,op[2])===find(a,op[3]));else if(op[0]==='size')outputs.push(-a[find(a,op[2])]);else throw Error('操作名が不正です');if(op[0]!=='merge')emit('版の照会',`版${base}、答え=${outputs.at(-1)}`,['版','親/負サイズ'],[[base,a.join(',')]]);
   }else if(mode==='partially_persistent_disjoint_set_union'){
    if(op[0]==='merge'){join(p,op[1],op[2]);versions.push(p.slice());time++;show('最新時刻の併合',`時刻${time}`);}else{const t=integer(op[1],0,time),a=versions[t];if(op[0]==='same')outputs.push(find(a,op[2])===find(a,op[3]));else if(op[0]==='size')outputs.push(-a[find(a,op[2])]);else throw Error('操作名が不正です');emit('過去時刻の照会',`時刻${t}、答え=${outputs.at(-1)}`,['時刻','親/負サイズ'],versions.map((v,i)=>[i,v.join(',')]));}
   }else if(mode==='interval_disjoint_set_union'){
    if(op[0]!=='merge')throw Error('mergeを指定してください');const l=op[1],r=op[2];range(l,r,n);let i=l;while(i+1<r){while(next[i]!==i)i=next[i];if(i+1>=r)break;join(p,i,i+1);next[i]=i+1;show('隣接境界を連結',`境界${i}|${i+1}を以後スキップ`);i=next[i];}
   }else if(mode==='bipartite_disjoint_set_union'){
    if(op[0]!=='edge')throw Error('edgeを指定してください');let u=integer(op[1],0,n-1),v=integer(op[2],0,n-1),x=0,y=0;while(p[u]>=0){x^=parity[u];u=p[u];}while(p[v]>=0){y^=parity[v];v=p[v];}if(u===v){if(x===y)bad[u]=true;}else{if(p[u]>p[v])[u,v]=[v,u];p[u]+=p[v];p[v]=u;parity[v]=x^y^1;bad[u]=bad[u]||bad[v];}outputs.push(!p.some((v,i)=>v<0&&bad[i]));show('辺を追加',`全体の二部性=${outputs.at(-1)}`);
   }else{
    if(op[0]==='merge'){let u=find(p,op[1]),v=find(p,op[2]);if(u!==v){const a=top[u],b=top[v],pair=join(p,u,v);weight[pair[0]]+=weight[pair[1]];if(mode==='merge_tree'){const k=history.length;history[a]=history[b]=k;history.push(-1);children.push([a,b]);top[pair[0]]=k;emit('履歴頂点を作成',`新しい根${k}`,['頂点','親','左子','右子'],history.map((v,i)=>[i,v,...children[i]]),{edges:history.flatMap((v,i)=>v>=0?[[i,v]]:[])});}}show('成分を併合');}
    else if(op[0]==='add'){const v=find(p,op[1]);weight[v]+=integer(op[2]);show('寄与を追加');}else if(op[0]==='prod'){outputs.push(weight[find(p,op[1])]);show('成分和を照会',String(outputs.at(-1)));}else throw Error('操作名が不正です');
   }
  }result=mode==='persistent_disjoint_set_union'||mode==='partially_persistent_disjoint_set_union'?{versions,answers:outputs}:mode==='merge_tree'?{parent:history,children}:{parent:p,answers:outputs};
 }else if(['dynamic_segtree','dynamic_lazy_segtree','range_actions'].includes(mode)){
  const n=integer(input.n,0,64),nodes=[],answers=[];function make(l,r){const k=nodes.length;nodes.push({l,r,sum:0,a:1,b:0,left:-1,right:-1});return k;}if(n)make(0,n);
  function show(title,note=''){emit(title,note,['ID','区間','和','遅延a','遅延b','左','右'],nodes.map((v,i)=>[i,`[${v.l},${v.r})`,v.sum,v.a,v.b,v.left,v.right]),{edges:nodes.flatMap((v,i)=>[v.left,v.right].filter(x=>x>=0).map(x=>[i,x]))});}
  function apply(k,a,b){const v=nodes[k];v.sum=a*v.sum+b*(v.r-v.l);v.a=a*v.a;v.b=a*v.b+b;}
  function push(k){const v=nodes[k];if(v.left<0){let m=Math.floor((v.l+v.r)/2);v.left=make(v.l,m);v.right=make(m,v.r);}if(v.a!==1||v.b!==0){apply(v.left,v.a,v.b);apply(v.right,v.a,v.b);v.a=1;v.b=0;show('遅延を子へ伝播',`ノード${k}`);}}
  function update(k,l,r,a,b){const v=nodes[k];if(r<=v.l||v.r<=l)return;if(l<=v.l&&v.r<=r){apply(k,a,b);show('被覆区間へ作用',`ノード${k}: x → ${a}x + ${b}`);return;}push(k);update(v.left,l,r,a,b);update(v.right,l,r,a,b);v.sum=nodes[v.left].sum+nodes[v.right].sum;}
  function sum(k,l,r){const v=nodes[k];if(r<=v.l||v.r<=l)return 0;if(l<=v.l&&v.r<=r)return v.sum;push(k);return sum(v.left,l,r)+sum(v.right,l,r);}
  show('初期状態');for(const op of ops()){if(op[0]==='set'){integer(op[1],0,n-1);integer(op[2]);update(0,op[1],op[1]+1,0,op[2]);}else{range(op[1],op[2],n);if(op[0]==='sum'){answers.push(op[1]===op[2]?0:sum(0,op[1],op[2]));show('区間和',String(answers.at(-1)));}else{let a=1,b;if(op[0]==='affine'){a=integer(op[3],-3,3);b=integer(op[4],-100,100);}else if(op[0]==='add')b=integer(op[3]);else if(op[0]==='assign'){a=0;b=integer(op[3]);}else throw Error('操作名が不正です');if(op[1]<op[2])update(0,op[1],op[2],a,b);}}show('操作完了');}result={sum:n?nodes[0].sum:0,answers};
 }else if(['fenwick_tree_2d','segtree_2d','compressed_fenwick_tree_2d'].includes(mode)){
  const h=integer(input.h,0,8),w=integer(input.w,0,8),grid=Array.from({length:h},()=>Array(w).fill(0)),answers=[];let bit=Array.from({length:h+1},()=>Array(w+1).fill(0));const points=array(input.points,32).map(p=>{range(p[0],p[0],h-1);range(p[1],p[1],w-1);return p;}),xs=[...new Set(points.map(p=>p[0]))].sort((a,b)=>a-b),ys=Array.from({length:xs.length+1},()=>[]);for(const [x,y]of points)for(let i=xs.indexOf(x)+1;i<=xs.length;i+=i&-i)ys[i].push(y);for(let i=1;i<ys.length;i++)ys[i]=[...new Set(ys[i])].sort((a,b)=>a-b);const cbits=ys.map(y=>Array(y.length+1).fill(0));
  let hb=1,wb=1;while(hb<h)hb*=2;while(wb<w)wb*=2;const seg=Array.from({length:2*hb},()=>Array(2*wb).fill(0));
  function show(title,note=''){let rows,cols;if(mode==='compressed_fenwick_tree_2d'){cols=['x BITノード','登録y','y BIT'];rows=ys.slice(1).map((y,i)=>[i+1,y.join(','),cbits[i+1].slice(1).join(',')]);}else if(mode==='segtree_2d'){cols=['x木ノード','y木の集約'];rows=seg.slice(1).map((v,i)=>[i+1,v.slice(1).join(',')]);}else{cols=['x BITノード','y方向のBIT'];rows=bit.slice(1).map((v,i)=>[i+1,v.slice(1).join(',')]);}emit(title,note,cols,rows,{grid});}
  const lower=(a,x)=>a.filter(v=>v<x).length;function prefix(x,y){let z=0;if(mode==='compressed_fenwick_tree_2d'){for(let i=lower(xs,x);i>0;i-=i&-i)for(let j=lower(ys[i],y);j>0;j-=j&-j)z+=cbits[i][j];}else for(let i=x;i>0;i-=i&-i)for(let j=y;j>0;j-=j&-j)z+=bit[i][j];return z;}
  function rowprod(x,l,r){let z=0;for(l+=wb,r+=wb;l<r;l>>=1,r>>=1){if(l&1)z+=seg[x][l++];if(r&1)z+=seg[x][--r];}return z;}
  show('初期状態');for(const op of ops()){if(op[0]==='add'){const x=integer(op[1],0,h-1),y=integer(op[2],0,w-1),d=integer(op[3]);grid[x][y]+=d;if(mode==='compressed_fenwick_tree_2d'){if(!points.some(p=>p[0]===x&&p[1]===y))throw Error('更新点が未登録です');for(let i=xs.indexOf(x)+1;i<=xs.length;i+=i&-i)for(let j=ys[i].indexOf(y)+1;j<cbits[i].length;j+=j&-j){cbits[i][j]+=d;show('圧縮BITセルを更新',`(${i},${j}) += ${d}`);}}
   else if(mode==='segtree_2d'){let i=x+hb,j=y+wb;seg[i][j]=grid[x][y];for(let k=j>>1;k;k>>=1)seg[i][k]=seg[i][k*2]+seg[i][k*2+1];for(i>>=1;i;i>>=1){seg[i][j]=seg[i*2][j]+seg[i*2+1][j];for(let k=j>>1;k;k>>=1)seg[i][k]=seg[i][k*2]+seg[i][k*2+1];show('二方向の祖先を再計算',`xノード${i}`);}}
   else for(let i=x+1;i<=h;i+=i&-i)for(let j=y+1;j<=w;j+=j&-j){bit[i][j]+=d;show('BITセルを更新',`(${i},${j}) += ${d}`);}}
   else if(op[0]==='sum'){const[l,d,r,u]=op.slice(1);range(l,r,h);range(d,u,w);let z=0;if(mode==='segtree_2d'){for(let a=l+hb,b=r+hb;a<b;a>>=1,b>>=1){if(a&1)z+=rowprod(a++,d,u);if(b&1)z+=rowprod(--b,d,u);}}else z=prefix(r,u)-prefix(l,u)-prefix(r,d)+prefix(l,d);answers.push(z);show('長方形和',`[${l},${r})×[${d},${u}) = ${z}`);}else throw Error('操作名が不正です');}result={grid,answers};
 }else if(mode==='disjoint_sparse_table'){
  const a=values(input.a),n=a.length,table=[],answers=[];initial(a.map((v,i)=>[i,v]),undefined,{values:a});for(let half=1;half<n;half*=2){let row=a.slice();for(let l=0;l<n;l+=half*2){const mid=Math.min(l+half,n),r=Math.min(l+2*half,n);for(let i=mid-2;i>=l;i--)row[i]=a[i]+row[i+1];for(let i=mid+1;i<r;i++)row[i]=row[i-1]+a[i];}table.push(row);emit('suffix/prefixの構築',`半幅${half}`,['レベル','各位置の部分和'],table.map((r,i)=>[i,r.join(',')]));}for(const[l,r]of array(input.queries)){range(l,r,n);let z=0,b=-1;if(r===l+1)z=a[l];else if(l<r){b=Math.floor(Math.log2(l^(r-1)));z=table[b][l]+table[b][r-1];}answers.push(z);emit('区間積',`[${l},${r})、選ぶレベル${b}、答え${z}`,['l','r','レベル','答え'],[[l,r,b,z]],{values:a,active:[l,r]});}result=answers;
 }else if(mode==='binary_trie'){
  const bits=integer(input.bits,1,16),nodes=[{child:[-1,-1],count:0,depth:0,prefix:0}],answers=[];let mask=0;function show(title,note=''){emit(title,note,['ID','深さ','生のprefix','個数','0の子','1の子'],nodes.map((n,i)=>[i,n.depth,n.prefix.toString(2),n.count,...n.child]),{edges:nodes.flatMap((n,i)=>n.child.filter(v=>v>=0).map(v=>[i,v]))});}function count(x){let k=0;for(let b=bits-1;b>=0;b--){k=nodes[k].child[(x>>b)&1];if(k<0)return 0;}return nodes[k].count;}
  show('初期状態');for(const op of ops()){if(op[0]==='xor'){mask^=integer(op[1],0,2**bits-1);show('XORマスク変更',`mask=${mask}`);}else if(op[0]==='insert'||op[0]==='erase'){const x=integer(op[1],0,2**bits-1)^mask,d=op[0]==='insert'?1:-1;if(d<0&&!count(x)){show('削除なし');continue;}let k=0;nodes[0].count+=d;for(let b=bits-1;b>=0;b--){const c=(x>>b)&1;if(nodes[k].child[c]<0){nodes[k].child[c]=nodes.length;nodes.push({child:[-1,-1],count:0,depth:bits-b,prefix:x>>b});}k=nodes[k].child[c];nodes[k].count+=d;show('経路上の個数を更新',`深さ${bits-b}、mask=${mask}`);}}
   else if(op[0]==='kth'||op[0]==='xor_min'){let rank=op[0]==='kth'?integer(op[1],0):0,x=op[0]==='xor_min'?integer(op[1],0,2**bits-1):0;if(rank>=nodes[0].count){answers.push(null);continue;}let k=0,raw=0;for(let b=bits-1;b>=0;b--){let c=((x^mask)>>b)&1,j=nodes[k].child[c],num=j<0?0:nodes[j].count;if(rank>=num){rank-=num;c^=1;}raw|=c<<b;k=nodes[k].child[c];show('個数で分岐',`bit=${b}、選んだ枝${c}、残り順位${rank}`);}answers.push(raw^mask);}else throw Error('操作名が不正です');}result={mask,size:nodes[0].count,answers};
 }else if(mode==='fast_set'){
  const n=integer(input.n,0,10000),levels=[];for(let size=Math.max(1,Math.ceil(n/64));;size=Math.ceil(size/64)){levels.push(Array(size).fill(0n));if(size===1)break;}const answers=[];let count=0;
  const contains=x=>Boolean(levels[0][x>>6]&(1n<<BigInt(x%64)));const lo=x=>{let i=0;while(!(x&1n)){i++;x>>=1n;}return i;},hi=x=>{let i=-1;while(x){i++;x>>=1n;}return i;};function show(t,note=''){emit(t,note,['階層','ワード','bit列（16進）'],levels.flatMap((a,h)=>a.map((x,i)=>[h,i,x.toString(16).padStart(16,'0')])));}
  show('初期状態');for(const op of ops()){let x=integer(op[1],-1,n);if(op[0]==='insert'||op[0]==='erase'){integer(x,0,n-1);const insert=op[0]==='insert';if(contains(x)===insert)continue;count+=insert?1:-1;for(let h=0;h<levels.length;h++){const i=x>>6,old=levels[h][i],bit=1n<<BigInt(x%64);levels[h][i]=insert?old|bit:old&~bit;show('非空ワードの伝播',`階層${h}、ワード${i}`);if(insert?old!==0n:levels[h][i]!==0n)break;x=i;}}
   else if(op[0]==='next'){x=Math.max(x,0);let answer=n;if(x<n)for(let h=0;h<levels.length;h++){if((x>>6)>=levels[h].length)break;const word=levels[h][x>>6]>>BigInt(x%64);show('次を探す',`階層${h}、候補word=${word.toString(16)}`);if(!word){x=(x>>6)+1;continue;}x+=lo(word);for(let k=h-1;k>=0;k--)x=x*64+lo(levels[k][x]);answer=x<n?x:n;break;}answers.push(answer);}
   else if(op[0]==='prev'){x=Math.min(x,n-1);let answer=-1;for(let h=0;h<levels.length&&x>=0;h++){const word=levels[h][x>>6]&((1n<<BigInt(x%64+1))-1n);show('前を探す',`階層${h}、候補word=${word.toString(16)}`);if(!word){x=(x>>6)-1;continue;}x=(x>>6)*64+hi(word);for(let k=h-1;k>=0;k--)x=x*64+hi(levels[k][x]);answer=x;break;}answers.push(answer);}else throw Error('操作名が不正です');}result={size:count,answers};
 }else if(['treap','rbst','ordered_set'].includes(mode)){
  let seed=integer(input.seed,0,2**32-1),root=null,id=0;const answers=[];const random=()=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed;},size=n=>n?n.size:0,distinct=n=>n?n.distinct:0;function pull(n){n.size=n.count+size(n.left)+size(n.right);n.distinct=1+distinct(n.left)+distinct(n.right);return n;}
  function split(n,x){if(!n)return[null,null];if(n.key<x){const[a,b]=split(n.right,x);n.right=a;return[pull(n),b];}const[a,b]=split(n.left,x);n.left=b;return[a,pull(n)];}
  function merge(a,b){if(!a||!b)return a||b;const left=mode==='rbst'?random()%(distinct(a)+distinct(b))<distinct(a):a.priority>b.priority;if(left){a.right=merge(a.right,b);return pull(a);}b.left=merge(a,b.left);return pull(b);}
  function lookup(x){let n=root;while(n&&n.key!==x)n=x<n.key?n.left:n.right;return n;}
  function insert(n,p){if(!n)return p;const top=mode==='rbst'?random()%(distinct(n)+1)===0:p.priority>n.priority;if(top){[p.left,p.right]=split(n,p.key);return pull(p);}if(p.key<n.key)n.left=insert(n.left,p);else n.right=insert(n.right,p);return pull(n);}
  function change(n,x,d){if(n.key===x){if(d<0&&n.count===1)return merge(n.left,n.right);n.count+=d;}else if(x<n.key)n.left=change(n.left,x,d);else n.right=change(n.right,x,d);return pull(n);}
  function show(title,note=''){const rows=[],edges=[];function walk(n){if(!n)return;rows.push([n.id,n.key,n.count,n.size,n.priority,n.left?.id??'-',n.right?.id??'-']);for(const c of[n.left,n.right])if(c){edges.push([n.id,c.id]);walk(c);}}walk(root);emit(title,note,['ID','キー','個数','部分木サイズ','優先度','左','右'],rows,{edges});}
  show('初期状態');for(const op of ops()){if(op[0]==='insert'){const x=integer(op[1]),old=lookup(x);if(old){if(mode!=='ordered_set')root=change(root,x,1);}else root=insert(root,{id:id++,key:x,count:1,size:1,distinct:1,priority:random(),left:null,right:null});show('挿入');}else if(op[0]==='erase'){const x=integer(op[1]);if(lookup(x))root=change(root,x,-1);show('一個削除');}else if(op[0]==='rank'){const x=integer(op[1]);let n=root,z=0;while(n){show('順位探索',`キー${n.key}、累積${z}`);if(n.key<x){z+=size(n.left)+n.count;n=n.right;}else n=n.left;}answers.push(z);}else if(op[0]==='kth'){let k=integer(op[1],0),n=root,value=null;if(k<size(root))while(n){show('k番目の探索',`残り順位${k}、左の個数${size(n.left)}`);if(k<size(n.left))n=n.left;else if(k<size(n.left)+n.count){value=n.key;break;}else{k-=size(n.left)+n.count;n=n.right;}}answers.push(value);}else throw Error('操作名が不正です');}const sorted=[];function walk(n){if(n){walk(n.left);for(let i=0;i<n.count;i++)sorted.push(n.key);walk(n.right);}}walk(root);result={values:sorted,answers};
 }else if(mode==='lazy_reversible_treap'){
  let a=values(input.a);const answers=[];function show(t,note='',active){emit(t,note,['位置','値'],a.map((v,i)=>[i,v]),{values:a,active});}show('初期状態');for(const op of ops()){if(op[0]==='insert'){integer(op[1],0,a.length);integer(op[2]);a.splice(op[1],0,op[2]);if(a.length>32)throw Error('列は32要素以内です');show('挿入');}else if(op[0]==='erase'){integer(op[1],0,a.length-1);a.splice(op[1],1);show('削除');}else{const l=op[1],r=op[2];range(l,r,a.length);show('対象区間の切出し',`[${l},${r})`,[l,r]);if(op[0]==='reverse')a.splice(l,r-l,...a.slice(l,r).reverse());else if(op[0]==='affine'){integer(op[3],-3,3);integer(op[4],-100,100);for(let i=l;i<r;i++)a[i]=op[3]*a[i]+op[4];}else if(op[0]==='sum')answers.push(a.slice(l,r).reduce((a,b)=>a+b,0));else throw Error('操作名が不正です');show('区間の結合','前後の向きと作用結果を反映',[l,r]);}}result={values:a,answers};
 }else if(mode==='persistent_stack'||mode==='persistent_queue'){
  const versions=[[]],nodes=[{value:'空',previous:0,depth:0,up:[]}],tails=[0],answers=[];function show(t,note=''){emit(t,note,['版','論理列','末尾/先頭ノード'],versions.map((v,i)=>[i,v.join(','),tails[i]]),{edges:nodes.flatMap((v,i)=>i?[[i,v.previous]]:[])});emit('共有ノードと祖先',note,['ノード','値','前のノード','祖先表'],nodes.map((v,i)=>[i,v.value,v.previous,v.up.join(',')]));}
  show('初期状態');for(const op of ops()){const b=integer(op[1],0,versions.length-1),a=versions[b].slice();if(op[0]==='push'){const x=integer(op[2]),p=a.length?tails[b]:0,up=[p];for(let k=1;up[k-1]&&nodes[up[k-1]].up.length>=k;k++)up.push(nodes[up[k-1]].up[k-1]);nodes.push({value:x,previous:p,depth:nodes[p].depth+1,up});a.push(x);versions.push(a);tails.push(nodes.length-1);show('共有して追加',`版${b}から分岐`);}else if(op[0]==='pop'){if(!a.length)throw Error('空版のpopです');const t=mode==='persistent_queue'?tails[b]:nodes[tails[b]].previous;if(mode==='persistent_queue')a.shift();else a.pop();versions.push(a);tails.push(t);show('版を取り出す',`版${b}から分岐`);}else if(op[0]==='front'){if(!a.length)throw Error('空版です');let node=tails[b];if(mode==='persistent_queue'){let distance=a.length-1;for(let bit=0;distance;bit++,distance>>=1)if(distance&1){const before=node;node=nodes[node].up[bit];emit('祖先をジャンプ',`ノード${before}から2^${bit}個前の${node}へ`,['ノード','値','前のノード','祖先表'],nodes.map((v,i)=>[i,v.value,v.previous,v.up.join(',')]));}}answers.push(nodes[node].value);show('先頭を照会',`答え=${answers.at(-1)}`);}else throw Error('操作名が不正です');}result={versions,answers};
 }else if(mode==='range_set'){
  let ranges=[];const answers=[];initial([],['左端','右端']);
  for(const op of ops()){
   const l=integer(op[1]);
   if(op[0]==='contains')answers.push(ranges.some(([a,b])=>a<=l&&l<b));
   else {
    const r=integer(op[2],l);
    if(op[0]==='insert'){
     if(l<r){
      const sorted=[...ranges,[l,r]].sort((a,b)=>a[0]-b[0]);ranges=[];
      for(const p of sorted){if(!ranges.length||ranges.at(-1)[1]<p[0])ranges.push(p.slice());else ranges.at(-1)[1]=Math.max(ranges.at(-1)[1],p[1]);}
     }
    }else if(op[0]==='erase'){
     if(l<r)ranges=ranges.flatMap(([a,b])=>b<=l||r<=a?[[a,b]]:[...(a<l?[[a,l]]:[]),...(r<b?[[r,b]]:[])]);
    }else if(op[0]==='covers')answers.push(l===r||ranges.some(([a,b])=>a<=l&&r<=b));
    else throw Error('操作名が不正です');
   }
   emit('区間集合の更新/照会',op.join(' '),['左端（含む）','右端（含まない）'],ranges,{intervals:ranges});
  }
  result={intervals:ranges,answers};
 }else if(mode==='offline_rmq'){
  const a=values(input.a),n=a.length,p=Array.from({length:n},(_,i)=>i),stack=[],queries=array(input.queries),answers=Array(queries.length).fill(-1);queries.forEach(([l,r])=>range(l,r,n));initial(a.map((v,i)=>[i,v]),undefined,{values:a});function find(x){let r=x;while(p[r]!==r)r=p[r];while(p[x]!==x){let y=p[x];p[x]=r;x=y;}return r;}
  for(let i=0;i<n;i++){while(stack.length&&a[i]<a[stack.at(-1)])p[stack.pop()]=i;stack.push(i);emit('次の小さい位置へリンク',`右端${i+1}、スタック=[${stack}]`,['位置','値','親リンク'],a.map((v,j)=>[j,v,p[j]]),{values:a,active:[0,i+1]});queries.forEach(([l,r],id)=>{if(l<r&&r===i+1){answers[id]=find(l);emit('右端が一致する問い',`[${l},${r}) → 最左argmin=${answers[id]}`,['ID','l','r','argmin'],queries.map((q,k)=>[k,...q,answers[k]]));}});}result=answers;
 }else if(mode==='rollback_mo'){
  const a=values(input.a),n=a.length,queries=array(input.queries),b=integer(input.block,1,64),answers=Array(queries.length).fill(0);queries.forEach(([l,r])=>range(l,r,n));let active=[];function show(t,note=''){emit(t,note,['位置','値','集計に含む'],a.map((v,i)=>[i,v,active.includes(i)?'●':'']),{values:a});}initial(a.map((v,i)=>[i,v]),undefined,{values:a});for(let block=0;block<=Math.floor(n/b);block++){const ids=queries.map((q,i)=>i).filter(i=>Math.floor(queries[i][0]/b)===block),base=Math.min(n,(block+1)*b),long=[];for(const id of ids){const[l,r]=queries[id];if(r>base){long.push(id);continue;}active=[];show('短い問いを空から開始');for(let i=l;i<r;i++){active.push(i);show('右へ追加');}answers[id]=active.reduce((s,i)=>s+a[i],0);show('短い問いの答え',`ID${id}=${answers[id]}`);}active=[];let right=base;long.sort((i,j)=>queries[i][1]-queries[j][1]);for(const id of long){const[l,r]=queries[id];while(right<r){active.push(right++);show('永続部分を右に伸ばす');}const token=active.length;show('snapshot',`履歴位置${token}`);for(let i=base;i>l;){active.push(--i);show('左を一時追加');}answers[id]=active.reduce((s,i)=>s+a[i],0);active.length=token;show('rollback',`ID${id}=${answers[id]}、履歴位置${token}へ戻す`);}}result=answers;
 }else if(mode==='mo_on_tree'){
  const n=integer(input.n,0,16),edges=array(input.edges,16),g=Array.from({length:n},()=>[]);for(const[u,v]of edges){integer(u,0,n-1);integer(v,0,n-1);g[u].push(v);g[v].push(u);}const parent=Array(n).fill(-1),depth=Array(n).fill(0),tin=Array(n),tout=Array(n),component=Array(n).fill(-1),tour=[];let comp=0;
  function dfs(v,p){component[v]=comp;parent[v]=p;tin[v]=tour.length;tour.push(v);for(const u of g[v])if(u!==p){if(component[u]>=0)throw Error('木/森を指定してください');depth[u]=depth[v]+1;dfs(u,v);}tout[v]=tour.length;tour.push(v);}for(let v=0;v<n;v++)if(component[v]<0){dfs(v,-1);comp++;}if(edges.length!==n-comp)throw Error('木/森を指定してください');
  const queries=array(input.queries),mapped=queries.map(([u,v],id)=>{integer(u,0,n-1);integer(v,0,n-1);if(component[u]!==component[v])throw Error('異なる成分です');let a=u,b=v;while(depth[a]>depth[b])a=parent[a];while(depth[b]>depth[a])b=parent[b];while(a!==b){a=parent[a];b=parent[b];}const w=a;if(tin[u]>tin[v])[u,v]=[v,u];return{id,l:w===u?tin[u]:tout[u],r:tin[v]+1,extra:w===u?-1:w};}),active=Array(n).fill(false),answers=Array(queries.length);let l=0,r=0;const width=Math.max(1,Math.floor(Math.sqrt(Math.max(1,tour.length))));mapped.sort((a,b)=>Math.floor(a.l/width)-Math.floor(b.l/width)||((Math.floor(a.l/width)&1)?b.r-a.r:a.r-b.r));
  initial(parent.map((p,v)=>[v,p,tin[v],tout[v]]),['頂点','親','入る位置','出る位置'],{edges});function show(t,note=''){emit(t,note,['Euler位置','頂点','有効'],tour.map((v,i)=>[i,v,active[v]?'●':'']),{values:tour,active:[l,r]});}function toggle(v){active[v]=!active[v];show('出現回数の偶奇を反転',`頂点${v} → ${active[v]?'追加':'削除'}`);}for(const q of mapped){while(l>q.l)toggle(tour[--l]);while(r<q.r)toggle(tour[r++]);while(l<q.l)toggle(tour[l++]);while(r>q.r)toggle(tour[--r]);if(q.extra>=0){toggle(q.extra);show('LCAを一時追加',`頂点${q.extra}`);}answers[q.id]=active.flatMap((v,i)=>v?[i]:[]);show('パスの頂点集合',`ID${q.id}=[${answers[q.id]}]`);if(q.extra>=0)toggle(q.extra);}result={tour,answers};
 }else throw Error('未対応のモードです');
 if(!frames.length)initial([]);emit('完了',JSON.stringify(result),['結果'],[[JSON.stringify(result)]]);return{frames,result};
}
const api={specs,paths,build};if(typeof module!=='undefined')module.exports=api;root.APriorityModels=api;
})(typeof window==='undefined'?globalThis:window);
