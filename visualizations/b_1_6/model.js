(function(root){
'use strict';
const metadata=typeof module!=='undefined'?require('./specs.js'):root.BMetadata;
const copy=x=>JSON.parse(JSON.stringify(x));
function integer(x,lo=-1000000,hi=1000000){if(!Number.isSafeInteger(x)||x<lo||x>hi)throw Error(`整数 ${lo}..${hi} を指定してください`);return x;}
function list(a,max=64){if(!Array.isArray(a)||a.length>max)throw Error(`配列は${max}要素以下です`);return a;}
function numbers(a,max=64){return list(a,max).map(x=>integer(x));}
function operations(input){return list(input.ops??[],100);}
function bounds(l,r,n){integer(l,0,n);integer(r,l,n);}
const sum=a=>a.reduce((s,x)=>s+x,0),cmp=(a,b)=>a-b;
const bitLength=x=>x?Math.floor(Math.log2(x))+1:0;
function gcd(a,b){while(b)[a,b]=[b,a%b];return a;}
function big(x){let n;try{n=BigInt(x);}catch{throw Error('非負整数を指定してください');}if(n<0n||n>18446744073709551615n)throw Error('uint64範囲を指定してください');return n;}
const paths=Object.fromEntries(Object.entries(metadata).map(([key,v])=>[key,v.path]));
const samples={
 fractional_cascading:{lists:[[1,4,8],[2,4,7],[],[1,3,9]],x:4},integer_kth_root:{n:'18446744073709551615',k:3},stern_brocot_tree:{p:12,q:8},fraction_binary_search:{limit:20,p:7,q:5},sqrt_decomposition:{values:[3,1,4,1,5,9,2,6],ops:[['sum',1,7],['set',3,8],['sum',0,5]]},nonmonotone_search:{costs:[8,2,7,-3,6,1,4,-3],slack:2},range_set:{ops:[['insert',1,6],['insert',6,9],['erase',3,7],['insert',4,5]]},
 offline_dynamic_connectivity:{n:5,ops:[['add',0,1],['query',0,3],['add',1,3],['query',0,3],['remove',0,1],['query',0,3]]},parallel_disjoint_set_union:{n:8,ops:[['range',0,4,3],['range',1,5,2],['merge',2,3],['same',3,6]]},dynamic_disjoint_set_union:{ops:[['merge','alice','bob'],['merge','carol','dave'],['same','alice','carol'],['merge','bob','dave']]},additive_disjoint_set_union:{values:[2,1,4,3],ops:[['component',0,5],['merge',0,1],['component',1,2],['vertex',0,-3],['merge',2,3],['merge',1,2]]},incremental_bridge_connectivity:{n:5,ops:[['add',0,1],['add',1,2],['add',2,0],['add',2,3],['add',3,4],['add',2,3]]},
 sparse_dynamic_segtree:{n:32,ops:[['set',3,5],['set',26,7],['sum',0,16],['sum',0,32]]},rollback_segtree:{values:[1,2,3,4],ops:[['snapshot'],['set',1,8],['sum',0,3],['rollback',0],['sum',0,4]]},persistent_lazy_segtree:{values:[1,2,3,4],ops:[['apply',0,0,3,2,1],['apply',0,1,4,0,5],['sum',1,0,4],['sum',2,0,4]]},sortable_segtree:{keys:[4,1,3,0,2],values:[10,20,30,40,50],ops:[['sort',0,5,0],['sum',0,3],['sort',1,5,1],['set',2,7,60],['sum',1,4]]},range_tree:{points:[[1,2],[3,4],[1,2],[5,1]],ops:[['add',0,7],['add',2,3],['add',1,5],['sum',0,0,4,5]]},segtree_fractional_cascading:{values:[4,1,7,2,4,6,3,8],query:[1,7,2,6]},dual_segtree_2d:{height:4,width:5,ops:[['add',0,3,1,5,4],['add',1,4,2,4,-1],['get',2,3],['get',3,0]]},
 weighted_wavelet_matrix:{keys:[3,1,4,1,5,2],weights:[10,20,-3,7,5,9],query:[0,6,1,4,3]},wavelet_tree:{values:[4,1,4,2,7,2],query:[1,6,2,5,2]},persistent_binary_trie:{bits:5,ops:[['insert',0,7],['insert',1,2],['insert',1,18],['kth',2,0,3],['erase',3,7],['kth',4,0,0]]},priority_sum:{k:3,ops:[['insert',8],['insert',1],['insert',5],['insert',2],['erase',1],['k',2]]},dynamic_bitset:{n:18,bits:[0,3,7,8,17],ops:[['left',1],['set',6,1],['xor',[2,6,10]],['right',3]]},succinct_bit_vector:{bits:'010110010100',queries:[['rank',8,1],['select',2,1],['select',4,0]]},linear_rmq:{values:[7,3,5,2,8,1,4,1,9,6,3,2,5,0,4,8],queries:[[2,10],[6,15],[4,4]]},sqrt_tree:{values:[1,2,3,4,5,6,7,8,9,10,11,12],queries:[[1,11],[4,8],[5,5]]},
 splay_tree:{ops:[['insert',5],['insert',2],['insert',8],['insert',1],['insert',3],['kth',2],['erase',2]]},red_black_tree:{ops:[['insert',5],['insert',2],['insert',8],['insert',1],['insert',3],['insert',7],['erase',2],['kth',3]]},weight_balanced_tree:{ops:[['insert',0],['insert',1],['insert',2],['insert',3],['insert',4],['insert',5],['erase',1],['kth',2]]},lazy_balanced_bst:{values:[1,2,3,4],ops:[['affine',0,3,2,1],['reverse',0,4],['insert',2,8],['sum',1,4]]},persistent_balanced_bst:{ops:[['insert',0,5],['insert',1,2],['insert',1,8],['erase',2,5],['kth',3,1]]},skew_heap:{ops:[['push',0,4],['push',0,1],['push',1,7],['push',1,3],['meld',0,1],['pop',0]]},leftist_heap:{ops:[['push',0,4],['push',0,1],['push',1,7],['push',1,3],['meld',0,1],['pop',0]]},radix_heap:{ops:[['push',8],['push',3],['push',12],['pop'],['push',6],['pop'],['pop']]},erasable_priority_queue:{ops:[['push',9],['push',4],['push',7],['erase',4],['pop'],['pop']]},
 rectangle_add_rectangle_sum:{ops:[['add',0,0,4,3,2],['add',2,1,5,4,-1],['sum',1,0,5,4]]},rectangle_add_point_get:{ops:[['add',0,0,4,3,2],['add',2,1,5,4,-1],['get',2,2],['get',4,3]]},kd_tree:{points:[[0,0],[4,3],[2,1],[1,5],[6,2]],ops:[['report',1,1,5,5],['nearest',3,2]]},hilbert_order_mo:{values:[1,2,1,3,2,4,1,5],queries:[[0,4],[2,7],[1,3],[4,8],[0,0]]},static_range_count_distinct:{values:[1,2,1,3,2,4,1,5],queries:[[0,4],[2,7],[1,3],[4,8],[0,0]]},dynamic_rectangle_affine_sum:{ops:[['point',1,1,3],['point',4,2,5],['affine',0,0,5,4,2,1],['point',2,2,7],['sum',0,0,5,4],['set',0,4],['sum',0,0,3,3]]}
};
const specs=Object.fromEntries(Object.entries(metadata).map(([key,v])=>[key,[v.title,samples[key],v.summary]]));
function build(mode,input){
 if(!Object.hasOwn(specs,mode))throw Error('未知のモードです');
 if(!input||typeof input!=='object'||Array.isArray(input))throw Error('JSON objectを指定してください');
 const frames=[];
 function frame(title,note,columns=['項目','状態'],rows=[],grid=[],graph=null,active=[]){frames.push(copy({title,note,columns,rows:rows.length?rows:[['状態','空']],grid,graph,active}));}
 let result;
 frame('初期状態',specs[mode][2]);
 if(mode==='integer_kth_root'){
  const n=big(input.n),k=integer(input.k,1,256);let l=0n,r=n;
  const fits=x=>{let p=1n;for(let i=0;i<k;i++){p*=x;if(p>n)return false;}return true;};
  if(k>=64&&n>0n)l=r=1n;
  while(l<r){const m=l+(r-l+1n)/2n,ok=fits(m);frame('整数二分探索',`${m}^${k} <= ${n} は ${ok}`,['lo','mid','hi','判定'],[[String(l),String(m),String(r),ok]]);if(ok)l=m;else r=m-1n;}
  result=String(l);
 }else if(mode==='stern_brocot_tree'){
  let p=big(input.p),q=big(input.q);if(!p||!q)throw Error('正の分子・分母が必要です');const g=gcd(p,q);p/=g;q/=g;const original=[String(p),String(q)],path=[];
  while(p!==q){let side,count;if(p<q){side='L';count=(q-1n)/p;q-=count*p;}else{side='R';count=(p-1n)/q;p-=count*q;}path.push([side,String(count)]);frame('Euclidのrun',`${side} を ${count} 回まとめる`,['方向','回数','残る分子','残る分母'],[[side,String(count),String(p),String(q)]]);}
  let a=0n,b=1n,c=1n,d=0n;
  for(const [side,text]of path){const k=BigInt(text);if(side==='L'){c+=k*a;d+=k*b;}else{a+=k*c;b+=k*d;}frame('隣接境界から復元','左右境界の行列式は1。現在値はmediant。',['左','現在','右'],[[`${a}/${b}`,`${a+c}/${b+d}`,`${c}/${d}`]]);}
  result={fraction:original,path};
 }else if(mode==='fraction_binary_search'){
  const limit=big(input.limit),p=big(input.p),q=big(input.q);if(!limit||!q)throw Error('limitと分母は正にしてください');let a=0n,b=1n,c=1n,d=0n;
  while(a+c<=limit&&b+d<=limit){const side=(a+c)*q<=p*(b+d);let x=side?a:c,y=side?b:d,u=side?c:a,v=side?d:b;let cap=limit;if(u)cap=cap<(limit-x)/u?cap:(limit-x)/u;if(v)cap=cap<(limit-y)/v?cap:(limit-y)/v;const same=k=>((x+k*u)*q<=p*(y+k*v))===side;let lo=1n,hi=1n;while(hi<cap&&same(hi)){lo=hi;hi=hi*2n<cap?hi*2n:cap;}if(same(hi))lo=hi;else while(hi-lo>1n){const m=(lo+hi)/2n;if(same(m))lo=m;else hi=m;}x+=lo*u;y+=lo*v;if(side){a=x;b=y;}else{c=x;d=y;}frame('同じ方向をまとめて移動',`${side?'true側':'false側'}へ${lo}回。分子・分母の上限を同時に検査。`,['最大true','最小false'],[[`${a}/${b}`,`${c}/${d}`]]);}
  result=[[String(a),String(b)],[String(c),String(d)]];
 }else if(mode==='fractional_cascading'){
  const lists=list(input.lists,20).map(a=>numbers(a,40));for(const a of lists)if(a.some((x,i)=>i&&x<a[i-1]))throw Error('各列は昇順にしてください');const x=integer(input.x),catalogs=lists.map(()=>[]),links=lists.map(()=>[]);
  for(let i=lists.length-1;i>=0;i--){const next=catalogs[i+1]??[];catalogs[i]=[...lists[i],...next.filter((_,j)=>j%2)].sort(cmp);links[i]=catalogs[i].map(v=>[lists[i].filter(x=>x<v).length,next.filter(x=>x<v).length]);links[i].push([lists[i].length,next.length]);frame('catalogを連鎖',`列${i}: 次の列の一つおきの値を取り込む`,['位置','値','元列prefix','次列prefix'],catalogs[i].map((v,j)=>[j,v,...links[i][j]]),catalogs);}
  let pos=catalogs.length?catalogs[0].filter(v=>v<x).length:0;result=[];
  for(let i=0;i<lists.length;i++){result.push(links[i][pos][0]);frame('検索を次の列へ伝播',`列${i}: lower_bound(${x})=${result.at(-1)}`,['catalog位置','元列位置'],[[pos,result.at(-1)]],catalogs, null,[[i,pos]]);pos=links[i][pos][1];if(i+1<lists.length&&pos>0&&catalogs[i+1][pos-1]>=x)pos--;}
 }else if(mode==='sqrt_decomposition'||mode==='lazy_balanced_bst'){
  const a=numbers(input.values,40),width=Math.max(1,Math.floor(Math.sqrt(a.length))),answers=[];
  const show=note=>frame('列と集約',note,['ブロック','区間','和'],Array.from({length:Math.ceil(a.length/width)},(_,i)=>[i,`[${i*width},${Math.min(a.length,(i+1)*width)})`,sum(a.slice(i*width,(i+1)*width))]),[a]);show('現在の論理列。Lazy BSTでは節点配置を省略します。');
  for(const op of operations(input)){const [type,l,r,x,y]=op;if(type==='set'){integer(l,0,a.length-1);a[l]=integer(r);}else if(type==='insert'&&mode==='lazy_balanced_bst'){integer(l,0,a.length);a.splice(l,0,integer(r));}else if(type==='sum'){bounds(l,r,a.length);answers.push(sum(a.slice(l,r)));}else if(type==='reverse'&&mode==='lazy_balanced_bst'){bounds(l,r,a.length);a.splice(l,r-l,...a.slice(l,r).reverse());}else if(type==='affine'&&mode==='lazy_balanced_bst'){bounds(l,r,a.length);integer(x,-5,5);integer(y,-30,30);for(let i=l;i<r;i++)a[i]=x*a[i]+y;}else throw Error('不明な列操作');show(JSON.stringify(op));}result={values:a,answers};
 }else if(mode==='nonmonotone_search'){
  const costs=numbers(input.costs,32),slack=integer(input.slack??0,0,100),queue=[],bound=(l,r)=>Math.min(...costs.slice(l,r))-slack;let best=null;
  if(costs.length)queue.push([0,costs.length,bound(0,costs.length)]);
  while(queue.length){queue.sort((a,b)=>a[2]-b[2]||a[0]-b[0]);const [l,r,lower]=queue.shift(),prune=best&&(best[1]<lower||(best[1]===lower&&l>=best[0]));if(!prune){if(r-l===1){if(!best||costs[l]<best[1]||(costs[l]===best[1]&&l<best[0]))best=[l,costs[l]];}else{const m=Math.floor((l+r)/2);queue.push([l,m,bound(l,m)],[m,r,bound(m,r)]);}}frame(prune?'下界で枝刈り':'候補区間を展開',`[${l},${r}), 下界=${lower}, 暫定解=${JSON.stringify(best)}。教材では既知配列の最小値−slackを下界に使用。`,['左','右','下界'],queue,[costs],null,Array.from({length:r-l},(_,i)=>[0,l+i]));}result=best;
 }else if(mode==='range_set'){
  let ranges=[];
  for(const [type,l,r]of operations(input)){integer(l);integer(r);if(l>r)throw Error('左端<=右端にしてください');if(l===r){frame('空区間は変更なし',`${type} [${l},${r})`,['左','右'],ranges);continue;}if(type==='insert'){ranges.push([l,r]);ranges=ranges.filter(([a,b])=>a<b).sort((a,b)=>a[0]-b[0]);const merged=[];for(const [a,b]of ranges){if(merged.length&&merged.at(-1)[1]>=a)merged.at(-1)[1]=Math.max(merged.at(-1)[1],b);else merged.push([a,b]);}ranges=merged;}else if(type==='erase')ranges=ranges.flatMap(([a,b])=>b<=l||r<=a?[[a,b]]:[[a,Math.min(b,l)],[Math.max(a,r),b]].filter(([x,y])=>x<y));else throw Error('insert/eraseを指定');frame('連続区間集合',`${type} [${l},${r})。接する区間は統合。`,['左（含む）','右（含まない）'],ranges,[ranges.map(([a,b])=>`[${a},${b})`)]);}result=ranges;
 }else if(['parallel_disjoint_set_union','dynamic_disjoint_set_union','additive_disjoint_set_union','incremental_bridge_connectivity','offline_dynamic_connectivity'].includes(mode)){
  result=dsuModel(mode,input,frame);
 }else if(['sparse_dynamic_segtree','rollback_segtree','persistent_lazy_segtree','sortable_segtree','dual_segtree_2d','segtree_fractional_cascading'].includes(mode)){
  result=rangeModel(mode,input,frame);
 }else if(['weighted_wavelet_matrix','wavelet_tree','persistent_binary_trie','priority_sum','dynamic_bitset','succinct_bit_vector','linear_rmq','sqrt_tree'].includes(mode)){
  result=staticModel(mode,input,frame);
 }else if(['splay_tree','red_black_tree','weight_balanced_tree','persistent_balanced_bst','skew_heap','leftist_heap','radix_heap','erasable_priority_queue'].includes(mode)){
  result=orderedModel(mode,input,frame);
 }else result=spatialModel(mode,input,frame);
 frame('完了','最終結果を左に表示します。入力を変えると全履歴を作り直します。',['結果'],[[JSON.stringify(result)]]);
 return {frames,result};
}

function dsu(n){const p=Array(n).fill(-1);function leader(v){while(p[v]>=0)v=p[v];return v;}function merge(a,b){a=leader(a);b=leader(b);if(a===b)return false;if(p[a]>p[b])[a,b]=[b,a];p[a]+=p[b];p[b]=a;return true;}return{p,leader,merge};}
function graphComponents(n,edges,skip=-1){const d=dsu(n);edges.forEach(([a,b],i)=>{if(i!==skip)d.merge(a,b);});return Array.from({length:n},(_,i)=>d.leader(i));}
function dsuModel(mode,input,frame){
 if(mode==='dynamic_disjoint_set_union'){
  const names=[],p=[],answers=[];const id=name=>{if(typeof name!=='string'||name.length>16)throw Error('labelは16文字以下');let i=names.indexOf(name);if(i<0){i=names.length;if(i>=25)throw Error('25頂点以下');names.push(name);p.push(-1);}return i;};const root=i=>p[i]<0?i:root(p[i]);
  for(const [type,a,b]of operations(input)){let u=root(id(a)),v=root(id(b));if(type==='merge'){if(u!==v){if(p[u]>p[v])[u,v]=[v,u];p[u]+=p[v];p[v]=u;}}else if(type==='same')answers.push(u===v);else throw Error('merge/sameを指定');frame('ラベル辞書とDSU',JSON.stringify([type,a,b]),['ID','label','親（負はsize）'],names.map((name,i)=>[i,name,p[i]]),[p],{n:p.length,edges:p.flatMap((x,i)=>x>=0?[[i,x]]:[]),labels:names});}return{labels:names,groups:names.map((_,i)=>root(i)),answers};
 }
 const n=mode==='additive_disjoint_set_union'?numbers(input.values,24).length:integer(input.n,0,24),base=mode==='additive_disjoint_set_union'?input.values.slice():Array(n).fill(0),d=dsu(n),offset=Array(n).fill(0),answers=[];
 const check=v=>integer(v,0,n-1),show=note=>frame('DSUの状態',note,['頂点','親/size','成分','offset','点固有値','現在値'],d.p.map((p,i)=>[i,p,d.leader(i),offset[i],base[i],value(i)]),[d.p],{n,edges:d.p.flatMap((p,i)=>p>=0?[[i,p]]:[])});
 function value(i){let v=base[i];for(;;){v+=offset[i];if(d.p[i]<0)return v;i=d.p[i];}}
 if(mode==='additive_disjoint_set_union'){
  for(const [type,a,b]of operations(input)){check(a);if(type==='merge'){check(b);let u=d.leader(a),v=d.leader(b);if(u!==v){if(d.p[u]>d.p[v])[u,v]=[v,u];offset[v]-=offset[u];d.p[u]+=d.p[v];d.p[v]=u;}}else if(type==='component')offset[d.leader(a)]+=integer(b);else if(type==='vertex')base[a]+=integer(b);else throw Error('merge/component/vertexを指定');show(JSON.stringify([type,a,b]));}return{values:base.map((_,i)=>value(i)),sums:base.map((_,i)=>sum(base.map((_,j)=>d.leader(i)===d.leader(j)?value(j):0)))};
 }
 if(mode==='parallel_disjoint_set_union'){
  const levels=[d];for(let length=2;length<=n;length*=2)levels.push(dsu(n-length+1));
  function unite(k,a,b){const changed=levels[k].merge(a,b);frame(changed?'区間対を初めて併合':'既処理の区間対を省略',`level=${k}, 長さ=${2**k}, starts=${a},${b}`,['level','長さ','親/size'],levels.map((x,i)=>[i,2**i,x.p.join(' ')]),levels.map(x=>x.p));if(changed&&k){unite(k-1,a,b);unite(k-1,a+2**(k-1),b+2**(k-1));}}
  for(const [type,a,b,length]of operations(input)){check(a);check(b);if(type==='range'){integer(length,0,Math.min(n-a,n-b));if(length){const k=bitLength(length)-1;unite(k,a,b);unite(k,a+length-2**k,b+length-2**k);}}else if(type==='merge')d.merge(a,b);else if(type==='same')answers.push(d.leader(a)===d.leader(b));else throw Error('range/merge/sameを指定');show(JSON.stringify([type,a,b,length]));}return{groups:Array.from({length:n},(_,i)=>d.leader(i)),answers};
 }
 if(mode==='incremental_bridge_connectivity'){
  const edges=[];for(const [type,a,b]of operations(input)){check(a);check(b);if(type!=='add')throw Error('addを指定');edges.push([a,b]);const bridges=edges.map(([u,v],i)=>{const c=graphComponents(n,edges,i);return c[u]!==c[v];}),blocks=graphComponents(n,edges.filter((_,i)=>!bridges[i]));frame('橋と二重辺連結成分',`辺(${a},${b})を追加。橋${bridges.filter(Boolean).length}本。教材では各辺を除くBFS相当で分類し、C++の縮約森の配置は省略。`,['辺ID','u','v','橋'],edges.map(([u,v],i)=>[i,u,v,bridges[i]]),[blocks],{n,edges:edges.map(([u,v],i)=>[u,v,bridges[i]?'bridge':'cycle'])});answers.push(bridges.filter(Boolean).length);}return answers;
 }
 const ops=operations(input),open=new Map(),intervals=[],queries=[];
 ops.forEach(([type,a,b],t)=>{check(a);check(b);if(a>b)[a,b]=[b,a];const key=`${a},${b}`;if(type==='add'){const old=open.get(key);open.set(key,old?[old[0]+1,old[1]]:[1,t]);}else if(type==='remove'){const old=open.get(key);if(!old)throw Error('存在しない辺を削除');if(old[0]===1){intervals.push([old[1],t,a,b]);open.delete(key);}else old[0]--;}else if(type==='query')queries.push([t,a,b,queries.length]);else throw Error('add/remove/queryを指定');});
 for(const [key,[,start]]of open){const [a,b]=key.split(',').map(Number);intervals.push([start,ops.length,a,b]);}
 frame('辺の有効期間','同じ辺の追加回数が0になるまで有効。時間軸へ半開区間を登録。',['開始','終了','u','v'],intervals);
 let baseSize=1;while(baseSize<ops.length)baseSize*=2;const buckets=Array.from({length:2*baseSize},()=>[]);for(let[l,r,a,b]of intervals)for(l+=baseSize,r+=baseSize;l<r;l>>=1,r>>=1){if(l&1)buckets[l++].push([a,b]);if(r&1)buckets[--r].push([a,b]);}
 const result=Array(queries.length);function visit(k,l,r){const old=d.p.slice();for(const[a,b]of buckets[k])d.merge(a,b);show(`時間[${l},${r})へ入り ${JSON.stringify(buckets[k])}を併合`);if(r-l===1){for(const [t,a,b,id]of queries)if(t===l)result[id]=d.leader(a)===d.leader(b);}else{const m=(l+r)/2;visit(2*k,l,m);visit(2*k+1,m,r);}d.p.splice(0,d.p.length,...old);show(`時間[${l},${r})から戻りsnapshotへrollback`);}visit(1,0,baseSize);return result;
}

function rangeModel(mode,input,frame){
 if(mode==='persistent_lazy_segtree'){
  const a=numbers(input.values,24),nodes=[],roots=[];function make(l,r){const k=nodes.length;nodes.push({l,r,left:-1,right:-1,sum:sum(a.slice(l,r)),a:1,b:0});if(r-l>1){const m=(l+r)>>1;nodes[k].left=make(l,m);nodes[k].right=make(m,r);}return k;}roots.push(make(0,a.length));
  function tag(k,a,b){const n={...nodes[k]};n.sum=a*n.sum+b*(n.r-n.l);n.a=a*n.a;n.b=a*n.b+b;nodes.push(n);return nodes.length-1;}
  function update(k,l,r,a,b){const n=nodes[k];if(r<=n.l||n.r<=l)return k;if(l<=n.l&&n.r<=r)return tag(k,a,b);const left=update(tag(n.left,n.a,n.b),l,r,a,b),right=update(tag(n.right,n.a,n.b),l,r,a,b);nodes.push({...n,left,right,sum:nodes[left].sum+nodes[right].sum,a:1,b:0});return nodes.length-1;}
  function query(k,l,r,a=1,b=0){const n=nodes[k];if(r<=n.l||n.r<=l)return 0;if(l<=n.l&&n.r<=r)return a*n.sum+b*(n.r-n.l);return query(n.left,l,r,a*n.a,a*n.b+b)+query(n.right,l,r,a*n.a,a*n.b+b);}
  function show(note){frame('コピーされた節点とroot',note,['節点ID','区間','left','right','sum','lazy a','lazy b'],nodes.map((n,i)=>[i,`[${n.l},${n.r})`,n.left,n.right,n.sum,n.a,n.b]),[roots.map((k,i)=>`v${i}:root${k}`)],{tree:true,n:nodes.length,edges:nodes.flatMap((n,i)=>[n.left,n.right].filter(x=>x>=0).map(x=>[i,x])),labels:nodes.map(n=>String(n.sum)),activeVertices:roots});}
  const answers=[];show('初期版');for(const[type,v,l,r,x,y]of operations(input)){integer(v,0,roots.length-1);bounds(l,r,a.length);if(type==='apply'){integer(x,-4,4);integer(y,-20,20);roots.push(l===r?roots[v]:update(roots[v],l,r,x,y));}else if(type==='sum')answers.push(l===r?0:query(roots[v],l,r));else throw Error('apply/sumを指定');show(JSON.stringify([type,v,l,r,x,y]));}return{answers,totals:roots.map(k=>nodes[k].sum),nodeCount:nodes.length};
 }
 if(mode==='dual_segtree_2d'){
  const h=integer(input.height,0,8),w=integer(input.width,0,8);let x=1,y=1;while(x<h)x*=2;while(y<w)y*=2;const tags=Array.from({length:2*x},()=>Array(2*y).fill(0)),answers=[];
  for(const[type,a,b,c,d,value]of operations(input)){if(type==='add'){bounds(a,b,h);bounds(c,d,w);integer(value);for(let l=a+x,r=b+x;l<r;l>>=1,r>>=1){const rows=[];if(l&1)rows.push(l++);if(r&1)rows.push(--r);for(const row of rows)for(let u=c+y,v=d+y;u<v;u>>=1,v>>=1){if(u&1)tags[row][u++]+=value;if(v&1)tags[row][--v]+=value;}}}else if(type==='get'){integer(a,0,h-1);integer(b,0,w-1);let total=0;for(let i=a+x;i;i>>=1)for(let j=b+y;j;j>>=1)total+=tags[i][j];answers.push(total);}else throw Error('add/getを指定');frame('二軸のtag',JSON.stringify([type,a,b,c,d,value]),['x節点','y節点','加算tag'],tags.flatMap((row,i)=>row.flatMap((v,j)=>v?[[i,j,v]]:[])),tags);}return answers;
 }
 if(mode==='segtree_fractional_cascading'){
  const values=numbers(input.values,32),[l,r,lo,hi]=list(input.query,4);bounds(l,r,values.length);integer(lo);integer(hi);let base=1;while(base<values.length)base*=2;const tree=Array.from({length:2*base},()=>({values:[],left:[0]}));values.forEach((v,i)=>tree[base+i].values=[v]);
  for(let k=base-1;k;k--){const a=tree[2*k].values,b=tree[2*k+1].values,n=tree[k];let i=0,j=0;while(i<a.length||j<b.length){if(j===b.length||(i<a.length&&a[i]<=b[j]))n.values.push(a[i++]);else n.values.push(b[j++]);n.left.push(i);}frame('stable mergeとprefix対応',`節点${k}のprefixから左右の子へ写す`,['位置','値','左prefix','右prefix'],n.values.map((v,p)=>[p,v,n.left[p],p-n.left[p]]),[n.values,n.left]);}
  function visit(k,a,b,x,y){if(r<=a||b<=l)return 0;if(l<=a&&b<=r){frame('完全に含む節点',`節点${k} [${a},${b}) の答えは ${y-x}`,['lower位置','upper位置','個数'],[[x,y,y-x]],[tree[k].values]);return y-x;}const m=(a+b)>>1,px=tree[k].left[x],py=tree[k].left[y];return visit(2*k,a,m,px,py)+visit(2*k+1,m,b,x-px,y-py);}
  return lo>=hi?0:visit(1,0,base,tree[1].values.filter(x=>x<lo).length,tree[1].values.filter(x=>x<hi).length);
 }
 if(mode==='sortable_segtree'){
  const keys=numbers(input.keys,24),values=numbers(input.values,24);if(keys.length!==values.length||new Set(keys).size!==keys.length)throw Error('同じ長さの配列と一意keyが必要');let runs=keys.map((k,i)=>({items:[[k,values[i]]],desc:false}));const answers=[];
  function cut(position){let start=0;for(let i=0;i<runs.length;i++){const end=start+runs[i].items.length;if(position===start)return i;if(start<position&&position<end){const run=runs[i],p=position-start;runs.splice(i,1,{items:run.items.slice(0,p),desc:run.desc},{items:run.items.slice(p),desc:run.desc});return i+1;}start=end;}return runs.length;}
  function show(note){let start=0;frame('sorted runと順方向／逆方向集約',note,['開始','key列','payload列','方向','forward','backward'],runs.map(run=>{const s=start;start+=run.items.length;const v=run.items.map(x=>x[1]);return[s,run.items.map(x=>x[0]).join(' '),v.join(' '),run.desc?'降順':'昇順',sum(v),sum(v.slice().reverse())];}),[runs.flatMap(run=>run.items.map(x=>x[0])),runs.flatMap(run=>run.items.map(x=>x[1]))]);}
  show('各要素を一つのrunとして開始');for(const op of operations(input)){const[type,l,r,desc]=op;if(type==='set'){integer(l,0,keys.length-1);integer(r);integer(desc);cut(l+1);const at=cut(l),old=runs[at].items[0][0];if(r!==old&&runs.some(run=>run.items.some(item=>item[0]===r)))throw Error('一意keyが必要');runs[at]={items:[[r,desc]],desc:false};show(JSON.stringify(op));continue;}bounds(l,r,keys.length);cut(r);const a=cut(l),b=cut(r);if(type==='sort'){const items=runs.slice(a,b).flatMap(x=>x.items).sort((x,y)=>desc?y[0]-x[0]:x[0]-y[0]);if(l<r)runs.splice(a,b-a,{items,desc:!!desc});}else if(type==='sum')answers.push(sum(runs.slice(a,b).flatMap(x=>x.items.map(v=>v[1]))));else throw Error('sort/sum/setを指定');show(`${JSON.stringify([type,l,r,desc])}。教材ではrun内部の値域木をソート列で表示。`);}return{items:runs.flatMap(x=>x.items),answers};
 }
 const sparse=mode==='sparse_dynamic_segtree',n=sparse?integer(input.n,0,128):numbers(input.values,64).length,a=sparse?Array(n).fill(0):input.values.slice();let base=1;while(base<n)base*=2;const tree=Array(2*base).fill(0),history=[],snapshots=[],answers=[],created=new Set();for(let i=0;i<n;i++)tree[base+i]=a[i];for(let k=base-1;k;k--)tree[k]=tree[2*k]+tree[2*k+1];
 function show(note){const ids=Array.from({length:2*base-1},(_,i)=>i+1).filter(k=>!sparse||created.has(k));frame(sparse?'疎に作られた節点':'節点配列と変更履歴',note,['節点','値','履歴長'],ids.map(k=>[k,tree[k],history.length]),[a],ids.length?{tree:true,n:ids.length,edges:ids.flatMap((k,i)=>[2*k,2*k+1].filter(c=>ids.includes(c)).map(c=>[i,ids.indexOf(c)])),labels:ids.map(k=>`${k}:${tree[k]}`)}:null);}
 show('初期状態');for(const op of operations(input)){const[type,p,value]=op;if(type==='set'){integer(p,0,n-1);integer(value);a[p]=value;for(let k=base+p;k;k>>=1){history.push([k,tree[k]]);created.add(k);tree[k]=k>=base?value:tree[2*k]+tree[2*k+1];}}else if(type==='sum'){bounds(p,value,n);let total=0;for(let l=p+base,r=value+base;l<r;l>>=1,r>>=1){if(l&1)total+=tree[l++];if(r&1)total+=tree[--r];}answers.push(total);}else if(type==='snapshot'&&!sparse)snapshots.push(history.length);else if(type==='rollback'&&!sparse){integer(p,0,snapshots.length-1);const token=snapshots[p];if(token>history.length)throw Error('破棄した履歴のsnapshot');while(history.length>token){const[k,v]=history.pop();tree[k]=v;}for(let i=0;i<n;i++)a[i]=tree[base+i];}else throw Error('set/sum/snapshot/rollbackを指定');show(JSON.stringify(op));}return{values:a,answers};
}

function staticModel(mode,input,frame){
 if(mode==='priority_sum'){
  let k=integer(input.k,0,100),a=[];for(const[type,x]of operations(input)){integer(x);if(type==='insert')a.push(x);else if(type==='erase'){const i=a.indexOf(x);if(i>=0)a.splice(i,1);}else if(type==='k')k=integer(x,0,100);else throw Error('insert/erase/kを指定');a.sort(cmp);frame('選択側と残り',`${type} ${x}: 比較器順の先頭${k}個`,['区分','値','和'],[['selected',a.slice(0,k).join(' '),sum(a.slice(0,k))],['rest',a.slice(k).join(' '),sum(a.slice(k))]],[a.slice(0,k),a.slice(k)]);}return{values:a,sum:sum(a.slice(0,k))};
 }
 if(mode==='dynamic_bitset'){
  const n=integer(input.n,0,128);let a=Array(n).fill(0);for(const i of numbers(input.bits,128)){integer(i,0,n-1);a[i]=1;}for(const[type,x,v]of operations(input)){if(type==='set'){integer(x,0,n-1);a[x]=integer(v,0,1);}else if(type==='left'||type==='right'){integer(x,0,10000);const old=a;a=a.map((_,i)=>old[type==='left'?i-x:i+x]??0);}else if(type==='xor'){for(const i of numbers(x,128)){integer(i,0,n-1);a[i]^=1;}}else throw Error('set/left/right/xorを指定');frame('word境界をまたぐbit',`${type}。図は8bit word、C++は64bit word。`,['word','下位bitからの列','popcount'],Array.from({length:Math.ceil(n/8)},(_,i)=>[i,a.slice(i*8,i*8+8).join(''),sum(a.slice(i*8,i*8+8))]),[a]);}return{bits:a,count:sum(a)};
 }
 if(mode==='succinct_bit_vector'){
  if(typeof input.bits!=='string'||!/^[01]{0,128}$/.test(input.bits))throw Error('128文字以下の0/1列');const a=[...input.bits].map(Number),prefix=[0];for(let i=0;i<a.length;i+=8)prefix.push(prefix.at(-1)+sum(a.slice(i,i+8)));const rank=(r,b)=>{const ones=prefix[Math.floor(r/8)]+sum(a.slice(Math.floor(r/8)*8,r));return b?ones:r-ones;};const answers=[];
  frame('wordのprefix', '各wordの前までの1の個数。図は8bit、C++は64bit。',['word','prefix','bits'],prefix.slice(0,-1).map((v,i)=>[i,v,a.slice(i*8,i*8+8).join('')]),[a,prefix]);
  for(const[type,x,value]of list(input.queries,50)){const b=integer(value,0,1);if(type==='rank'){integer(x,0,a.length);answers.push(rank(x,b));}else if(type==='select'){integer(x,0,128);let lo=0,hi=a.length;while(lo<hi){const mid=(lo+hi)>>1;if(rank(mid+1,b)>x)hi=mid;else lo=mid+1;frame('selectのrank二分探索',`[${lo},${hi})、rank(${mid+1},${b})=${rank(mid+1,b)}`,['lo','hi'],[[lo,hi]],[a]);}answers.push(lo===a.length?-1:lo);}else throw Error('rank/selectを指定');frame('rank/selectの結果',JSON.stringify([type,x,b]),['答え'],[[answers.at(-1)]],[a]);}return answers;
 }
 if(mode==='persistent_binary_trie'){
  const width=integer(input.bits??8,1,8),nodes=[{child:[0,0],count:0}],roots=[0],answers=[];
  function change(k,bit,x,delta){const id=nodes.length;nodes.push({child:nodes[k].child.slice(),count:nodes[k].count+delta});if(bit>=0){const side=(x>>bit)&1;nodes[id].child[side]=change(nodes[k].child[side],bit-1,x,delta);}return id;}
  function count(k,x){for(let b=width-1;b>=0;b--)k=nodes[k].child[(x>>b)&1];return nodes[k].count;}
  for(const[type,v,x,y]of operations(input)){integer(v,0,roots.length-1);if(type==='insert'||type==='erase'){integer(x,0,2**width-1);if(type==='erase'&&!count(roots[v],x))throw Error('存在しないkeyの削除');roots.push(change(roots[v],width-1,x,type==='insert'?1:-1));}else if(type==='kth'){integer(x,0,1000);integer(y,0,2**width-1);let k=roots[v],rank=x,value=0;if(rank>=nodes[k].count)answers.push(null);else{for(let b=width-1;b>=0;b--){let side=(y>>b)&1;const zero=nodes[nodes[k].child[side]].count;if(rank>=zero){rank-=zero;side^=1;value|=1<<b;}k=nodes[k].child[side];}answers.push(value);}}else throw Error('insert/erase/kthを指定');frame('版ごとのrootとpath copy',`${JSON.stringify([type,v,x,y])}。図は${width}bit、C++は全64bit。`,['節点','0の子','1の子','個数'],nodes.map((n,i)=>[i,...n.child,n.count]),[roots.map((r,i)=>`v${i}:root${r}`)]);}return{sizes:roots.map(k=>nodes[k].count),answers};
 }
 if(mode==='weighted_wavelet_matrix'){
  const keys=numbers(input.keys,32),weights=numbers(input.weights,32);if(keys.length!==weights.length)throw Error('keyと重みは同じ長さ');const unique=[...new Set(keys)].sort(cmp),levels=bitLength(Math.max(0,unique.length-1)),bits=[],zeros=[],prefix=[];let cur=keys.map(k=>unique.indexOf(k)),w=weights.slice();
  for(let b=levels-1;b>=0;b--){const flags=cur.map(x=>(x>>b)&1),order=[...cur.keys()].filter(i=>!flags[i]).concat([...cur.keys()].filter(i=>flags[i]));zeros.push(flags.filter(x=>!x).length);bits.push(flags);cur=order.map(i=>cur[i]);w=order.map(i=>w[i]);const p=[0];for(const x of w)p.push(p.at(-1)+x);prefix.push(p);frame('重みもstable partition',`bit ${b}: 0の群→1の群、重みprefixを保持`,['位置','圧縮key','重み','prefix'],cur.map((x,i)=>[i,x,w[i],p[i+1]]),[flags,cur,w,p]);}
  const original=[0];for(const x of weights)original.push(original.at(-1)+x);if(!levels)prefix.push(original);
  const [l,r,lo,hi,k]=list(input.query,5);bounds(l,r,keys.length);integer(lo);integer(hi);integer(k,0,r-l);
  function less(x){let a=l,b=r,result=0;if(x>=unique.length)return original[b]-original[a];for(let d=0;d<levels;d++){const al=sum(bits[d].slice(0,a)),br=sum(bits[d].slice(0,b)),zl=a-al,zr=b-br;if((x>>(levels-1-d))&1){result+=prefix[d][zr]-prefix[d][zl];a=zeros[d]+al;b=zeros[d]+br;}else{a=zl;b=zr;}}return result;}
  let a=l,b=r,remain=k,small=0;for(let d=0;d<levels;d++){const al=sum(bits[d].slice(0,a)),br=sum(bits[d].slice(0,b)),zl=a-al,zr=b-br;if(remain<zr-zl){a=zl;b=zr;}else{small+=prefix[d][zr]-prefix[d][zl];remain-=zr-zl;a=zeros[d]+al;b=zeros[d]+br;}frame('小さいk個の和',`level ${d}, 残り${remain}, 確定和${small}`,['l','r','残りk','確定和'],[[a,b,remain,small]]);}small+=prefix.at(-1)[a+remain]-prefix.at(-1)[a];return{rangeSum:lo>=hi?0:less(unique.filter(x=>x<hi).length)-less(unique.filter(x=>x<lo).length),smallest:small};
 }
 const a=numbers(input.values,64);
 if(mode==='wavelet_tree'){
  const values=[...new Set(a)].sort(cmp),nodes=[];
  function make(items,lo,hi){const id=nodes.length,node={lo,hi,prefix:[0],left:-1,right:-1};nodes.push(node);if(hi-lo>1){const mid=(lo+hi)>>1,l=[],r=[];for(const x of items){(x<mid?l:r).push(x);node.prefix.push(l.length);}node.left=make(l,lo,mid);node.right=make(r,mid,hi);}frame('値域を二分',`node ${id}: rank [${lo},${hi})`,['prefix位置','左の個数'],node.prefix.map((x,i)=>[i,x]),[items,node.prefix]);return id;}make(a.map(x=>values.indexOf(x)),0,values.length);
  const [l,r,lo,hi,k]=list(input.query,5);bounds(l,r,a.length);integer(lo);integer(hi);integer(k,0,Math.max(0,r-l));
  function less(id,l,r,x){const n=nodes[id];if(!values.length||x<=n.lo||l===r)return 0;if(n.hi<=x)return r-l;const p=n.prefix[l],q=n.prefix[r];return less(n.left,p,q,x)+less(n.right,l-p,r-q,x);}
  let id=0,x=l,y=r,rank=k,kth=null;if(k<r-l){while(nodes[id].hi-nodes[id].lo>1){const n=nodes[id],p=n.prefix[x],q=n.prefix[y];if(rank<q-p){x=p;y=q;id=n.left;}else{rank-=q-p;x-=p;y-=q;id=n.right;}frame('rankを子へ写す',`節点${id}: [${x},${y}), 残りrank=${rank}`,['node','l','r','rank'],[[id,x,y,rank]]);}kth=values[nodes[id].lo];}return{count:lo>=hi?0:less(0,l,r,values.filter(x=>x<hi).length)-less(0,l,r,values.filter(x=>x<lo).length),kth};
 }
 if(mode==='linear_rmq'){
  const width=Math.max(1,Math.floor((bitLength(a.length)-1)/2)),masks=[],mins=[];
  function better(i,j){return i<0?j:j<0?i:a[j]<a[i]?j:a[i]<a[j]?i:Math.min(i,j);}
  for(let i=0;i<a.length;i++){let mask=i%width?masks[i-1]:0;while(mask){const p=bitLength(mask)-1,index=Math.floor(i/width)*width+p;if(a[index]<=a[i])break;mask^=1<<p;}masks.push(mask|(1<<(i%width)));const b=Math.floor(i/width);mins[b]=better(mins[b]??-1,i);frame('単調stackをbitmask化',`位置${i}。同値をpopせず最左を残す。`,['位置','値','mask'],a.slice(0,i+1).map((x,j)=>[j,x,masks[j].toString(2)]),[a,masks]);}
  const sparse=[mins];for(let k=1;2**k<=mins.length;k++)sparse.push(Array.from({length:mins.length-2**k+1},(_,i)=>better(sparse[k-1][i],sparse[k-1][i+2**(k-1)])));
  function inside(l,r){const mask=masks[r-1]&(~0<<(l%width));let p=0;while(!((mask>>p)&1))p++;return Math.floor(l/width)*width+p;}
  const answers=[];for(const[l,r]of list(input.queries,50)){bounds(l,r,a.length);let best=-1;if(l<r){if(Math.floor(l/width)===Math.floor((r-1)/width))best=inside(l,r);else{const x=Math.floor(l/width)+1,y=Math.floor((r-1)/width);best=better(inside(l,x*width),inside(y*width,r));if(x<y){const k=bitLength(y-x)-1;best=better(best,better(sparse[k][x],sparse[k][y-2**k]));}}}answers.push(best);frame('端blockと中央blockのRMQ',`[${l},${r}) → argmin=${best}`,['ブロック','最小添字'],mins.map((x,i)=>[i,x]),[a,mins],null,best>=0?[[0,best]]:[]);}return answers;
 }
 // Static Sqrt Tree: prefix/suffix/between with highest-differing-bit lookup.
 let base=1;while(base<a.length)base*=2;const values=a.concat(Array(base-a.length).fill(0)),layers=[],lookup=[];
 for(let k=bitLength(base)-1;k>1;k=Math.ceil(k/2)){const half=Math.ceil(k/2),width=2**half,count=2**(k-half),prefix=Array(base).fill(0),suffix=Array(base).fill(0),between=Array(base).fill(0),index=layers.length;for(let bit=half;bit<k;bit++)lookup[bit]=index;for(let start=0;start<base;start+=2**k){for(let b=0;b<count;b++){let p=0,s=0;const l=start+b*width,r=l+width;for(let i=l;i<r;i++)prefix[i]=p+=values[i];for(let i=r-1;i>=l;i--)suffix[i]=s+=values[i];}for(let l=0;l<count;l++){let p=0;for(let r=l;r<count;r++)between[start+l*count+r]=p+=prefix[start+(r+1)*width-1];}}layers.push({k,half,count,prefix,suffix,between});frame('平方分割の層',`幅2^${k}を2^${half}ずつ分ける`,['位置','prefix','suffix','between'],values.map((_,i)=>[i,prefix[i],suffix[i],between[i]]),[values,prefix,suffix,between]);}
 const answers=[];for(const[l,r]of list(input.queries,50)){bounds(l,r,a.length);let value=0;if(r-l===1)value=values[l];else if(l<r){const bit=bitLength(l^(r-1))-1;if(bit===0)value=values[l]+values[r-1];else{const layer=layers[lookup[bit]],start=(l>>layer.k)<<layer.k,x=(l-start)>>layer.half,y=(r-1-start)>>layer.half;value=layer.suffix[l]+layer.prefix[r-1];if(x+1<y)value+=layer.between[start+(x+1)*layer.count+y-1];}}answers.push(value);frame('最高相違bitで層を選択',`[${l},${r}) の和=${value}`,['答え'],[[value]],[a]);}return answers;
}

function orderedModel(mode,input,frame){
 if(mode==='radix_heap'){
  const buckets=Array.from({length:9},()=>[]),answers=[];let last=0;
  function pull(){if(!buckets[0].length){let k=1;while(k<9&&!buckets[k].length)k++;if(k===9)throw Error('空のheap');last=Math.min(...buckets[k]);const items=buckets[k];buckets[k]=[];for(const x of items)buckets[bitLength(x^last)].push(x);frame('bucketを再分配',`新しいlast=${last}、bucket${k}を分配`,['bucket','keys'],buckets.map((a,i)=>[i,a.join(' ')]),buckets);}}
  for(const[type,key]of operations(input)){if(type==='push'){integer(key,last,255);buckets[bitLength(key^last)].push(key);}else if(type==='pop'){pull();answers.push(buckets[0].pop());}else throw Error('push/popを指定');frame('最高相違bitのbucket',`${type}。教材は8bit、C++は32/64bit。last=${last}`,['bucket','keys'],buckets.map((a,i)=>[i,a.join(' ')]),buckets);}return{last,answers,remaining:buckets.flat().sort(cmp)};
 }
 if(mode==='erasable_priority_queue'){
  const heap=[],deleted=[],counts=new Map(),answers=[];
  function clean(){heap.sort((a,b)=>b-a);deleted.sort((a,b)=>b-a);while(deleted.length&&heap[0]===deleted[0]){const x=heap.shift();deleted.shift();frame('topの削除予約を回収',`同じtop ${x}を両heapから除く`,['heap','keys'],[['values',heap.join(' ')],['deleted',deleted.join(' ')]],[heap,deleted]);}}
  for(const[type,x]of operations(input)){if(type==='push'){integer(x);heap.push(x);counts.set(x,(counts.get(x)??0)+1);}else if(type==='erase'){integer(x);if(counts.get(x)){counts.set(x,counts.get(x)-1);deleted.push(x);}}else if(type==='pop'){clean();if(!heap.length)throw Error('空のheap');const x=heap.shift();answers.push(x);counts.set(x,counts.get(x)-1);}else throw Error('push/erase/popを指定');clean();frame('値heapと削除予約',JSON.stringify([type,x]),['heap','keys'],[['values',heap.join(' ')],['deleted',deleted.join(' ')]],[heap,deleted]);}return{answers,values:[...counts].flatMap(([x,n])=>Array(n).fill(x)).sort(cmp)};
 }
 if(mode==='skew_heap'||mode==='leftist_heap'){
  let serial=0;const roots=[null,null],answers=[];
  const rank=n=>n?.rank??0;function merge(a,b){if(!a||!b)return a??b;if(a.key<b.key)[a,b]=[b,a];a.right=merge(a.right,b);if(mode==='skew_heap'||rank(a.left)<rank(a.right))[a.left,a.right]=[a.right,a.left];a.rank=rank(a.right)+1;return a;}
  function show(note){const nodes=[],edges=[];function visit(n){if(!n)return;const id=nodes.length;nodes.push(n);for(const child of[n.left,n.right])if(child){const next=nodes.length;visit(child);edges.push([id,next]);}}roots.forEach(visit);frame('併合可能heap',note,['ID','key','rank','left','right'],nodes.map(n=>[n.id,n.key,n.rank,n.left?.id??'-',n.right?.id??'-']),[],{tree:true,n:nodes.length,edges,labels:nodes.map(n=>`${n.key} (r${n.rank})`)});}
  for(const[type,a,x]of operations(input)){integer(a,0,1);if(type==='push'){integer(x);roots[a]=merge(roots[a],{id:serial++,key:x,rank:1,left:null,right:null});}else if(type==='meld'){integer(x,0,1);if(a!==x){roots[a]=merge(roots[a],roots[x]);roots[x]=null;}}else if(type==='pop'){if(!roots[a])throw Error('空のheap');answers.push(roots[a].key);roots[a]=merge(roots[a].left,roots[a].right);}else throw Error('push/meld/popを指定');show(JSON.stringify([type,a,x]));}return{answers,tops:roots.map(n=>n?.key??null)};
 }
 const nodes=[];let rootNode=0;const answers=[],versions=[0];let seed=712367821;
 const size=k=>k?nodes[k-1].size:0,at=k=>nodes[k-1],red=k=>!!k&&at(k).red;
 function node(key,parent=0){nodes.push({key,count:1,size:1,weight:1,left:0,right:0,parent,red:true,priority:0});return nodes.length;}
 function pull(k){if(k){const n=at(k);n.size=size(n.left)+n.count+size(n.right);n.weight=(n.left?at(n.left).weight:0)+1+(n.right?at(n.right).weight:0);}return k;}
 function show(note,current=rootNode){const ids=[],edges=[];function visit(k){if(!k)return;const index=ids.length;ids.push(k);for(const child of[at(k).left,at(k).right])if(child){const next=ids.length;visit(child);edges.push([index,next,mode==='red_black_tree'?(at(child).red?'red':'black'):'']);}}visit(current);frame('木と部分木サイズ',note,['ID','key','count','size','left','right','色/weight'],ids.map(k=>{const n=at(k);return[k,n.key,n.count,n.size,n.left,n.right,mode==='red_black_tree'?(n.red?'red':'black'):n.weight];}),[ids.map(k=>at(k).key)],{tree:true,n:ids.length,edges,colors:mode==='red_black_tree'?ids.map(k=>at(k).red?'#b74e4e':'#304640'):null,labels:ids.map(k=>`${at(k).key} ×${at(k).count}`),activeVertices:ids.filter(k=>at(k).red&&mode==='red_black_tree').map(k=>ids.indexOf(k))});}
 function rotateSplay(x){const p=at(x).parent,g=at(p).parent,right=at(p).right===x,middle=right?at(x).left:at(x).right;if(g){if(at(g).left===p)at(g).left=x;else at(g).right=x;}else rootNode=x;at(x).parent=g;if(right){at(p).right=middle;at(x).left=p;}else{at(p).left=middle;at(x).right=p;}at(p).parent=x;if(middle)at(middle).parent=p;pull(p);pull(x);show(`節点${x}を回転`);}
 function splay(x){while(at(x).parent){const p=at(x).parent,g=at(p).parent;if(g)rotateSplay((at(g).left===p)===(at(p).left===x)?p:x);rotateSplay(x);}}
 function find(key){let k=rootNode,last=0;while(k){last=k;if(key===at(k).key){splay(k);return k;}k=key<at(k).key?at(k).left:at(k).right;}if(last)splay(last);return 0;}
 function leftRotate(k){const x=at(k).right;at(k).right=at(x).left;at(x).left=k;at(x).red=at(k).red;at(k).red=true;pull(k);pull(x);return x;}
 function rightRotate(k){const x=at(k).left;at(k).left=at(x).right;at(x).right=k;at(x).red=at(k).red;at(k).red=true;pull(k);pull(x);return x;}
 function flip(k){at(k).red=!at(k).red;at(at(k).left).red=!at(at(k).left).red;at(at(k).right).red=!at(at(k).right).red;}
 function fix(k){if(red(at(k).right))k=leftRotate(k);if(red(at(k).left)&&red(at(at(k).left).left))k=rightRotate(k);if(red(at(k).left)&&red(at(k).right))flip(k);return pull(k);}
 function moveLeft(k){flip(k);if(red(at(at(k).right).left)){at(k).right=rightRotate(at(k).right);k=leftRotate(k);flip(k);}return k;}
 function moveRight(k){flip(k);if(red(at(at(k).left).left)){k=rightRotate(k);flip(k);}return k;}
 function rbInsert(k,key){if(!k)return node(key);if(key<at(k).key)at(k).left=rbInsert(at(k).left,key);else if(key>at(k).key)at(k).right=rbInsert(at(k).right,key);else at(k).count++;if(red(at(k).right)&&!red(at(k).left))k=leftRotate(k);if(red(at(k).left)&&red(at(at(k).left).left))k=rightRotate(k);if(red(at(k).left)&&red(at(k).right))flip(k);return pull(k);}
 function rbMin(k){if(!at(k).left)return 0;if(!red(at(k).left)&&!red(at(at(k).left).left))k=moveLeft(k);at(k).left=rbMin(at(k).left);return fix(k);}
 function rbErase(k,key){if(key<at(k).key){if(!red(at(k).left)&&!red(at(at(k).left).left))k=moveLeft(k);at(k).left=rbErase(at(k).left,key);}else{if(red(at(k).left))k=rightRotate(k);if(key===at(k).key&&!at(k).right)return 0;if(!red(at(k).right)&&!red(at(at(k).right).left))k=moveRight(k);if(key===at(k).key){let next=at(k).right;while(at(next).left)next=at(next).left;at(k).key=at(next).key;at(k).count=at(next).count;at(k).right=rbMin(at(k).right);}else at(k).right=rbErase(at(k).right,key);}return fix(k);}
 function count(k,key){while(k){if(key===at(k).key)return at(k).count;k=key<at(k).key?at(k).left:at(k).right;}return 0;}
 function decrement(k,key){if(key===at(k).key)at(k).count--;else if(key<at(k).key)decrement(at(k).left,key);else decrement(at(k).right,key);pull(k);}
 function flatten(k,ids){if(!k)return;flatten(at(k).left,ids);ids.push(k);flatten(at(k).right,ids);}
 function balanced(ids,l,r){if(l===r)return 0;const m=(l+r)>>1,k=ids[m];at(k).left=balanced(ids,l,m);at(k).right=balanced(ids,m+1,r);return pull(k);}
 function balance(k){if(!k)return 0;pull(k);const n=at(k),weight=x=>x?at(x).weight:0;if(4*Math.max(weight(n.left),weight(n.right))>3*n.weight){const ids=[];flatten(k,ids);k=balanced(ids,0,ids.length);}return k;}
 function wbInsert(k,key){if(!k)return node(key);if(key<at(k).key)at(k).left=wbInsert(at(k).left,key);else if(key>at(k).key)at(k).right=wbInsert(at(k).right,key);else at(k).count++;return balance(k);}
 function wbMin(k){if(!at(k).left)return at(k).right;at(k).left=wbMin(at(k).left);return balance(k);}
 function wbErase(k,key){if(key<at(k).key)at(k).left=wbErase(at(k).left,key);else if(key>at(k).key)at(k).right=wbErase(at(k).right,key);else if(at(k).count>1)at(k).count--;else{if(!at(k).left)return at(k).right;if(!at(k).right)return at(k).left;let next=at(k).right;while(at(next).left)next=at(next).left;at(k).key=at(next).key;at(k).count=at(next).count;at(k).right=wbMin(at(k).right);}return balance(k);}
 function clone(k){nodes.push({...at(k)});return nodes.length;}
 function random(){seed^=seed<<13;seed^=seed>>>17;seed^=seed<<5;return seed>>>0;}
 function treapInsert(k,key,priority){if(!k){const x=node(key);at(x).priority=priority;return x;}k=clone(k);if(key===at(k).key)at(k).count++;else if(key<at(k).key){at(k).left=treapInsert(at(k).left,key,priority);if(at(at(k).left).priority>at(k).priority)k=rightRotate(k);}else{at(k).right=treapInsert(at(k).right,key,priority);if(at(at(k).right).priority>at(k).priority)k=leftRotate(k);}return pull(k);}
 function treapMerge(a,b){if(!a||!b)return a||b;if(at(a).priority>at(b).priority){a=clone(a);at(a).right=treapMerge(at(a).right,b);return pull(a);}b=clone(b);at(b).left=treapMerge(a,at(b).left);return pull(b);}
 function treapErase(k,key){if(!k)return 0;k=clone(k);if(key<at(k).key)at(k).left=treapErase(at(k).left,key);else if(key>at(k).key)at(k).right=treapErase(at(k).right,key);else if(at(k).count>1)at(k).count--;else return treapMerge(at(k).left,at(k).right);return pull(k);}
 function kth(k,rank){if(rank<0||rank>=size(k))return null;while(k){const l=size(at(k).left);if(rank<l)k=at(k).left;else if(rank<l+at(k).count)return k;else{rank-=l+at(k).count;k=at(k).right;}}return null;}
 for(const op of operations(input)){
  const type=op[0],persistent=mode==='persistent_balanced_bst',version=persistent?integer(op[1],0,versions.length-1):0,key=integer(op[persistent?2:1]);if(persistent)rootNode=versions[version];
  if(type==='insert'){
   if(mode==='splay_tree'){if(!rootNode)rootNode=node(key);else{let k=rootNode;for(;;){if(key===at(k).key){at(k).count++;pull(k);splay(k);break;}const side=key<at(k).key?'left':'right';if(at(k)[side])k=at(k)[side];else{const x=node(key,k);at(k)[side]=x;splay(x);break;}}}}
   else if(mode==='red_black_tree'){rootNode=rbInsert(rootNode,key);at(rootNode).red=false;}
   else if(mode==='weight_balanced_tree')rootNode=wbInsert(rootNode,key);
   else{rootNode=treapInsert(rootNode,key,random());versions.push(rootNode);}
  }else if(type==='erase'){
   if(mode==='splay_tree'){const k=find(key);if(k){if(at(k).count>1){at(k).count--;pull(k);}else{const left=at(k).left,right=at(k).right;if(left)at(left).parent=0;if(right)at(right).parent=0;rootNode=left;if(!left)rootNode=right;else{let x=left;while(at(x).right)x=at(x).right;splay(x);at(x).right=right;if(right)at(right).parent=x;pull(x);}}}}
   else if(mode==='red_black_tree'){const c=count(rootNode,key);if(c>1)decrement(rootNode,key);else if(c){if(!red(at(rootNode).left)&&!red(at(rootNode).right))at(rootNode).red=true;rootNode=rbErase(rootNode,key);if(rootNode)at(rootNode).red=false;}}
   else if(mode==='weight_balanced_tree'){if(count(rootNode,key))rootNode=wbErase(rootNode,key);}
   else{rootNode=treapErase(rootNode,key);versions.push(rootNode);}
  }else if(type==='kth'){const k=kth(rootNode,key);answers.push(k?at(k).key:null);if(k&&mode==='splay_tree')splay(k);}else throw Error('insert/erase/kthを指定');
  show(JSON.stringify(op));
 }
 function values(k){if(!k)return[];return [...values(at(k).left),...Array(at(k).count).fill(at(k).key),...values(at(k).right)];}
 return mode==='persistent_balanced_bst'?{versions:versions.map(values),answers}:{values:values(rootNode),answers};
}

function spatialModel(mode,input,frame){
 if(mode==='hilbert_order_mo'||mode==='static_range_count_distinct'){
  const a=numbers(input.values,48),queries=list(input.queries,50).map(q=>{list(q,2);bounds(q[0],q[1],a.length);return q;}),answer=Array(queries.length);
  if(mode==='static_range_count_distinct'){
   const last=new Map(),bit=Array(a.length+1).fill(0),marks=Array(a.length).fill(0);let r=0;function add(i,x){marks[i]+=x;for(i++;i<bit.length;i+=i&-i)bit[i]+=x;}function prefix(r){let s=0;for(;r;r-=r&-r)s+=bit[r];return s;}
   const order=[...queries.keys()].sort((i,j)=>queries[i][1]-queries[j][1]);for(const id of order){const[l,qr]=queries[id];while(r<qr){if(last.has(a[r]))add(last.get(a[r]),-1);add(r,1);last.set(a[r],r++);frame('最後の出現だけ1にする',`右端=${r}`,['値','最後の位置'],[...last],[a,marks,bit]);}answer[id]=prefix(qr)-prefix(l);frame('区間和が種類数',`query ${id} [${l},${qr}) → ${answer[id]}`,['ID','種類数'],[[id,answer[id]]],[a,marks,bit]);}return answer;
  }
  function hilbert(x,y,bits){if(!bits)return 0;const half=2**(bits-1),quadrant=x<half?(y<half?0:3):(y<half?1:2);x%=half;y%=half;if(quadrant===0)[x,y]=[y,x];if(quadrant===3)[x,y]=[half-1-y,half-1-x];return quadrant*4**(bits-1)+hilbert(x,y,bits-1);}
  const bits=Math.max(1,bitLength(a.length)),keys=queries.map(([l,r])=>hilbert(l,r,bits)),order=[...queries.keys()].sort((i,j)=>keys[i]-keys[j]||i-j),freq=new Map();let l=0,r=0;frame('Hilbert順へ並べ替え','左右端を平面の点と見なす',['ID','l','r','Hilbert'],order.map(i=>[i,...queries[i],keys[i]]));
  function move(i,delta){freq.set(a[i],(freq.get(a[i])??0)+delta);if(!freq.get(a[i]))freq.delete(a[i]);frame(delta>0?'区間へ追加':'区間から削除',`位置${i}、現在[${l},${r})`,['値','頻度'],[...freq],[a],null,[[0,i]]);}
  for(const id of order){const[ql,qr]=queries[id];while(l>ql)move(--l,1);while(r<qr){const i=r++;move(i,1);}while(l<ql){const i=l++;move(i,-1);}while(r>qr)move(--r,-1);answer[id]=freq.size;frame('問い合わせ完了',`ID ${id} [${l},${r}) → ${answer[id]}`,['ID','答え'],[[id,answer[id]]],[a]);}return answer;
 }
 if(mode==='rectangle_add_rectangle_sum'||mode==='rectangle_add_point_get'){
  const corners=new Map(),answers=[];function corner(x,y,w){const key=`${x},${y}`,c=corners.get(key)??[x,y,0,0,0,0];c[2]+=w;c[3]-=w*x;c[4]-=w*y;c[5]+=w*x*y;corners.set(key,c);}
  function prefix(x,y,inclusive=false){const c=[0,0,0,0];for(const[a,b,...w]of corners.values())if(inclusive?a<=x&&b<=y:a<x&&b<y)for(let i=0;i<4;i++)c[i]+=w[i];return inclusive?c[0]:c[0]*x*y+c[1]*y+c[2]*x+c[3];}
  for(const op of operations(input)){const[type,l,d,r,u,w]=op;if(type==='add'){[l,d,r,u].forEach(x=>integer(x,-30,30));integer(w,-30,30);if(l<r&&d<u){corner(l,d,w);corner(l,u,-w);corner(r,d,-w);corner(r,u,w);}}else if(type==='get'&&mode==='rectangle_add_point_get'){integer(l,-30,30);integer(d,-30,30);answers.push(prefix(l,d,true));}else if(type==='sum'&&mode==='rectangle_add_rectangle_sum'){[l,d,r,u].forEach(x=>integer(x,-30,30));answers.push(l>=r||d>=u?0:prefix(r,u)-prefix(l,u)-prefix(r,d)+prefix(l,d));}else throw Error('addとsum/getを指定');frame('四隅の差分と係数',`${JSON.stringify(op)}。BITの物理配置を省略し、各cornerの係数を表示。`,['x','y','w','−wx','−wy','wxy'],[...corners.values()]);}return answers;
 }
 if(mode==='range_tree'){
  const points=list(input.points,32).map(p=>{list(p,2);return p.map(x=>integer(x,-50,50));}),weights=points.map(()=>0),order=[...points.keys()].sort((a,b)=>points[a][0]-points[b][0]),position=points.map((_,i)=>order.indexOf(i));let base=1;while(base<points.length)base*=2;const ys=Array.from({length:2*base},()=>[]),bit=ys.map(()=>[]);order.forEach((id,i)=>ys[base+i]=[points[id][1]]);for(let k=base-1;k;k--)ys[k]=[...new Set([...ys[2*k],...ys[2*k+1]])].sort(cmp);for(let k=1;k<ys.length;k++)bit[k]=Array(ys[k].length+1).fill(0);
  const answers=[];function prefix(k,y){let p=ys[k].filter(x=>x<y).length,s=0;for(;p;p-=p&-p)s+=bit[k][p];return s;}
  for(const[type,a,b,c,d]of operations(input)){if(type==='add'||type==='set'){integer(a,0,points.length-1);integer(b);const delta=type==='set'?b-weights[a]:b;weights[a]+=delta;for(let k=base+position[a];k;k>>=1)for(let p=ys[k].indexOf(points[a][1])+1;p<bit[k].length;p+=p&-p)bit[k][p]+=delta;}else if(type==='sum'){[a,b,c,d].forEach(x=>integer(x,-100,100));let result=0;if(a<c&&b<d){let l=order.filter(i=>points[i][0]<a).length+base,r=order.filter(i=>points[i][0]<c).length+base;for(;l<r;l>>=1,r>>=1){if(l&1){result+=prefix(l,d)-prefix(l,b);l++;}if(r&1){--r;result+=prefix(r,d)-prefix(r,b);}}}answers.push(result);}else throw Error('add/set/sumを指定');frame('x節点ごとのy列とBIT',JSON.stringify([type,a,b,c,d]),['x節点','y座標','BIT'],ys.flatMap((y,i)=>y.length?[[i,y.join(' '),bit[i].join(' ')]]:[]),[weights]);}return{weights,answers};
 }
 // KdTree models: alternating median partition, closed boxes and lazy affine.
 const points=[],nodes=[];let rootNode=-1;const answers=[];
 function make(ids,depth,parent=-1){if(!ids.length)return-1;const axis=depth%2;ids.sort((a,b)=>points[a][axis]-points[b][axis]||a-b);const m=ids.length>>1,id=ids[m],k=nodes.length,p=points[id];nodes.push({id,left:-1,right:-1,parent,size:1,xmin:p[0],xmax:p[0],ymin:p[1],ymax:p[1],sum:p[2],a:1,b:0});nodes[k].left=make(ids.slice(0,m),depth+1,k);nodes[k].right=make(ids.slice(m+1),depth+1,k);pull(k);return k;}
 function pull(k){const n=nodes[k],p=points[n.id];n.size=1;n.sum=p[2];n.xmin=n.xmax=p[0];n.ymin=n.ymax=p[1];for(const child of[n.left,n.right])if(child>=0){const c=nodes[child];n.size+=c.size;n.sum+=c.sum;n.xmin=Math.min(n.xmin,c.xmin);n.xmax=Math.max(n.xmax,c.xmax);n.ymin=Math.min(n.ymin,c.ymin);n.ymax=Math.max(n.ymax,c.ymax);}}
 function tag(k,a,b){if(k<0)return;const n=nodes[k];points[n.id][2]=a*points[n.id][2]+b;n.sum=a*n.sum+b*n.size;n.a=a*n.a;n.b=a*n.b+b;}
 function push(k){const n=nodes[k];tag(n.left,n.a,n.b);tag(n.right,n.a,n.b);n.a=1;n.b=0;}
 function flush(k){if(k<0)return;push(k);flush(nodes[k].left);flush(nodes[k].right);}
 function relation(n,l,d,r,u){if(n.xmax<l||n.xmin>=r||n.ymax<d||n.ymin>=u)return 0;return l<=n.xmin&&n.xmax<r&&d<=n.ymin&&n.ymax<u?2:1;}
 function show(note,active=[]){frame('Kd木のbounding boxと集約',note,['節点','点ID','box','size','sum','lazy a,b'],nodes.map((n,k)=>[k,n.id,`[${n.xmin},${n.xmax}] × [${n.ymin},${n.ymax}]`,n.size,n.sum,`${n.a},${n.b}`]),[points.map(p=>`${p[0]},${p[1]}:${p[2]}`)],{tree:true,n:nodes.length,edges:nodes.flatMap((n,i)=>[n.left,n.right].filter(k=>k>=0).map(k=>[i,k])),labels:nodes.map(n=>`p${n.id}:Σ${n.sum}`),activeVertices:active});}
 function update(k,l,d,r,u,a,b){if(k<0)return;const n=nodes[k],type=relation(n,l,d,r,u);if(!type){show(`節点${k}のboxは矩形外。枝刈り`,[k]);return;}if(type===2){tag(k,a,b);show(`節点${k}全体へw←${a}w+${b}`,[k]);return;}push(k);const p=points[n.id];if(l<=p[0]&&p[0]<r&&d<=p[1]&&p[1]<u)p[2]=a*p[2]+b;update(n.left,l,d,r,u,a,b);update(n.right,l,d,r,u,a,b);pull(k);}
 function query(k,l,d,r,u,report=false){if(k<0)return report?[]:0;const n=nodes[k],type=relation(n,l,d,r,u);if(!type){show(`節点${k}を枝刈り`,[k]);return report?[]:0;}if(type===2&&!report){show(`節点${k}の和を一括取得`,[k]);return n.sum;}push(k);const p=points[n.id],inside=l<=p[0]&&p[0]<r&&d<=p[1]&&p[1]<u,x=query(n.left,l,d,r,u,report),y=query(n.right,l,d,r,u,report);return report?[...(inside?[n.id]:[]),...x,...y]:(inside?p[2]:0)+x+y;}
 if(mode==='kd_tree'){
  for(const p of list(input.points,32)){list(p,2);points.push([integer(p[0],-50,50),integer(p[1],-50,50),0]);}rootNode=make([...points.keys()],0);show('交互の中央値分割で構築');
  for(const[type,l,d,r,u]of operations(input)){if(type==='report'){[l,d,r,u].forEach(x=>integer(x,-100,100));answers.push(l>=r||d>=u?[]:query(rootNode,l,d,r,u,true).sort(cmp));}else if(type==='nearest'){integer(l,-100,100);integer(d,-100,100);let best=-1,distance=Infinity;function box(k){if(k<0)return Infinity;const n=nodes[k];return Math.max(n.xmin-l,0,l-n.xmax)**2+Math.max(n.ymin-d,0,d-n.ymax)**2;}function visit(k){if(k<0||box(k)>distance)return;const n=nodes[k],p=points[n.id],v=(p[0]-l)**2+(p[1]-d)**2;if(v<distance||(v===distance&&(best<0||n.id<best))){best=n.id;distance=v;}show(`最近点候補p${best}、二乗距離${distance}`,[k]);let a=n.left,b=n.right;if(box(a)>box(b))[a,b]=[b,a];visit(a);visit(b);}visit(rootNode);answers.push(best<0?null:[best,distance]);}else throw Error('report/nearestを指定');}return answers;
 }
 for(const[type,l,d,r,u,a,b]of operations(input)){
  if(type==='point'){integer(l,-50,50);integer(d,-50,50);integer(r,-100,100);flush(rootNode);points.push([l,d,r]);nodes.length=0;rootNode=make([...points.keys()],0);show('新点は過去の作用を受けない。教材では全体再構築（C++は部分再構築）。');}
  else if(type==='set'){integer(l,0,points.length-1);integer(d,-100,100);flush(rootNode);points[l][2]=d;function repair(k){if(k<0)return;repair(nodes[k].left);repair(nodes[k].right);pull(k);}repair(rootNode);show(`点ID${l}を${d}へ代入`);}
  else if(type==='affine'||type==='sum'){[l,d,r,u].forEach(x=>integer(x,-100,100));if(type==='affine'){integer(a,-4,4);integer(b,-20,20);if(l<r&&d<u)update(rootNode,l,d,r,u,a,b);}else answers.push(l>=r||d>=u?0:query(rootNode,l,d,r,u));show(JSON.stringify([type,l,d,r,u,a,b]));}
  else throw Error('point/set/affine/sumを指定');
 }
 flush(rootNode);return{points,answers};
}

const api={specs,paths,build};
if(typeof module!=='undefined')module.exports=api;else root.BModels=api;
})(typeof window!=='undefined'?window:globalThis);
