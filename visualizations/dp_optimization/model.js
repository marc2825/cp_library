(function(root){
'use strict';
const MODES=['lis','lcs','edit','zero-one','unbounded','bounded','bitset','interval','digit','subset','rectangle','grid'];
const copy=x=>JSON.parse(JSON.stringify(x));
function integer(x,lo,hi,label){const n=Number(x);if(String(x).trim()===''||!Number.isSafeInteger(n)||n<lo||n>hi)throw new Error(`${label}は ${lo}〜${hi} の整数にしてください。`);return n;}
function numbers(input,max=12,lo=-30,hi=30){const values=typeof input==='string'?input.trim().split(/[\s,]+/).filter(Boolean):input||[];if(values.length>max)throw new Error(`配列は${max}要素以内にしてください。`);return values.map(x=>integer(x,lo,hi,'配列の値'));}
function lines(input){return String(input??'').trim().split('\n').filter(x=>x.trim()).map(x=>x.trim().split(/\s+/));}
function normalize(input){
 const mode=input.mode||'lis';if(!MODES.includes(mode))throw new Error('未対応のモードです。');const config={mode};
 if(['lis','interval','rectangle','bitset'].includes(mode))config.a=numbers(input.a,mode==='interval'?8:12,mode==='rectangle'||mode==='bitset'?0:-30,mode==='bitset'?200:mode==='interval'?50:30);
 if(mode==='lis'){config.strict=input.strict!==false;config.decreasing=!!input.decreasing;}
 if(mode==='lcs'||mode==='edit'){config.a=String(input.a??'');config.b=String(input.b??'');if([config.a,config.b].some(s=>s.length>10||/[^\x20-\x7e]/.test(s)))throw new Error('文字列の図解はASCII文字10文字以内にしてください。');}
 if(['zero-one','unbounded','bounded'].includes(mode)){
  let items=typeof input.items==='string'?lines(input.items):input.items||[];if(items.length>6)throw new Error('品物は6種類以内にしてください。');
  config.items=items.map(item=>{if(item.length!==(mode==='bounded'?3:2))throw new Error(mode==='bounded'?'品物は 重さ 価値 個数 の3個です。':'品物は 重さ 価値 の2個です。');return {weight:integer(item[0],mode==='unbounded'?1:0,24,'重さ'),value:integer(item[1],-20,30,'価値'),count:mode==='bounded'?integer(item[2],0,8,'個数'):1};});
 }
 if(['zero-one','unbounded','bounded','bitset'].includes(mode))config.capacity=integer(input.capacity??10,0,mode==='bitset'?192:24,'容量');
 if(mode==='digit'){config.upper=String(input.upper??'314');if(!/^\d{1,6}$/.test(config.upper))throw new Error('上限は1〜6桁の十進数字にしてください。');config.divisor=integer(input.divisor??3,1,9,'桁和の法');config.includeZero=!!input.includeZero;}
 if(mode==='grid'){
  let rows=typeof input.grid==='string'?String(input.grid).trim().split('\n').filter(x=>x.trim()).map(x=>/\s/.test(x.trim())?x.trim().split(/\s+/):[...x.trim()]):input.grid||[];
  if(rows.length>6||rows.some(row=>row.length>8))throw new Error('グリッドは6行8列以内にしてください。');if(rows.some(row=>row.length!==(rows[0]?.length??0)))throw new Error('各行の列数を揃えてください。');config.grid=rows.map(row=>row.map(x=>integer(Number(x),0,1,'マス')));
 }
 if(mode==='subset'){
  config.kind=input.kind||'tsp';if(!['tsp','zeta','mobius','submasks'].includes(config.kind))throw new Error('未対応の部分集合操作です。');
  if(config.kind==='tsp'){
   const matrix=typeof input.matrix==='string'?lines(input.matrix):input.matrix||[];const n=matrix.length;if(n>6||matrix.some(row=>row.length!==n))throw new Error('費用は6頂点以内の正方行列にしてください。');
   config.matrix=matrix.map(row=>row.map(x=>x===null||x==='-'?null:integer(x,-20,30,'辺の費用')));config.start=n?integer(input.start??0,0,n-1,'始点'):0;
  }else if(config.kind==='submasks')config.mask=integer(input.mask??13,0,31,'mask');
  else{config.a=numbers(input.a,32,-30,30);if(!config.a.length||(config.a.length&(config.a.length-1)))throw new Error('変換する配列の長さは正の2冪にしてください。');}
 }
 return config;
}
function run(input){
 const config=normalize(input),{mode}=config,frames=[];let state={phase:'準備',focus:[],cells:[],result:null,extra:'',rule:''};
 const save=(title,description,patch={})=>{Object.assign(state,{focus:[],cells:[],extra:'',rule:''},patch);frames.push(copy({...state,title,description}));};
 if(mode==='lis'){
  const a=config.a,tails=[],tailIndices=[],previous=Array(a.length).fill(-1),ending=Array(a.length).fill(null);Object.assign(state,{a,tails,tailIndices,previous,ending,selected:[]});
  save('末尾の最良値を持つ','長さごとに、後続をつなぎやすい末尾を保持します。tails自体が元の配列の部分列になるとは限りません。');
  const cmp=(x,y)=>config.decreasing?x>y:x<y;
  for(let i=0;i<a.length;++i){let lo=0,hi=tails.length;while(lo<hi){const mid=(lo+hi)>>1;save('挿入位置を二分探索',`a[${i}]=${a[i]} と tails[${mid}]=${tails[mid]} を比較します。`,{phase:'二分探索',focus:[i],lo,hi,mid});if(config.strict?cmp(tails[mid],a[i]):!cmp(a[i],tails[mid]))lo=mid+1;else hi=mid;}
   if(lo)previous[i]=tailIndices[lo-1];ending[i]=lo+1;tails[lo]=a[i];tailIndices[lo]=i;
   save('末尾を更新',`長さ ${lo+1} の末尾を ${a[i]} にします。`,{phase:'更新',focus:[i],lo,hi:lo,mid:lo,rule:config.strict?'strict: lower_bound':'non-strict: upper_bound'});
  }
  for(let v=tailIndices.at(-1)??-1;v!==-1;v=previous[v]){state.selected.unshift(v);save('前の要素をたどる','前駆の添字を逆にたどって、一つの最長部分列を復元します。',{phase:'復元',focus:[v]});}
  save('最長部分列が確定','同じ長さの解が複数あるときの辞書順は保証しません。',{phase:'完了',result:tails.length,extra:`indices: ${state.selected.join(', ')} / values: ${state.selected.map(i=>a[i]).join(', ')}`});
 }else if(mode==='lcs'||mode==='edit'){
  const {a,b}=config,n=a.length,m=b.length,table=Array.from({length:n+1},()=>Array(m+1).fill(null));
  for(let i=0;i<=n;++i)table[i][0]=mode==='edit'?i:0;for(let j=0;j<=m;++j)table[0][j]=mode==='edit'?j:0;
  Object.assign(state,{a,b,table,path:[],matches:[],operations:[]});save('空の接頭辞を初期化','行はAの先頭i文字、列はBの先頭j文字を表します。');
  for(let i=1;i<=n;++i)for(let j=1;j<=m;++j){const same=a[i-1]===b[j-1];table[i][j]=mode==='lcs'?Math.max(table[i-1][j],table[i][j-1],same?table[i-1][j-1]+1:0):Math.min(table[i-1][j]+1,table[i][j-1]+1,table[i-1][j-1]+(same?0:1));save(`dp[${i}][${j}] を確定`,mode==='lcs'?'上・左の答えと、一致する場合の対角からの候補を比べます。':'上は削除、左は挿入、対角は一致／置換です。',{phase:'表の計算',cells:[[i,j],[i-1,j],[i,j-1],[i-1,j-1]],active:[i,j],rule:mode==='lcs'?'max(上, 左, 一致なら対角+1)':'min(上+1, 左+1, 対角+不一致)'});}
  let i=n,j=m;state.path.push([i,j]);
  while(i||j){let operation;
   if(mode==='lcs'){
    if(i&&j&&a[i-1]===b[j-1]&&table[i][j]===table[i-1][j-1]+1){state.matches.unshift([i-1,j-1]);--i;--j;operation='一致する文字を採用';}else if(i&&(!j||table[i-1][j]>=table[i][j-1])){--i;operation='Aの末尾を使わない';}else{--j;operation='Bの末尾を使わない';}
   }else{
    if(i&&j&&table[i][j]===table[i-1][j-1]+(a[i-1]===b[j-1]?0:1)){const kind=a[i-1]===b[j-1]?'Keep':'Replace';--i;--j;state.operations.unshift({kind,i,j});operation=kind;}
    else if(i&&table[i][j]===table[i-1][j]+1){--i;state.operations.unshift({kind:'Erase',i,j});operation='Erase';}
    else{--j;state.operations.unshift({kind:'Insert',i,j});operation='Insert';}
   }
   state.path.push([i,j]);save(operation,'最適値を作った遷移を逆にたどります。操作列は最後に元の向きへ揃えます。',{phase:'復元',cells:[[i,j]],active:[i,j]});
  }
  save(mode==='lcs'?'共通部分列を復元':'編集操作を復元',mode==='lcs'?'対応する添字を両方の列の順序で返します。':'Keepは0、挿入・削除・置換は各1です。',{phase:'完了',result:table[n][m],extra:mode==='lcs'?state.matches.map(([i])=>a[i]).join(''):state.operations.map(op=>op.kind).join(' → ')});
 }else if(['zero-one','unbounded','bounded'].includes(mode)){
  const {items,capacity}=config,dp=Array(capacity+1).fill(null);dp[0]=0;Object.assign(state,{dp,old:dp.slice(),queue:[],item:-1,processed:0,position:null});save('重さ0だけに到達','dp[w] は重さちょうどwでの最大価値。— は未到達です。');
  for(let index=0;index<items.length;++index){const item=items[index];state.item=index;state.old=dp.slice();state.queue=[];state.position=null;save(`品物 ${index} を処理`,`重さ ${item.weight}、価値 ${item.value}${mode==='bounded'?`、上限 ${item.count} 個`:''}。`,{phase:'品物の開始',rule:mode==='zero-one'?'重さを大きい側から更新':mode==='unbounded'?'重さを小さい側から更新':'重さの剰余ごとに、個数制限の窓を保つ'});
   if(mode!=='bounded'){
    const order=Array.from({length:Math.max(0,capacity-item.weight+1)},(_,i)=>mode==='zero-one'?capacity-i:item.weight+i);
    for(const w of order){state.position=w;const source=w-item.weight;if(dp[source]!==null){const candidate=dp[source]+item.value;dp[w]=dp[w]===null?candidate:Math.max(dp[w],candidate);save(`重さ ${w} を更新`,mode==='zero-one'?'更新前の行から1個だけ追加します。':'同じ品物で更新した小さい重さを再利用できます。',{phase:'重さの更新',focus:[source,w],source,target:w,candidate});}}
   }else if(item.weight===0){const gain=Math.max(0,item.value)*item.count;for(let w=0;w<=capacity;++w)if(dp[w]!==null)dp[w]+=gain;save('重さ0の品物をまとめる','価値が正なら上限まで、0以下なら0個を選びます。',{phase:'重さ0',gain});}
   else if(item.count&&item.weight<=capacity){
    for(let r=0;r<item.weight&&r<=capacity;++r){const queue=[];state.queue=queue;state.residue=r;
     for(let t=0;t<=Math.floor((capacity-r)/item.weight);++t){const w=r+t*item.weight;state.position=w;state.t=t;
      while(queue.length&&t-queue[0].t>item.count){queue.shift();save('期限切れの候補を除く','t−上限個数より前の候補は使えません。',{phase:'候補キュー',focus:[w]});}
      const score=c=>c.value+(t-c.t)*item.value;
      if(state.old[w]!==null){while(queue.length&&score(queue.at(-1))<=state.old[w]){queue.pop();save('劣る末尾候補を除く','新しい候補のほうが価値が高く、期限も遅いので、古い末尾は不要です。',{phase:'候補キュー',focus:[w]});}queue.push({t,value:state.old[w]});}
      dp[w]=queue.length?score(queue[0]):null;save(`剰余 ${r}、位置 ${t} を確定`,'キュー先頭が有効範囲で最もよい候補です。',{phase:'重さの更新',focus:[w],target:w,scores:queue.map(score),rule:`old[r+jw] + (t−j)×value / t−count ≤ j ≤ t`});
     }
    }
   }
   state.processed=index+1;state.position=null;save(`品物 ${index} が完了`,'次の品物へ進む前の、全ての重さの答えです。',{phase:'品物の完了'});
  }
  save('容量以下の最良値','空集合も選べるので、容量以下の答えは0以上です。負の値も「ちょうどの重さ」の状態には残ります。',{phase:'完了',result:Math.max(...dp.filter(x=>x!==null))});
 }else if(mode==='bitset'){
  const {a,capacity}=config,mask=(1n<<BigInt(capacity+1))-1n;let bits=1n;Object.assign(state,{before:[],shifted:[],bits:Array.from({length:capacity+1},(_,i)=>i===0?1:0),processed:0,item:-1});
  save('0だけを到達可能にする','ビットsが1なら、重さの合計sを作れます。');
  const array=value=>Array.from({length:capacity+1},(_,s)=>Number(value>>BigInt(s)&1n));
  for(let i=0;i<a.length;++i){const shifted=(bits<<BigInt(a[i]))&mask;state.before=array(bits);state.shifted=array(shifted);bits=(bits|shifted)&mask;state.bits=array(bits);state.processed=i+1;state.item=i;save(`重さ ${a[i]} を1個追加`,'追加前のビット列を左シフトし、元の列とORします。更新後の列を繰り返し参照しません。',{phase:'shift-OR',rule:'reachable |= reachable << weight',extra:'C++は64ビット単位。図解では同じ集合演算をBigIntで表します。'});}
  const sums=state.bits.flatMap((bit,s)=>bit?[s]:[]);save('到達可能な和','上限を超えるビットは切り落とします。重さ0でも集合は変わりません。',{phase:'完了',result:sums.length,extra:sums.join(', ')});
 }else if(mode==='interval'){
  const a=config.a,n=a.length,prefix=[0],table=Array.from({length:n+1},()=>Array(n+1).fill(null)),split=Array.from({length:n+1},()=>Array(n+1).fill(-1));a.forEach(x=>prefix.push(prefix.at(-1)+x));
  for(let i=0;i<=n;++i){table[i][i]=0;if(i<n)table[i][i+1]=0;}Object.assign(state,{a,table,split,merges:[]});save('長さ0・1は結合不要','隣接する区間をまとめる費用が区間内の重さの総和、という例です。');
  for(let length=2;length<=n;++length)for(let l=0;l+length<=n;++l){const r=l+length;for(let k=l+1;k<r;++k){const candidate=table[l][k]+table[k][r]+prefix[r]-prefix[l];if(table[l][r]===null||candidate<table[l][r]){table[l][r]=candidate;split[l][r]=k;}save(`[${l},${r}) を ${k} で分割`,'左右の短い区間の最適値に、今回の結合費用を足します。',{phase:'区間の計算',cells:[[l,r],[l,k],[k,r]],active:[l,r],interval:[l,k,r],candidate,rule:`${table[l][k]} + ${table[k][r]} + ${prefix[r]-prefix[l]} = ${candidate}`});}}
  function restore(l,r){const k=split[l][r];if(k<0)return;restore(l,k);restore(k,r);state.merges.push([l,k,r]);}restore(0,n);
  save('最小費用と結合順','splitから子の結合を先に並べて復元します。',{phase:'完了',result:table[0][n],extra:state.merges.map(([l,k,r])=>`[${l},${k})+[${k},${r})`).join(' → ')});
 }else if(mode==='digit'){
  const {upper,divisor}=config,zero=()=>Array.from({length:divisor},()=>[[0,0],[0,0]]);let dp=zero();dp[0][1][0]=1;Object.assign(state,{dp,next:zero(),position:0,expanded:[],transitions:[],completedDigits:0});save('上限一致・未開始で1通り','左から桁を選びます。開始前の0は桁和を変えず、整数0は最後まで未開始です。');
  for(let position=0;position<upper.length;++position){const limit=Number(upper[position]),next=zero();Object.assign(state,{position,dp,next,expanded:[],transitions:[]});
   for(let s=0;s<divisor;++s)for(let tight=0;tight<2;++tight)for(let started=0;started<2;++started){if(!dp[s][tight][started])continue;const transitions=[];for(let digit=0;digit<=(tight?limit:9);++digit){const to=(s+digit)%divisor,nt=Number(tight&&digit===limit),ns=Number(started||digit!==0);next[to][nt][ns]+=dp[s][tight][started];transitions.push({digit,to,tight:nt,started:ns,ways:dp[s][tight][started]});}state.expanded.push([s,tight,started]);state.transitions=transitions;save(`桁 ${position+1}: 状態 ${s}/${tight}/${started}`,'上限一致の間だけ今回の上限桁まで。それ以外は0〜9を選べます。',{phase:'桁の遷移',activeState:[s,tight,started],rule:'next_tight = tight && digit == limit / next_started = started || digit != 0'});}
   dp=next;state.dp=dp;state.next=zero();state.completedDigits=position+1;state.expanded=[];save(`${position+1} 桁が完了`,'次の桁へは完成した行だけを渡します。',{phase:'桁の完了'});
  }
  const answer=dp[0][0][1]+dp[0][1][1]+(config.includeZero?dp[0][0][0]+dp[0][1][0]:0);save('条件を満たす整数を数える','桁和の余りが0の状態を合計し、0を含めるかを開始フラグで選びます。',{phase:'完了',result:answer});
 }else if(mode==='subset'){
  if(config.kind==='submasks'){state.submasks=[];save('maskに含まれる部分集合','(sub−1)&maskで、使えるビットだけを残して次へ進みます。');for(let sub=config.mask;;sub=(sub-1)&config.mask){state.submasks.push(sub);save(`sub = ${sub}`,'降順に列挙します。0を処理したら終了し、最初へ戻るのを防ぎます。',{phase:'部分集合列挙',mask:config.mask,sub,rule:`${sub.toString(2)} ⊆ ${config.mask.toString(2)}`});if(!sub)break;}save('列挙完了','空集合も含めた件数です。',{phase:'完了',result:state.submasks.length});
  }else if(config.kind!=='tsp'){
   const values=config.a.slice();state.values=values;save('部分集合ごとの値','添字maskを集合として扱います。');for(let bit=1;bit<values.length;bit<<=1)for(let mask=0;mask<values.length;++mask)if(mask&bit){values[mask]+=config.kind==='zeta'?values[mask^bit]:-values[mask^bit];save(`bit ${Math.log2(bit)} / mask ${mask}`,config.kind==='zeta'?'このビットを持たない部分集合から加算します。':'このビットを持たない状態を引いて逆変換します。',{phase:'集合変換',focus:[mask,mask^bit],bit,mask,rule:config.kind==='zeta'?'f[mask] += f[mask ^ bit]':'f[mask] -= f[mask ^ bit]'});}save('変換完了','各ビットを順に処理するのでO(N 2^N)です。',{phase:'完了',result:values.join(', ')});
  }else{
   const cost=config.matrix,n=cost.length,start=config.start,states=1<<n,table=Array.from({length:states},()=>Array(n).fill(null)),previous=Array.from({length:states},()=>Array(n).fill(-1));state.table=table;state.route=[];
   if(n)table[1<<start][start]=0;save('始点だけを訪問済みにする','dp[mask][v]は、maskの頂点を訪れてvで終わる最小費用。— は未到達です。');
   for(let mask=0;mask<states;++mask)for(let v=0;v<n;++v)if(table[mask][v]!==null)for(let to=0;to<n;++to)if(!(mask>>to&1)&&cost[v][to]!==null){const next=mask|1<<to,candidate=table[mask][v]+cost[v][to];if(table[next][to]===null||candidate<table[next][to]){table[next][to]=candidate;previous[next][to]=v;save(`集合 ${mask} から ${next} へ`,'未訪問の頂点を一つ追加します。同じ頂点を再訪しません。',{phase:'集合の拡張',cells:[[mask,v],[next,to]],active:[next,to],mask:next,from:v,to,candidate,rule:`${table[mask][v]} + ${cost[v][to]} = ${candidate}`});}}
   let answer=null,end=-1;for(let v=0;v<n;++v)if(v!==start&&table[states-1][v]!==null&&cost[v][start]!==null){const value=table[states-1][v]+cost[v][start];if(answer===null||value<answer){answer=value;end=v;}}
   if(n<=1){answer=0;state.route=n?[start]:[];}else if(end!==-1){let mask=states-1,v=end;while(v!==start){state.route.unshift(v);const p=previous[mask][v];mask^=1<<v;v=p;}state.route.unshift(start);state.route.push(start);}
   save('始点へ戻って巡回路を完成',answer===null?'全頂点を訪れて戻る経路がありません。':'最後の頂点から始点への辺も費用に含めます。',{phase:'完了',result:answer===null?'なし':answer,extra:state.route.join(' → ')});
  }
 }else if(mode==='rectangle'||mode==='grid'){
  const grid=mode==='grid'?config.grid:null;let global={area:0,left:0,right:0,height:0,top:0,bottom:0};state.best=global;state.stack=[];state.heights=mode==='rectangle'?config.a:[];state.row=-1;
  save('小さい境界まで幅を伸ばす',mode==='grid'?'0が使えるマス、1が障害物です。行ごとに上向きに続く0の高さを作ります。':'高さの小さい順に候補を保ちます。自分より低い棒が来たときに右端が確定します。');
  const scan=(height,row)=>{const stack=[];state.stack=stack;state.heights=height.slice();state.row=row;state.position=-1;save(row<0?'ヒストグラムを走査':`行 ${row} の高さを作る`,'同じ高さもまとめて、左端を引き継ぎます。',{phase:row<0?'走査開始':'行の開始'});
   for(let i=0;i<=height.length;++i){const current=i===height.length?0:height[i];let left=i;state.position=i;
    while(stack.length&&stack.at(-1).height>=current){const entry=stack.pop(),area=entry.height*(i-entry.left);left=entry.left;const candidate={area,left:entry.left,right:i,height:entry.height,top:row<0?0:row+1-entry.height,bottom:row<0?0:row+1};if(area>global.area){global=candidate;state.best=global;}save('右端が確定した候補を精算',`高さ ${entry.height}、区間 [${entry.left},${i})、面積 ${area}。`,{phase:'候補の精算',candidate,focus:[entry.index,i],rule:`${entry.height} × (${i} − ${entry.left}) = ${area}`});}
    if(i<height.length){stack.push({height:current,left,index:i});save(`棒 ${i} を追加`,'候補は高さが狭義単調増加。各棒は高々1回追加・削除されます。',{phase:'候補の追加',focus:[i]});}
   }
  };
  if(grid){let height=Array(grid[0]?.length??0).fill(0);for(let row=0;row<grid.length;++row){height=height.map((h,j)=>grid[row][j]?0:h+1);scan(height,row);}}else scan(config.a,-1);
  save('最大長方形を確定','同点解の辞書順は保証しません。面積0は空の長方形を返します。',{phase:'完了',result:global.area,extra:mode==='grid'?`[${global.top},${global.bottom}) × [${global.left},${global.right})`:`[${global.left},${global.right}) / height ${global.height}`});
 }
 return {config,frames};
}
const api={MODES,normalize,run};if(typeof module!=='undefined'&&module.exports)module.exports=api;else root.DPModel=api;
})(typeof globalThis!=='undefined'?globalThis:this);
