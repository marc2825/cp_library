(function(root){
'use strict';
const specs={
 suffix_automaton:['Suffix Automaton',{text:'abcbc',pattern:'bc'},'各文字の追加で状態・suffix link・cloneがどう変わるかを追います。'],
 lyndon:['Lyndon Factorization',{text:'banana'},'比較位置i/j/kと、非増加順に確定する因子を追います。'],
 common_substring:['Longest Common Substring',{first:'banana',second:'ananas'},'SAMの遷移とsuffix linkを使い、共通する連続区間を伸ばします。'],
 hash_2d:['2D Rolling Hash',{grid:['abcd','efgh','ijkl'],rectangle:[1,3,1,3],row_base:911382323,column_base:972663749},'二つの法のprefixを作り、上下左右の除去と指数の移動を追います。'],
 nim_product:['Nim Product',{a:'16',b:'16'},'小さいnim積のmexと、Fermatの2冪に沿った体の分割を追います。'],
 periodic_grundy:['周期的Grundy数列の検出',{moves:[1,3,4],terms:28,index:'18446744073709551615'},'観測列の候補と、最大手M個の窓一致で証明した周期を区別します。'],
 zobrist:['Zobrist Hash',{board:[0,1,2],values:4,seed:'42',changes:[[1,3],[1,1]]},'同じ位置のoldをXORで取り消し、newを加える差分を追います。'],
 tabu:['Tabu Search',{scores:[4,3,5,1],edges:[[1,2],[0],[3],[2]],start:0,tenure:2,steps:6,goal:'minimize'},'禁止期限・aspiration・悪化手・全体最良を追います。'],
 beam_unique:['Beam Searchの重複除去',{scores:[0,1,2,3],edges:[[1,1,2,3],[],[],[]],start:0,width:3,depth:1,goal:'minimize'},'score順にkeyの代表を選び、その後にBeam幅を制限します。'],
 incremental:['スコア差分更新とapply/undo',{state:4,target:0,moves:[-1,2,-2,3],rule:'improve'},'状態を一時更新し、棄却した手をundoしてscoreも戻します。']
};
const paths={suffix_automaton:'string_algorithm/suffix_automaton',lyndon:'string_algorithm/lyndon_factorization',common_substring:'string_algorithm/longest_common_substring',hash_2d:'string_algorithm/rolling_hash_2d',nim_product:'game_theory/nim_product',periodic_grundy:'game_theory/periodic_grundy',zobrist:'heuristic_search/zobrist_hash',tabu:'heuristic_search/tabu_search',beam_unique:'heuristic_search/beam_search_unique',incremental:'heuristic_search/incremental_score'};
const clone=x=>JSON.parse(JSON.stringify(x,(_,v)=>typeof v==='bigint'?v.toString():v));
const integer=(x,lo,hi,label)=>{if(!Number.isSafeInteger(x)||x<lo||x>hi)throw Error(`${label}: ${lo}〜${hi}の整数を指定してください`);return x;};
const numbers=(a,max,lo=-100,hi=100)=>{if(!Array.isArray(a)||a.length>max)throw Error(`配列は${max}項以下です`);return a.map(x=>integer(x,lo,hi,'値'));};
const bytes=(s,max=16)=>typeof s==='string'?numbers([...new TextEncoder().encode(s)],max,0,255):numbers(s,max,0,255);
const u64=x=>{if(!(typeof x==='string'&&/^\d+$/.test(x))&&!(typeof x==='number'&&Number.isSafeInteger(x)&&x>=0))throw Error('64bit値は非負の整数または10進文字列です');const n=BigInt(x);if(n<0n||n>0xffffffffffffffffn)throw Error('uint64の範囲外です');return n;};
const MASK=0xffffffffffffffffn;
function random64(seed){let state=seed;return()=>{state=(state+0x9e3779b97f4a7c15n)&MASK;let x=state;x=((x^(x>>30n))*0xbf58476d1ce4e5b9n)&MASK;x=((x^(x>>27n))*0x94d049bb133111ebn)&MASK;return x^(x>>31n);};}
function sam(text,onStep=()=>{}){
 const states=[{length:0,link:-1,first_end:-1,clone:false,next:{}}],prefixes=[];let last=0;
 for(let index=0;index<text.length;index++){
  const c=text[index],cur=states.length;states.push({length:states[last].length+1,link:0,first_end:index,clone:false,next:{}});let p=last;
  onStep('新しい末尾状態',`byte ${c}、状態${cur}`,states,{current:cur,prefix:index});
  while(p>=0&&states[p].next[c]===undefined){states[p].next[c]=cur;onStep('不足する遷移を追加',`${p} → ${cur}、byte ${c}`,states,{current:cur,source:p,prefix:index});p=states[p].link;}
  if(p>=0){const q=states[p].next[c];if(states[p].length+1===states[q].length)states[cur].link=q;else{
   const id=states.length;states.push({...clone(states[q]),length:states[p].length+1,clone:true});onStep('状態をclone',`状態${q}を${id}へ複製し、長さを${states[id].length}にする`,states,{current:id,prefix:index});
   while(p>=0&&states[p].next[c]===q){states[p].next[c]=id;p=states[p].link;}states[q].link=states[cur].link=id;
  }}last=cur;prefixes.push(cur);onStep('文字の追加完了',`${index+1}文字のSAM`,states,{current:cur,complete:true,prefix:index+1});
 }
 return{states,prefixes};
}
function build(mode,input){
 if(!Object.hasOwn(specs,mode)||!input||typeof input!=='object'||Array.isArray(input))throw Error('入力オブジェクトとモードを確認してください');
 const frames=[];let result;
 const emit=(title,note,grid=[],active=[],extra={})=>frames.push(clone({title,note,grid,active,columns:['行','内部状態'],rows:grid.length?grid.map((row,i)=>[i,row.join(' ')]):[['状態',note]],...extra}));
 function graph(states,active=[]){return{n:states.length,directed:true,labels:states.map((s,i)=>`${i}:${s.length}`),edges:states.flatMap((s,i)=>[...Object.entries(s.next).map(([c,v])=>[i,v,Number(c)]),...(s.link>=0?[[i,s.link,'link']]:[])]),activeVertices:active};}
 function samFrame(title,note,states,meta){emit(title,note,states.map((s,i)=>[i,s.length,s.link,s.first_end,+s.clone]),[],{...meta,states,graph:graph(states,[meta.current]),columns:['状態','len / link / first_end / clone'],rows:states.map((s,i)=>[i,`${s.length} / ${s.link} / ${s.first_end} / ${+s.clone}`])});}
 if(mode==='suffix_automaton'){
  const text=bytes(input.text),pattern=bytes(input.pattern??'',8),{states,prefixes}=sam(text,samFrame);const counts=states.map(()=>0);prefixes.forEach(v=>counts[v]++);const order=states.map((s,i)=>i).sort((i,j)=>states[j].length-states[i].length);for(const v of order)if(states[v].link>=0)counts[states[v].link]+=counts[v];
  let v=0;for(const c of pattern){if(states[v].next[c]===undefined){v=-1;break;}v=states[v].next[c];}
  result={distinct_substrings:states.slice(1).reduce((n,s)=>n+s.length-states[s.link].length,0),contains:v>=0,count_occurrences:!pattern.length?text.length+1:v>=0?counts[v]:0,first_occurrence:!pattern.length?0:v>=0?states[v].first_end-pattern.length+1:null};
  emit('出現回数をsuffix linkへ集約','各prefixの末尾から、長さの降順に親へ加える',states.map((s,i)=>[i,counts[i]]),[],{states,counts,graph:graph(states,v>=0?[v]:[])});
 }else if(mode==='lyndon'){
  const s=bytes(input.text);result=[];let i=0;
  while(i<s.length){let j=i+1,k=i;while(j<s.length&&s[k]<=s[j]){emit('文字を比較',`i=${i}, j=${j}, k=${k}。${s[k]} ${s[k]===s[j]?'=':'<'} ${s[j]}`,[s],[[0,k],[0,j]],{i,j,k});k=s[k]<s[j]?i:k+1;j++;}
   const length=j-k;while(i<=k){result.push([i,i+length]);emit('因子を確定',`[${i},${i+length})、因子長${length}`,[s],Array.from({length},(_,x)=>[0,i+x]),{factors:result,i,j,k});i+=length;}
  }
 }else if(mode==='common_substring'){
  const a=bytes(input.first),b=bytes(input.second),{states}=sam(a);let v=0,length=0;result={first_left:0,second_left:0,length:0};
  for(let i=0;i<b.length;i++){
   const c=b[i];while(v&&states[v].next[c]===undefined){v=states[v].link;length=states[v].length;emit('suffix linkへ戻る',`byte ${c}の遷移がないため長さを${length}へ`,[a,b],[[1,i]],{state:v,length,graph:graph(states,[v])});}
   if(states[v].next[c]===undefined){v=0;length=0;}else{v=states[v].next[c];length++;if(length>result.length)result={first_left:states[v].first_end+1-length,second_left:i+1-length,length};}
   emit('次のbyteを処理',`second[${i}]=${c}、一致長${length}、最良${result.length}`,[a,b],[[1,i]],{state:v,length,result,graph:graph(states,[v])});
  }
 }else if(mode==='hash_2d'){
  if(!Array.isArray(input.grid)||input.grid.length>4)throw Error('グリッドは4行以下です');const a=input.grid.map(row=>bytes(row,6)),h=a.length,w=a[0]?.length??0;if(a.some(row=>row.length!==w))throw Error('グリッドの幅を揃えてください');
  const rb=BigInt(integer(input.row_base??911382323,257,1000000006,'row_base')),cb=BigInt(integer(input.column_base??972663749,257,1000000006,'column_base')),mods=[1000000007n,1000000009n];
  const rect=numbers(input.rectangle,4,0,6);if(rect.length!==4)throw Error('rectangleはtop,bottom,left,rightの4値です');const[t,b,l,r]=rect;if(t>b||b>h||l>r||r>w)throw Error('長方形の範囲外です');
  const p=mods.map(()=>Array.from({length:h+1},()=>Array(w+1).fill(0n)));
  for(let i=0;i<h;i++)for(let j=0;j<w;j++){for(let k=0;k<2;k++){const m=mods[k],x=p[k];x[i+1][j+1]=(BigInt(a[i][j]+1)+rb*x[i][j+1]+cb*x[i+1][j]-rb*cb%m*x[i][j]+m*m)%m;}emit('prefix矩形を更新',`(${i+1},${j+1})、行baseと列baseを掛けて重複を除く`,p[0],[[i+1,j+1]],{prefix:p,row:i+1,column:j+1});}
  const pow=(x,n,m)=>{let r=1n;for(let i=0;i<n;i++)r=r*x%m;return r;};const terms=mods.map((m,k)=>{const x=p[k],rp=pow(rb,b-t,m),cp=pow(cb,r-l,m);return[x[b][r],x[t][r]*rp%m,x[b][l]*cp%m,x[t][l]*rp%m*cp%m];});result=terms.map((v,k)=>(v[0]-v[1]-v[2]+v[3]+2n*mods[k])%mods[k]);emit('上と左を除き、左上を戻す','P(bottom,right) − 上 − 左 + 左上',terms,[],{terms});
 }else if(mode==='nim_product'){
  const a=u64(input.a),b=u64(input.b);const table=Array.from({length:16},()=>Array(16).fill(0));for(let x=0;x<16;x++)for(let y=0;y<16;y++){const seen=new Set();for(let i=0;i<x;i++)for(let j=0;j<y;j++)seen.add(table[i][y]^table[x][j]^table[i][j]);let n=0;while(seen.has(n))n++;table[x][y]=n;}
  function nim(x,y,bits,depth){if(!x||!y)return 0n;if(x<16n&&y<16n)return BigInt(table[Number(x)][Number(y)]);const h=bits/2,mask=(1n<<BigInt(h))-1n,x0=x&mask,x1=x>>BigInt(h),y0=y&mask,y1=y>>BigInt(h);if(depth<3)emit('体の二次拡大で分割',`w=${bits}、X=2^${h}、X²=X XOR 2^${h-1}`,[[x0,x1],[y0,y1]],[],{bits,depth});const lo=nim(x0,y0,h,depth+1),hi=nim(x1,y1,h,depth+1),upper=nim(x0^x1,y0^y1,h,depth+1)^lo,lower=lo^nim(hi,1n<<BigInt(h-1),h,depth+1),answer=lower^(upper<<BigInt(h));if(depth<3)emit('下位と上位を組み立てる',`下位=${lower}、上位=${upper}`,[[lower,upper],[answer]],[],{bits,depth});return answer;}
  result=nim(a,b,64,0).toString();if(a<16n&&b<16n){const forbidden=Array.from({length:Number(a)},(_,x)=>Array.from({length:Number(b)},(_,y)=>table[x][Number(b)]^table[Number(a)][y]^table[x][y]));emit('mexの定義を確認','行a′・列b′の到達値を集め、ない最小非負整数を取る',forbidden,[],{forbidden});}
 }else if(mode==='periodic_grundy'){
  const observed=input.kind==='observed';let values,moves=[];if(observed)values=numbers(input.values,64,0,64);else{
   moves=[...new Set(numbers(input.moves,12,1,1000))].sort((a,b)=>a-b);const n=integer(input.terms,1,64,'terms');values=Array(n).fill(0);emit('g(0)=0','正の手がない0個の局面は終局',[[0,0]]);
   for(let i=1;i<n;i++){const reach=moves.filter(d=>d<=i).map(d=>values[i-d]),set=new Set(reach);let x=0;while(set.has(x))x++;values[i]=x;emit('mexで次のGrundy値',`g(${i})=mex{${reach.join(',')}}=${x}`,[values.slice(0,i+1)],[[0,i]],{index:i,reach});}
  }
  const reversed=[...values].reverse(),z=reversed.map((_,p)=>{let x=0;while(p+x<reversed.length&&reversed[x]===reversed[p+x])x++;return x;});emit('逆順のZ値','z[p]はpだけずらした末尾の一致長',[reversed,z],[],{z});
  const repetitions=observed?integer(input.repetitions??3,2,64,'repetitions'):0,cap=observed?integer(input.max_period??values.length,0,64,'max_period'):values.length;let period=null;
  for(let p=1;p<values.length&&p<=cap;p++){const start=values.length-p-z[p],enough=observed?Math.floor((values.length-start)/p)>=repetitions:z[p]>=(moves.at(-1)??0);if(enough&&(!period||start<period.preperiod)){period={preperiod:start,period:p};emit(observed?'観測された反復候補':'同じM窓から周期を証明',`preperiod=${start}、period=${p}、一致長${z[p]}`,[values],[],{candidate:period,observed});}}
  if(!observed&&!moves.length)period={preperiod:0,period:1};
  if(observed)result=period;else{const index=u64(input.index??'100'),pos=index<BigInt(values.length)?Number(index):period?period.preperiod+Number((index-BigInt(period.preperiod))%BigInt(period.period)):-1;result={values,period,value:pos<0?null:values[pos]};}
 }else if(mode==='zobrist'){
  const board=numbers(input.board,8,0,5),values=integer(input.values,1,6,'values');if(board.some(x=>x>=values))throw Error('盤面値がvalues以上です');if(!Array.isArray(input.changes)||input.changes.length>12)throw Error('changesは12手以下です');const raw=random64(u64(input.seed??'0')),tokens=board.map(()=>Array.from({length:values},()=>raw()));let hash=board.reduce((x,v,i)=>x^tokens[i][v],0n);emit('位置×値の乱数表','0も通常の値としてtokenを持つ',tokens,[],{tokens,hash,board});
  for(const change of input.changes){const v=numbers(change,2,0,7);if(v.length!==2)throw Error('変更は[position,value]です');const[pos,next]=v;if(pos>=board.length||next>=values)throw Error('変更の範囲外です');const old=board[pos];hash^=tokens[pos][old];emit('oldのtokenを取り消す',`位置${pos}、値${old}のtokenをXOR`,[board],[[0,pos]],{hash,phase:'remove',tokens,board});hash^=tokens[pos][next];board[pos]=next;emit('newのtokenを加える',`位置${pos}を${old}から${next}へ`,[board],[[0,pos]],{hash,phase:'add',tokens,board});}result={hash:hash.toString(),board};
 }else if(mode==='tabu'||mode==='beam_unique'){
  const scores=numbers(input.scores,8,-1000,1000),n=scores.length;if(!n)throw Error('局面が1個以上必要です');if(!Array.isArray(input.edges)||input.edges.length!==n)throw Error('edgesの行数がscoresと違います');const edges=input.edges.map(row=>numbers(row,12,0,n-1)),keys=input.keys===undefined?scores.map((_,i)=>i):numbers(input.keys,8,-100,100);if(keys.length!==n)throw Error('keysの長さが違います');const start=integer(input.start,0,n-1,'start'),goal=input.goal??'minimize';if(!['minimize','maximize'].includes(goal))throw Error('goalはminimize/maximizeです');const better=(a,b)=>goal==='minimize'?a<b:a>b,diagram={n,directed:true,labels:scores.map((s,i)=>`${i}:${s}`),edges:edges.flatMap((row,i)=>row.map(j=>[i,j]))};
  if(mode==='beam_unique'){
   const width=integer(input.width,1,8,'width'),depthLimit=integer(input.depth,0,12,'depth');let frontier=[start],depth=0,evaluations=1,termination='depth_limit';emit('初期frontier',`幅${width}、深さごとに重複を除く`,[frontier],[],{graph:diagram});
   for(let step=0;step<depthLimit;step++){const children=frontier.flatMap(v=>edges[v]);evaluations+=children.length;if(!children.length){termination='exhausted';break;}const sorted=children.map((v,i)=>({v,i})).sort((a,b)=>scores[a.v]===scores[b.v]?a.i-b.i:better(scores[a.v],scores[b.v])?-1:1);emit('全候補をscore順に安定ソート','同じkeyでも最良scoreの候補を先に調べる',sorted.map(({v})=>[v,scores[v],keys[v]]),[],{graph:diagram,candidates:children});const seen=new Set(),kept=[];for(const{v}of sorted)if(!seen.has(keys[v])){seen.add(keys[v]);kept.push(v);if(kept.length===width)break;}frontier=kept;depth++;emit('keyごとに代表を残す','重複除去の後に幅を制限',[frontier],[],{graph:{...diagram,activeVertices:frontier},frontier,depth});}result={frontier,depth,evaluations,termination};
  }else{
   const tenure=integer(input.tenure,0,20,'tenure'),limit=integer(input.steps,0,12,'steps'),until=new Map();if(tenure)until.set(keys[start],tenure);let current=start,best=start,iterations=0,evaluations=1,accepted=0,termination='iteration_limit';emit('初期状態',`初期keyを最初の${tenure}走査で禁止`,[[current,scores[current]]],[],{graph:diagram});
   for(let step=0;step<limit;step++){for(const[key,end]of until)if(end<=step)until.delete(key);let chosen=null;const rows=[];for(const child of edges[current]){evaluations++;const tabu=(until.get(keys[child])??0)>step,aspiration=better(scores[child],scores[best]),allowed=!tabu||aspiration;rows.push([child,scores[child],keys[child],until.get(keys[child])??0,+tabu,+aspiration,+allowed]);if(allowed&&(chosen===null||better(scores[child],scores[chosen])))chosen=child;}iterations++;emit('候補と禁止期限を調べる',`走査${step}、列: 頂点/score/key/until/tabu/aspiration/許可`,rows,[],{graph:{...diagram,activeVertices:[current]},step,current,best,until:[...until]});if(chosen===null){termination='exhausted';break;}current=chosen;accepted++;if(better(scores[current],scores[best]))best=current;if(tenure)until.set(keys[current],step+tenure+1);emit('手を採用する',`現在${current}、最良${best}。次の${tenure}走査は同じkeyを禁止`,[[current,scores[current]],[best,scores[best]]],[],{graph:{...diagram,activeVertices:[current,best]},current,best,step,until:[...until]});}result={best_state:best,current_state:current,best_score:scores[best],current_score:scores[current],iterations,evaluations,accepted,termination};
  }
 }else if(mode==='incremental'){
  let state=integer(input.state,-100,100,'state');const target=integer(input.target,-100,100,'target'),moves=numbers(input.moves,20,-20,20);let score=(state-target)**2,accepted=0;if(input.accept!==undefined&&(!Array.isArray(input.accept)||input.accept.length!==moves.length||input.accept.some(x=>typeof x!=='boolean')))throw Error('acceptはmovesと同じ長さのtrue/false列です');if(input.rule!==undefined&&input.rule!=='improve')throw Error('ruleはimproveです');emit('初期の全評価',`state=${state}、score=(state−${target})²=${score}`,[[state,score]]);
  for(let i=0;i<moves.length;i++){const move=moves[i],oldState=state,oldScore=score,delta=2*(state-target)*move+move*move;state+=move;score+=delta;emit('applyと差分スコア',`move=${move}、delta=${delta}。まだ採否は未確定`,[[oldState,oldScore],[state,score]],[[1,0],[1,1]],{temporary:true,state,score,move,delta});const keep=input.accept?input.accept[i]:score<oldScore;if(keep)accepted++;else{state=oldState;score=oldScore;}emit(keep?'採用':'棄却してundo',keep?'現在解として保持する':'状態とscoreを元へ戻す',[[state,score]],[],{state,score,accepted,temporary:false});}result={state,score,accepted};
 }
 if(!frames.length)emit('初期状態',specs[mode][2]);emit('完了',JSON.stringify(clone(result)),[],[],{result});return{frames,result:clone(result)};
}
const api={specs,paths,build};if(typeof module!=='undefined')module.exports=api;root.FinalAModels=api;
})(typeof globalThis!=='undefined'?globalThis:this);
