(function(root){
'use strict';
const specs={
 divide_conquer_dp:['Divide and Conquer DP Optimization',{values:[1,2,3,2,1],groups:3},'区間和の二乗。中央の行の最適分割点から探索範囲を狭めます。'],
 knuth_optimization:['Knuth Optimization',{values:[10,20,30,10]},'非負のSlimes。隣り合う区間のoptの間だけを調べます。'],
 convex_hull_trick_dp:['Convex Hull Trick DP',{heights:[10,30,20,40],constant:10},'二乗距離を直線に変形し、query後に次の直線を追加します。'],
 li_chao_dp:['Li Chao Tree DP',{heights:[10,30,20,40],constant:10},'登録座標のLi Chao Treeで、中央の座標で優れる直線を残します。'],
 slope_trick:['Slope Trick',{operations:[['abs',1],['abs',5],['shift',-1,2],['right',3],['prefix'],['abs',2]]},'左右の折れ点と最小値、現在の凸関数を追います。'],
 aliens_trick:['Aliens Trick / WQS Binary Search',{values:[1,2,3,2],groups:2,low:-100,high:100},'非負列の分割DP。罰則の同値解は区間数が多いものを選びます。'],
 euler_phi:['Euler φ',{limit:16},'SPFからphi(p*q)を更新。単一値のPollard–Rhoは既存図解へ。'],
 mobius:['Möbius μ',{limit:16},'素因数の重複なら0、重複しなければ符号反転します。'],
 crt:['CRT / 一般化中国剰余定理',{residues:[2,8],moduli:[6,9]},'gcdで矛盾を判定し、解の代表元とlcmを更新します。'],
 garner:['Garner',{residues:[2,3,2],moduli:[3,5,7],target:12},'混合基数の桁ごとに、後続の法の係数と定数を更新します。'],
 floor_sum:['Floor Sum',{n:12,m:10,a:-6,b:13},'負値を正規化し、格子点の軸を交換して総和を縮めます。'],
 primitive_root:['Primitive Root',{prime:17},'p-1の各素因数で位数が小さくないかを調べます。'],
 stirling:['Stirling数（第1種・第2種）',{n:6,kind:'second',modulus:97},'kindをfirst / signed / secondへ変え、三角表の漸化式を比較できます。'],
 bell:['Bell数',{n:7,modulus:97},'Bell三角形の行頭が、その要素数の集合分割数です。'],
 partition_numbers:['整数の分割数',{n:12,modulus:97},'一般化五角数の添字へ+,+,-,-の順に戻ります。'],
 derangements:['Montmort数 / Derangements',{n:8,modulus:97},'固定点のない順列を、2-cycleと長いcycleへの挿入に分けます。'],
 burnside:['Burnsideの補題',{fixed:[8,2,2]},'回転の固定点数を足して群の位数で割ります。群作用の正しさは入力側の前提です。'],
 polya:['Pólya Enumeration / Cycle Index',{n:5,dihedral:true,weights:[1,2],modulus:101},'輪の回転/反転群。各cycleの長さへ色の重みのべき和を代入します。']
};
const paths={};for(const k of Object.keys(specs))paths[k]=(Object.keys(specs).indexOf(k)<6?'dp_optimization/':Object.keys(specs).indexOf(k)<12?'number_theory_basic/':'combinatorics/')+k;
function integer(x,lo=-1000,hi=1000){if(!Number.isSafeInteger(x)||x<lo||x>hi)throw Error(`整数 ${lo}..${hi} を指定してください`);return x;}
function array(x,max=30){if(!Array.isArray(x)||x.length>max)throw Error(`長さ${max}以下の配列を指定してください`);return x;}
const LO=-(1n<<63n),HI=(1n<<63n)-1n;
function big(x,lo=LO,hi=HI){if(typeof x==='number'&&!Number.isSafeInteger(x)||typeof x!=='number'&&typeof x!=='string'||typeof x==='string'&&!/^-?\d+$/.test(x))throw Error('整数は安全なJSON数値か10進文字列で指定してください');let v;try{v=BigInt(x);}catch{throw Error('整数を指定してください');}if(v<lo||v>hi)throw Error('整数が範囲外です');return v;}
const mod=(a,m)=>(a%m+m)%m,gcd=(a,b)=>{while(b)[a,b]=[b,a%b];return a;};
function inverse(a,m){let r=a,s=m,x=1n,y=0n;while(s){let q=r/s;[r,s]=[s,r-q*s];[x,y]=[y,x-q*y];}if(r!==1n)throw Error('逆元が存在しません');return mod(x,m);}
function power(a,n,m){let v=1n%m;a=mod(a,m);while(n){if(n&1n)v=v*a%m;a=a*a%m;n>>=1n;}return v;}
const checked=x=>{if(x<LO||x>HI)throw Error('long longの中間値を超過しました');return x;};
const factor=n=>{const f=[];for(let p=2;p*p<=n;p++)if(n%p===0){f.push(p);while(n%p===0)n/=p;}if(n>1)f.push(n);return f;};
function build(mode,input){
 if(!Object.hasOwn(specs,mode)||!input||typeof input!=='object'||Array.isArray(input))throw Error('モードと入力オブジェクトを確認してください');
 const frames=[];let result;
 const emit=(title,note,columns,rows,extra={})=>{if(frames.length>6000)throw Error('状態数が多すぎます');frames.push({title,note,columns,rows:rows.length?rows:[['空']],...extra});};
 const series=(label,a)=>a.some(v=>v!==null)?({series:[{label,points:a.flatMap((v,i)=>v===null?[]:[[i,v]])}]}):{};
 if(mode==='divide_conquer_dp'||mode==='knuth_optimization'||mode==='aliens_trick'){
  const values=array(input.values,14).map(x=>integer(x,0,100)),n=values.length,prefix=[0];for(const x of values)prefix.push(prefix.at(-1)+x);
  const cost=(l,r)=>(prefix[r]-prefix[l])**2;
  if(mode==='divide_conquer_dp'){
   const groups=integer(input.groups,0,16),dp=[Array(n+1).fill(null)],parents=[Array(n+1).fill(-1)];dp[0][0]=0;
   for(let k=1;k<=groups;k++){
    const row=Array(n+1).fill(null),parent=Array(n+1).fill(-1);
    function solve(l,r,lo,hi){if(l>=r)return;const mid=Math.floor((l+r)/2),candidates=[];let best=lo,bestValue=null;for(let j=lo;j<=hi;j++){const v=j<mid&&dp[k-1][j]!==null?dp[k-1][j]+cost(j,mid):null;candidates.push([j,v===null?'∞':v]);if(v!==null&&(bestValue===null||v<bestValue)){best=j;bestValue=v;}}row[mid]=bestValue;if(bestValue!==null)parent[mid]=best;emit('中央の行を解く',`段${k}、r=${mid}、候補[${lo},${hi}]、opt=${best}`,['分割点','候補費用'],candidates,series('今回のDP行',row));solve(l,mid,lo,best);solve(mid+1,r,best,hi);}
    if(n)solve(1,n+1,0,n-1);dp.push(row);parents.push(parent);
   }
   let parts=null;if(dp[groups][n]!==null){parts=[];let end=n;for(let k=groups;k>0;k--){parts.push([parents[k][end],end]);end=parents[k][end];}parts.reverse();}result={dp,partitions:parts};
  }else if(mode==='knuth_optimization'){
   const dp=Array.from({length:n+1},()=>Array(n+1).fill(0)),split=Array.from({length:n+1},()=>Array(n+1).fill(-1));
   for(let len=2;len<=n;len++)for(let l=0;l+len<=n;l++){const r=l+len,lo=len===2?l+1:Math.max(l+1,split[l][r-1]),hi=len===2?r-1:Math.min(r-1,split[l+1][r]),candidates=[];for(let k=lo;k<=hi;k++){const v=dp[l][k]+dp[k][r]+prefix[r]-prefix[l];candidates.push([k,dp[l][k],dp[k][r],v]);if(split[l][r]<0||v<dp[l][r]){dp[l][r]=v;split[l][r]=k;}}emit('optの間だけ探索',`[${l},${r})、探索範囲[${lo},${hi}]、opt=${split[l][r]}`,['k','左DP','右DP','合計'],candidates);}
   const merges=[];function visit(l,r){const k=split[l][r];if(k<0)return;visit(l,k);visit(k,r);merges.push([l,k,r]);}visit(0,n);result={value:dp[0][n],split,merges};
  }else{
   const k=integer(input.groups,0,16);let low=integer(input.low,-100000,100000),high=integer(input.high,low,100000);const history=[];
   const oracle=p=>{const dp=Array(n+1).fill(Infinity),count=Array(n+1).fill(0);dp[0]=0;for(let r=1;r<=n;r++)for(let l=0;l<r;l++){const v=dp[l]+cost(l,r)+p,c=count[l]+1;if(v<dp[r]||v===dp[r]&&c>count[r]){dp[r]=v;count[r]=c;}}history.push([p,dp[n],count[n]]);emit('罰則付きDPを解く',`同値なら多い個数。探索範囲=[${low},${high}]`,['penalty','最適値','区間数'],history.slice());return{value:dp[n],count:count[n]};};
   const first=oracle(low),last=oracle(high);if(first.count<k||last.count>k)result=null;else{if(last.count!==k){while(low<high){const mid=Math.ceil((low+high)/2),v=oracle(mid);if(v.count>=k)low=mid;else high=mid-1;}}else low=high;const v=oracle(low);result={value:v.value-low*k,penalty:low,relaxed:v};}
  }
 }else if(mode==='convex_hull_trick_dp'||mode==='li_chao_dp'){
  const heights=array(input.heights,14).map(x=>integer(x,-100,100)),constant=integer(input.constant,-100,100),dp=[],lines=[],xs=[...new Set(heights)].sort((a,b)=>a-b),tree=[];
  const value=(line,x)=>line[0]*x+line[1];
  function insert(node,l,r,line){if(!tree[node]){tree[node]=line;return;}const m=Math.floor((l+r)/2),left=value(line,xs[l])<value(tree[node],xs[l]),middle=value(line,xs[m])<value(tree[node],xs[m]);if(middle)[tree[node],line]=[line,tree[node]];emit('中央で優れた直線を保持',`ノード${node}、座標[${xs[l]},${xs[r-1]}]`,['ノード','傾き','切片'],tree.flatMap((a,i)=>a?[[i,...a]]:[]));if(r-l===1)return;if(left!==middle)insert(node*2,l,m,line);else insert(node*2+1,m,r,line);}
  function query(x){const pos=xs.indexOf(x);let l=0,r=xs.length,node=1,best=Infinity;for(;;){if(tree[node])best=Math.min(best,value(tree[node],x));if(r-l===1)return best;const m=Math.floor((l+r)/2);if(pos<m){node*=2;r=m;}else{node=node*2+1;l=m;}}}
  for(let i=0;i<heights.length;i++){const x=heights[i];dp.push(i?(mode==='li_chao_dp'?query(x):Math.min(...lines.map(l=>value(l,x))))+x*x+constant:0);emit('過去の直線へquery',`i=${i}、h=${x}、dp=${dp[i]}`,['j','傾き','切片','現在座標での値'],lines.map((l,j)=>[j,...l,value(l,x)]),{series:[...lines.slice(-3).map((l,j)=>({label:`直線${lines.length-Math.min(lines.length,3)+j}`,points:xs.map(x=>[x,value(l,x)])})),{label:'DP',points:dp.map((v,j)=>[heights[j],v])}]});const line=[-2*x,dp[i]+x*x];lines.push(line);if(mode==='li_chao_dp')insert(1,0,xs.length,line);emit('次の直線を追加','自分自身への遷移は含めない',['j','傾き','切片'],lines.map((l,j)=>[j,...l]));}result=dp;
 }else if(mode==='slope_trick'){
  let left=[],right=[],minimum=0;const operations=array(input.operations,35);const L=x=>{left.push(x);left.sort((a,b)=>b-a);},R=x=>{right.push(x);right.sort((a,b)=>a-b);};
  function addA(a){if(right.length&&right[0]<a){const r=right.shift();minimum+=a-r;R(a);L(r);}else L(a);}
  function addX(a){if(left.length&&left[0]>a){const l=left.shift();minimum+=l-a;L(a);R(l);}else R(a);}
  const evaluate=x=>minimum+left.reduce((s,a)=>s+Math.max(a-x,0),0)+right.reduce((s,a)=>s+Math.max(x-a,0),0);
  for(const op of operations){array(op,3);const kind=op[0];if(['abs','left','right','constant','shift'].includes(kind))integer(op[1],-50,50);if(kind==='abs'){addA(op[1]);addX(op[1]);}else if(kind==='left')addA(op[1]);else if(kind==='right')addX(op[1]);else if(kind==='constant')minimum+=op[1];else if(kind==='prefix')right=[];else if(kind==='suffix')left=[];else if(kind==='shift'){const a=op[1],b=op.length===2?a:integer(op[2],a,50);left=left.map(x=>x+a);right=right.map(x=>x+b);}else throw Error('abs/left/right/constant/prefix/suffix/shiftを指定してください');const lo=Math.min(-10,...left,...right)-3,hi=Math.max(10,...left,...right)+3;emit('凸関数を更新',`${JSON.stringify(op)}、minimum=${minimum}`,['heap','折れ点'],[['左（降順）',left.join(',')],['右（昇順）',right.join(',')]],{series:[{label:'f(x)',points:Array.from({length:61},(_,i)=>{const x=lo+(hi-lo)*i/60;return[x,evaluate(x)];})}]});}result={minimum,argmin:[left[0]??null,right[0]??null],values:Array.from({length:21},(_,i)=>evaluate(i-10))};
 }else if(mode==='euler_phi'||mode==='mobius'){
  const n=integer(input.limit,0,100),spf=Array(n+1).fill(0),answer=Array(n+1).fill(0);if(n)answer[1]=1;
  for(let i=2;i<=n;i++)if(!spf[i])for(let j=i;j<=n;j+=i)if(!spf[j])spf[j]=i;
  for(let i=2;i<=n;i++){const p=spf[i],q=i/p;answer[i]=mode==='mobius'?(q%p===0?0:0-answer[q]):answer[q]*(q%p===0?p:p-1);emit('SPFで更新',`${i}=${p}×${q}、${q%p===0?'同じ素因数':'新しい素因数'}`,['n','SPF','値'],answer.slice(0,i+1).map((v,j)=>[j,spf[j],v]),series(mode==='mobius'?'μ':'φ',answer.slice(0,i+1)));}result=answer;
 }else if(mode==='crt'||mode==='garner'){
  const residues=array(input.residues,10).map(x=>big(x)),mods=array(input.moduli,10).map(x=>big(x,1n));if(residues.length!==mods.length)throw Error('residuesとmoduliの長さを揃えてください');
  if(mode==='crt'){
   let r=0n,m=1n;result={remainder:'0',modulus:'1'};
   for(let i=0;i<mods.length;i++){const b=mod(residues[i],mods[i]),n=mods[i],g=gcd(m,n),delta=b-r;if(delta%g){emit('矛盾を検出',`gcd=${g}が差${delta}を割らない`,['現在のr','現在のm','新しい剰余','新しい法'],[[r.toString(),m.toString(),b.toString(),n.toString()]]);result=null;break;}const reduced=n/g;checked(m*reduced);const t=mod(delta/g*inverse(m/g,reduced),reduced);r+=m*t;m*=reduced;result={remainder:r.toString(),modulus:m.toString()};emit('合同式を併合',`gcd=${g}、補正係数=${t}`,['r','m','残りの法'],[[r.toString(),m.toString(),mods.slice(i+1).join(',')]]);}
  }else{
   const target=big(input.target,1n),m=[...mods,target],constant=m.map(()=>0n),coefficient=m.map(x=>1n%x);for(let i=0;i<mods.length;i++)for(let j=0;j<i;j++)if(gcd(mods[i],mods[j])!==1n)throw Error('入力法は互いに素にしてください');
   for(let i=0;i<mods.length;i++){const t=mod((mod(residues[i],m[i])-constant[i])*inverse(coefficient[i],m[i]),m[i]);for(let j=i+1;j<m.length;j++){constant[j]=mod(constant[j]+coefficient[j]*t,m[j]);coefficient[j]=coefficient[j]*m[i]%m[j];}emit('混合基数の桁を確定',`桁${i}=${t}`,['位置','法','定数','係数'],m.map((v,j)=>[j,v.toString(),constant[j].toString(),coefficient[j].toString()]));}result=constant.at(-1).toString();
  }
 }else if(mode==='floor_sum'){
  let n=big(input.n,0n,4294967295n),m=big(input.m,1n,4294967295n),a=big(input.a),b=big(input.b);const qa=(a-mod(a,m))/m,qb=(b-mod(b,m))/m,first=checked(n*(n-1n)/2n*qa),second=checked(n*qb);let base=n?checked(first+second):0n,positive=0n;a=mod(a,m);b=mod(b,m);
  emit('負係数を正規化',`qa=${qa}、qb=${qb}、定数寄与=${base}`,['n','m','a','b'],[[n,m,a,b].map(String)]);
  while(n){positive+=n*(n-1n)/2n*(a/m)+n*(b/m);a%=m;b%=m;emit('商の寄与を足す',`非負部分=${positive}`,['n','m','a','b'],[[n,m,a,b].map(String)]);const y=a*n+b;if(y<m)break;n=y/m;b=y%m;[m,a]=[a,m];emit('格子の軸を交換','Euclidと同じく法が縮む',['n','m','a','b'],[[n,m,a,b].map(String)]);}result=checked(base+positive).toString();
 }else if(mode==='primitive_root'){
  const p=integer(input.prime,2,2000);if(factor(p).length!==1||factor(p)[0]!==p)throw Error('素数を指定してください');const factors=factor(p-1);result=p===2?1:null;for(let g=2;g<p;g++){const tests=factors.map(q=>[q,Number(power(BigInt(g),BigInt((p-1)/q),BigInt(p)))]);emit('候補の位数を検査',`g=${g}、p-1の素因数=[${factors}]`,['q','g^((p-1)/q) mod p'],tests);if(tests.every(t=>t[1]!==1)){result=g;break;}}
 }else if(['stirling','bell','partition_numbers','derangements'].includes(mode)){
  const n=integer(input.n,0,35),m=BigInt(integer(input.modulus,1,1000000000)),M=x=>Number(mod(BigInt(x),m));
  if(mode==='stirling'){
   const kind=input.kind;if(!['first','second','signed'].includes(kind))throw Error('kindはfirst/signed/secondです');const a=[[Number(1n%m)]];
   for(let i=1;i<=n;i++){const row=Array(i+1).fill(0);for(let k=1;k<=i;k++)row[k]=M(a[i-1][k-1]+(kind==='signed'?-1:1)*(kind==='second'?k:i-1)*(a[i-1][k]||0));a.push(row);emit('三角表を更新',`n=${i}、kind=${kind}`,['n','値'],a.map((r,j)=>[j,r.join(',')]),series('現在の行',row));}result=a;
  }else if(mode==='bell'){
   let row=[Number(1n%m)];const answer=[row[0]];for(let i=1;i<=n;i++){const next=[row.at(-1)];for(let k=1;k<=i;k++)next.push(M(next[k-1]+row[k-1]));answer.push(next[0]);row=next;emit('Bell三角形を更新',`行${i}の先頭=${row[0]}`,['位置','値'],row.map((x,j)=>[j,x]),series('Bell数',answer));}result=answer;
  }else if(mode==='partition_numbers'){
   const a=Array(n+1).fill(0);a[0]=Number(1n%m);for(let i=1;i<=n;i++){const terms=[];for(let k=1;k*(3*k-1)/2<=i;k++)for(const p of[k*(3*k-1)/2,k*(3*k+1)/2])if(p<=i){const sign=k%2?1:-1;a[i]=M(a[i]+sign*a[i-p]);terms.push([p,sign,i-p,a[i-p],a[i]]);}emit('五角数から遷移',`n=${i}`,['五角数','符号','参照先','参照値','累計'],terms,series('p(n)',a.slice(0,i+1)));}result=a;
  }else{
   const a=Array(n+1).fill(0);a[0]=Number(1n%m);for(let i=2;i<=n;i++){a[i]=M((i-1)*(a[i-1]+a[i-2]));emit('2種類のcycleで更新',`D(${i})=(${i}-1)(D(${i-1})+D(${i-2}))`,['n','値'],a.slice(0,i+1).map((v,j)=>[j,v]),series('D(n)',a.slice(0,i+1)));}result=a;
  }
 }else if(mode==='burnside'){
  const counts=array(input.fixed,30).map(x=>big(x,0n,(1n<<64n)-1n));if(!counts.length)throw Error('空の群は使えません');let sum=0n;for(let i=0;i<counts.length;i++){sum+=counts[i];if(sum>(1n<<64n)-1n)throw Error('uint64の固定点総和を超過しました');emit('固定点を合計',`群要素${i}、総和=${sum}`,['群要素','固定点数'],counts.slice(0,i+1).map((x,j)=>[j,x.toString()]));}if(sum%BigInt(counts.length))throw Error('平均が整数になりません');result=(sum/BigInt(counts.length)).toString();
 }else if(mode==='polya'){
  const n=integer(input.n,0,10),weights=array(input.weights,5).map(x=>integer(x,-10,10)),m=BigInt(integer(input.modulus,1,1000000000));if(typeof input.dihedral!=='boolean')throw Error('dihedralは真偽値です');const group=[];if(!n)group.push([]);else for(let flip=0;flip<=Number(input.dihedral);flip++)for(let shift=0;shift<n;shift++)group.push(Array.from({length:n},(_,i)=>(shift+(flip?n-i:i))%n));
  let sum=0n;const terms=new Map();for(const p of group){const visited=Array(n).fill(false),counts=Array(n+1).fill(0),cycles=[];let value=1n%m;for(let v=0;v<n;v++)if(!visited[v]){let u=v;const cycle=[];do{visited[u]=true;cycle.push(u);u=p[u];}while(u!==v);cycles.push(cycle);counts[cycle.length]++;value=value*weights.reduce((s,w)=>mod(s+power(BigInt(w),BigInt(cycle.length),m),m),0n)%m;}sum=(sum+value)%m;const key=counts.join(',');terms.set(key,(terms.get(key)||0)+1);emit('cycleの重みを代入',`cycle=[${cycles.map(c=>`(${c.join(' ')})`).join('')}]、寄与=${value}`,['cycle長','個数','色の重みべき和'],counts.flatMap((count,i)=>count?[[i,count,weights.reduce((s,w)=>mod(s+power(BigInt(w),BigInt(i),m),m),0n).toString()]]:[]),{graph:{n,directed:true,edges:p.map((v,u)=>[u,v])}});}
  const answer=sum*inverse(BigInt(group.length),m)%m;result={value:Number(answer),terms:[...terms].map(([key,count])=>[key.split(',').map(Number),count])};
 }else throw Error('不明なモードです');
 if(!frames.length)emit('初期状態',specs[mode][2],['状態'],[['空または基底値']]);emit('完了',JSON.stringify(result),['結果'],[[JSON.stringify(result)]]);return{frames,result};
}
const api={specs,paths,build};if(typeof module!=='undefined')module.exports=api;root.MathAModels=api;
})(typeof window==='undefined'?globalThis:window);
