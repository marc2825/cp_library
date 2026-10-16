(function(root){
'use strict';
const MODES=['matrix','product','power','gauss','xor'],MIN=-(1n<<63n),MAX=(1n<<63n)-1n;
const norm=(x,p)=>p?(x%p+p)%p:x;
const matrix=(rows,cols,value=0n)=>({rows,cols,data:Array.from({length:rows},()=>Array(cols).fill(value))});
function defaults(mode){return {mode,rows:'2',inner:'3',cols:mode==='gauss'?'3':'3',a:mode==='power'?'1 1\n1 0':mode==='gauss'?'1 2 3\n2 4 6':'1 2 3\n4 5 6',b:'7 8 9\n10 11 12\n13 14 15',rhs:'4 8',exponent:'10',modulus:mode==='gauss'?'real':'integer',epsilon:'1e-10',vectors:'3 5 6',seed:'1',operation:'max'};}
function scenario(mode,type){const c=defaults(mode);if(type==='zero'){c.a=Array(mode==='power'?4:6).fill('0').join(' ');c.rhs='0 0';c.vectors='0 0';}if(type==='empty'){c.rows='0';c.inner='0';c.cols='3';c.a='';c.b='';c.rhs='';c.vectors='';}if(type==='unique'){c.cols='2';c.a='1 1\n2 -1';c.rhs='3 0';}if(type==='inconsistent'){c.rhs='4 9';}if(type==='swap'){c.cols='2';c.a='0 1\n1 1';c.rhs='2 3';}if(type==='tiny'){c.cols='2';c.a='0.00000000000000000001 1\n1 1';c.rhs='2 3';}if(type==='mod17'){c.modulus='17';c.rhs='4 8';}if(type==='mod12')c.modulus='12';if(type==='mod1')c.modulus='1';if(type==='one'){c.rows='1';c.inner='1';c.cols='1';c.a='7';c.b='2';c.rhs='3';c.vectors='7';}if(type==='exponent-zero')c.exponent='0';if(type==='huge'){c.a='0 1\n1 0';c.exponent='18446744073709551615';}if(type==='overflow'){c.rows='1';c.inner='1';c.cols='1';c.a='9223372036854775807';c.b='2';c.exponent='2';}if(type==='highest'){c.vectors='9223372036854775808 1 9223372036854775809';c.seed='18446744073709551615';}if(type==='dependent'){c.vectors='3 5 6 0 3';c.operation='contains';c.seed='6';}if(type==='outside'){c.operation='contains';c.seed='1';}return c;}
function parse(raw){
 const mode=MODES.includes(raw.mode)?raw.mode:'matrix';
 const integer=(s,name,lo,hi)=>{if(!/^-?\d+$/.test(String(s)))throw Error(`${name}は整数で指定します。`);const x=BigInt(s);if(x<lo||x>hi)throw Error(`${name}が範囲外です。`);return x;};
 if(mode==='xor'){
  const list=String(raw.vectors||'').trim(),a=list?list.split(/[\s,]+/).map(x=>integer(x,'ベクトル',0n,(1n<<64n)-1n)):[];if(a.length>12)throw Error('図のXOR入力は12個までです。');
  return {mode,vectors:a,seed:integer(raw.seed,'seed',0n,(1n<<64n)-1n),operation:['min','max','contains'].includes(raw.operation)?raw.operation:'max'};
 }
 const rows=Number(integer(raw.rows,'行数',0n,5n)),cols=mode==='power'?rows:Number(integer(raw.cols,'列数',0n,5n)),inner=mode==='product'?Number(integer(raw.inner,'内側の次元',0n,5n)):cols;
 const real=mode==='gauss'&&raw.modulus==='real';if(mode==='gauss'&&!real&&!['2','17','998244353'].includes(String(raw.modulus)))throw Error('連立方程式には実数か素数法を選びます。');
 const p=real?0n:raw.modulus==='integer'?0n:BigInt(['1','2','12','17','998244353'].includes(String(raw.modulus))?raw.modulus:(()=>{throw Error('計算方法を選んでください。');})());
 const scalar=s=>{if(real){if(!String(s).trim()||!Number.isFinite(Number(s)))throw Error('係数は有限の実数にします。');return Number(s);}return norm(integer(s,'係数',MIN,MAX),p);};
 const list=(s,size,name)=>{const a=String(s||'').trim()?String(s).trim().split(/[\s,]+/).map(scalar):[];if(a.length!==size)throw Error(`${name}は${size}項が必要です。`);return a;};
 const a=matrix(rows,inner,real?0:0n),values=list(raw.a,rows*inner,'A');for(let i=0;i<rows;++i)for(let j=0;j<inner;++j)a.data[i][j]=values[i*inner+j];
 let b=null,rhs=[];if(mode==='product'){b=matrix(inner,cols);const entries=list(raw.b,inner*cols,'B');for(let i=0;i<inner;++i)for(let j=0;j<cols;++j)b.data[i][j]=entries[i*cols+j];}if(mode==='gauss')rhs=list(raw.rhs,rows,'b');
 const epsilon=Number(raw.epsilon);if(real&&(!Number.isFinite(epsilon)||epsilon<=0))throw Error('epsilonは正の有限値にします。');
 return {mode,rows,cols,inner,a,b,rhs,real,p,epsilon,exponent:mode==='power'?integer(raw.exponent,'指数',0n,(1n<<64n)-1n):0n};
}
function run(raw){
 const c=parse(raw),{mode}=c,frames=[];const emit=(title,description,state={})=>frames.push(structuredClone({title,description,result:null,...state}));
 if(mode==='xor'){
  const pivots=Array(64).fill(0n);let rank=0;const prefix=[];
  const state=(extra={})=>({pivots,rank,prefix,phase:'insert',...extra});emit('ビットベクトルとして読む','XORはGF(2)の加算です。最上位の1が同じなら、その基底で消せます。',state());
  for(let inputIndex=0;inputIndex<c.vectors.length;++inputIndex){let value=c.vectors[inputIndex];prefix.push(value);emit(`入力${inputIndex}を挿入`,'既存のpivotを大きいビットから使い、残った1を探します。',state({value,inputIndex,stage:'start'}));for(let bit=63;bit>=0;--bit)if(value>>BigInt(bit)&1n){if(pivots[bit]){const before=value;value^=pivots[bit];emit(`bit ${bit}をXORで消去`,'既存ベクトルとのXORで、そのビットの1を消します。',state({value,inputIndex,bit,before,stage:'reduce'}));}else{pivots[bit]=value;rank++;emit(`bit ${bit}を新しいpivotにする`,'0まで消せなかったため、張る空間の次元が1増えます。',state({value,inputIndex,bit,stage:'insert'}));break;}}if(!value)emit('従属なベクトル','0は既存の基底のXORで表せます。基底は変わりません。',state({value,inputIndex,stage:'dependent'}));}
  emit('基底の構築が完了',`rank=${rank}、表せる値は2^rank種類です。`,state({stage:'complete'}));let value=c.seed;
  emit('seedから問い合わせる','基底が張る空間の値をXORして、seedの属する剰余類を調べます。',state({value,phase:'query',stage:'start'}));
  for(let bit=63;bit>=0;--bit)if(pivots[bit]){const before=value,candidate=value^pivots[bit],take=c.operation==='max'?candidate>value:candidate<value;if(take)value=candidate;emit(`bit ${bit}の候補を比較`,c.operation==='max'?'数値を増やせる場合にXORを採用します。':'数値を減らせる場合にXORを採用します。',state({value,before,candidate,take,bit,phase:'query',stage:'query'}));}
  const result=c.operation==='contains'?value===0n:value;emit('問い合わせの完了',c.operation==='contains'?'最小の剰余が0なら、seedは基底の張る空間に含まれます。':'大きいpivotを確定すると、より下のビットではその選択を覆せません。',state({value,phase:'query',stage:'done',result}));return {config:c,frames,result};
 }
 const zero=c.real?0:0n,one=c.real?1:norm(1n,c.p);
 const checked=x=>{if(c.real){if(!Number.isFinite(x))throw Error('途中の演算が非有限値になりました。');return x;}x=norm(x,c.p);if(!c.p&&(x<MIN||x>MAX))throw Error('途中の積・加減算がlong longの範囲を超えます。');return x;};
 const add=(a,b)=>checked(a+b),sub=(a,b)=>checked(a-b),mul=(a,b)=>checked(a*b),identity=n=>{const a=matrix(n,n,zero);for(let i=0;i<n;++i)a.data[i][i]=one;return a;};
 function product(a,b){const out=matrix(a.rows,b.cols,zero);for(let i=0;i<a.rows;++i)for(let j=0;j<b.cols;++j)for(let k=0;k<a.cols;++k)out.data[i][j]=add(out.data[i][j],mul(a.data[i][k],b.data[k][j]));return out;}
 if(mode==='matrix'){
  const target=matrix(c.a.cols,c.a.rows,zero);emit('長方形の形と行優先の保持','(行,列)の順で要素を指定します。0行や0列でも次元を保持します。',{a:c.a,target,stage:'input'});
  for(let i=0;i<c.a.rows;++i)for(let j=0;j<c.a.cols;++j){target.data[j][i]=c.a.data[i][j];emit(`A(${i},${j})を転置`,'元の行と列を入れ替えます。要素の値は変わりません。',{a:c.a,target,cell:[i,j],stage:'copy'});}
  emit('転置の完了','n×mの行列はm×nになります。',{a:c.a,target,stage:'done',result:target});return {config:c,frames,result:structuredClone(target)};
 }
 if(mode==='product'){
  const out=matrix(c.rows,c.cols,zero);emit('行と列の内積','Aの列数とBの行数をそろえます。出力CはAの行数×Bの列数です。',{a:c.a,b:c.b,out,stage:'input'});
  for(let i=0;i<c.rows;++i)for(let j=0;j<c.cols;++j){emit(`C(${i},${j})の計算を開始`,'Aの行iとBの列jを、同じkで対応させます。',{a:c.a,b:c.b,out,cell:[i,j],stage:'start'});for(let k=0;k<c.inner;++k){const before=out.data[i][j],term=mul(c.a.data[i][k],c.b.data[k][j]);out.data[i][j]=add(before,term);emit(`k=${k}の積を加える`,`${c.a.data[i][k]} × ${c.b.data[k][j]}を部分和へ足します。`,{a:c.a,b:c.b,out,cell:[i,j],k,before,term,stage:'term'});}emit(`C(${i},${j})が確定`,'内側の次元を全てたどりました。',{a:c.a,b:c.b,out,cell:[i,j],stage:'complete'});}
  emit('行列積の完了','行列積は順序に依存します。A*BとB*Aは一般に異なります。',{a:c.a,b:c.b,out,stage:'done',result:out});return {config:c,frames,result:structuredClone(out)};
 }
 if(mode==='power'){
  let base=structuredClone(c.a),acc=identity(c.rows),remaining=c.exponent,weight=1n,used=0n,bit=0;
  const state=extra=>({base,acc,remaining,weight,used,bit,...extra});emit('単位行列から始める','指数0なら、この単位行列が答えです。',state({stage:'input'}));
  while(remaining){if(remaining&1n){acc=product(acc,base);remaining--;used+=weight;emit(`bit ${bit}を採用`,'答えの右側へ、この2冪の行列を掛けます。',state({stage:'select'}));}if(!remaining)break;base=product(base,base);remaining>>=1n;weight<<=1n;bit++;emit(`Aの${weight}乗へ二乗`,'基底の行列を二乗し、次のビットを調べます。',state({stage:'square'}));}
  emit('累乗の完了','必要な指数を全て積に取り込みました。最後の不要な二乗は行いません。',state({stage:'done',result:acc}));return {config:c,frames,result:structuredClone(acc)};
 }
 const pow=(a,e)=>{let r=1n;for(;e;e>>=1n,a=a*a%c.p)if(e&1n)r=r*a%c.p;return r;};
 const divide=(a,b)=>c.real?checked(a/b):mul(a,pow(b,c.p-2n));
 const iszero=x=>c.real?Math.abs(x)<=c.epsilon:x===0n;
 const r=matrix(c.rows,c.cols+1,zero),transform=identity(c.rows),pivots=[];
 for(let i=0;i<c.rows;++i)r.data[i]=[...c.a.data[i],c.rhs[i]];
 const state=extra=>({r,transform,pivots,rank:pivots.length,...extra});emit('拡大係数行列 [A|b]','右辺にも同じ行基本変形を適用して、同じ方程式系を保ちます。',state({stage:'input'}));
 for(let col=0;col<c.cols&&pivots.length<c.rows;++col){const row=pivots.length;let pivot=row;if(c.real){for(let i=row+1;i<c.rows;++i)if(Math.abs(r.data[i][col])>Math.abs(r.data[pivot][col]))pivot=i;}else while(pivot<c.rows&&iszero(r.data[pivot][col]))pivot++;
  if(pivot===c.rows||iszero(r.data[pivot][col])){for(let i=row;i<c.rows;++i)r.data[i][col]=zero;emit(`列${col}は自由変数の候補`,'使えるpivotがないため、この列を飛ばします。',state({col,row,stage:'skip'}));continue;}
  emit(`列${col}のpivotを選ぶ`,c.real?'残っている行から、絶対値が最大の係数を選びます。':'残っている行から、0でない係数を選びます。',state({col,row,pivot,stage:'pivot'}));
  if(pivot!==row){[r.data[pivot],r.data[row]]=[r.data[row],r.data[pivot]];[transform.data[pivot],transform.data[row]]=[transform.data[row],transform.data[pivot]];emit('行を交換','式の順番を入れ替えて、pivotを処理行へ置きます。',state({col,row,pivot,stage:'swap'}));}
  const divisor=r.data[row][col];for(let j=col+1;j<=c.cols;++j)r.data[row][j]=divide(r.data[row][j],divisor);for(let j=0;j<c.rows;++j)transform.data[row][j]=divide(transform.data[row][j],divisor);r.data[row][col]=one;pivots.push(col);
  emit('pivotを1に正規化','pivotの係数で、式全体を割ります。',state({col,row,divisor,stage:'normalize'}));
  for(let i=0;i<c.rows;++i)if(i!==row){const factor=r.data[i][col];for(let j=col+1;j<=c.cols;++j)r.data[i][j]=sub(r.data[i][j],mul(factor,r.data[row][j]));for(let j=0;j<c.rows;++j)transform.data[i][j]=sub(transform.data[i][j],mul(factor,transform.data[row][j]));r.data[i][col]=zero;emit(`行${i}からpivot行を引く`,'係数factor倍のpivot行を引き、この列を0にします。',state({col,row,target:i,factor,stage:'eliminate'}));}
  emit(`列${col}の消去が完了`,'このpivot列は、pivot行が1、それ以外の行が0になりました。',state({col,row,stage:'complete'}));
 }
 let status='unique',particular=[],basis=[];for(let i=pivots.length;i<c.rows;++i)if(!iszero(r.data[i][c.cols]))status='none';
 if(status!=='none'){status=pivots.length===c.cols?'unique':'multiple';particular=Array(c.cols).fill(zero);for(let i=0;i<pivots.length;++i)particular[pivots[i]]=r.data[i][c.cols];for(let col=0;col<c.cols;++col)if(!pivots.includes(col)){const v=Array(c.cols).fill(zero);v[col]=one;for(let i=0;i<pivots.length;++i)v[pivots[i]]=sub(zero,r.data[i][col]);basis.push(v);}}
 const result={status,particular,basis,rank:pivots.length,pivots};emit('解空間を読み取る',status==='none'?'左辺が0で右辺が0でない行があり、矛盾しています。':status==='unique'?'全ての変数にpivotがあり、解は一意です。':'自由変数を0にした特解と、各自由変数を1にした零空間の基底を返します。',state({stage:'done',result}));return {config:c,frames,result:structuredClone(result)};
}
const api={MODES,defaults,scenario,parse,run};if(typeof module!=='undefined'&&module.exports)module.exports=api;else root.LinearAlgebraModel=api;
})(typeof globalThis!=='undefined'?globalThis:this);
