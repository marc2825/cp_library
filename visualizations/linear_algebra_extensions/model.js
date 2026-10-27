(function(root){
'use strict';
const modes=typeof module!=='undefined'?require('./modes.js'):root.LinearAlgebraExtensionModes;
const check=(ok,message)=>{if(!ok)throw Error(message);};
const integer=x=>{check((typeof x==='number'&&Number.isSafeInteger(x))||(typeof x==='string'&&/^-?\d{1,128}$/.test(x)),'整数は安全な数値または128桁以内の10進文字列です');return BigInt(x);};
const mod=(x,m)=>(x%m+m)%m;
const gcd=(a,b)=>{while(b)[a,b]=[b,a%b];return a<0n?-a:a;};
const egcd=(a,b)=>{let x=1n,y=0n,u=0n,v=1n;while(b){const q=a/b;[a,b,x,u,y,v]=[b,a-q*b,u,x-q*u,v,y-q*v];}if(a<0n)return [-a,-x,-y];return [a,x,y];};
const identity=(n,R)=>Array.from({length:n},(_,i)=>Array.from({length:n},(_,j)=>i===j?R.one():R.zero()));
const Z={zero:()=>0n,one:()=>1n,add:(a,b)=>a+b,sub:(a,b)=>a-b,mul:(a,b)=>a*b,divmod:(a,b)=>[a/b,a%b],empty:a=>a===0n,unit:a=>a<0n?-1n:1n,format:String};
function polynomialRing(p){
 const add=(a,b)=>mod(a+b,p),sub=(a,b)=>mod(a-b,p),mul=(a,b)=>a*b%p;
 const pow=(a,k)=>{let r=1n;for(;k;k>>=1n,a=mul(a,a))if(k&1n)r=mul(r,a);return r;};
 const inv=a=>{check(a!==0n,'0の逆元はありません');return pow(a,p-2n);};
 const trim=a=>{a=a.slice();while(a.length&&a.at(-1)===0n)a.pop();return a;};
 const plus=(a,b,minus=false)=>trim(Array.from({length:Math.max(a.length,b.length)},(_,i)=>minus?sub(a[i]||0n,b[i]||0n):add(a[i]||0n,b[i]||0n)));
 const product=(a,b)=>{if(!a.length||!b.length)return [];const c=Array(a.length+b.length-1).fill(0n);for(let i=0;i<a.length;i++)for(let j=0;j<b.length;j++)c[i+j]=add(c[i+j],mul(a[i],b[j]));return trim(c);};
 const divmod=(a,b)=>{a=trim(a);b=trim(b);check(b.length,'ゼロ多項式で除算できません');const q=Array(Math.max(0,a.length-b.length+1)).fill(0n),scale=inv(b.at(-1));for(let i=q.length-1;i>=0;i--){q[i]=mul(a[i+b.length-1],scale);for(let j=0;j<b.length;j++)a[i+j]=sub(a[i+j],mul(q[i],b[j]));}return [trim(q),trim(a)];};
 const monic=a=>a.length?a.map(x=>mul(x,inv(a.at(-1)))):[];
 const extended=(a,b)=>{let x=[1n],y=[],u=[],v=[1n];while(b.length){const[q,r]=divmod(a,b);[a,b,x,u,y,v]=[b,r,u,plus(x,product(q,u),true),v,plus(y,product(q,v),true)];}if(!a.length)return [[],[],[]];const s=inv(a.at(-1));return [a,x,y].map(f=>f.map(c=>mul(c,s)));};
 const power=(a,k,f)=>{let r=divmod([1n],f)[1];a=divmod(a,f)[1];for(;k;k>>=1n,a=divmod(product(a,a),f)[1])if(k&1n)r=divmod(product(r,a),f)[1];return r;};
 return {zero:()=>[],one:()=>[1n],add:plus,sub:(a,b)=>plus(a,b,true),mul:product,divmod,empty:a=>!a.length,unit:a=>[inv(a.at(-1))],format:a=>'['+a.map(String).join(',')+']',monic,extended,power,trim};
}
function smith(a,R,emit=()=>{}){
 a=a.map(row=>row.slice());const m=a.length,n=m?a[0].length:0,u=identity(m,R),v=identity(n,R);
 let operations=0;
 const show=(title,k)=>{check(++operations<=10000,'図解の基本操作上限を超えました');emit(title,a.map(row=>row.map(R.format)),[k,k]);};
 const rowAdd=(i,j,q)=>{for(let c=0;c<n;c++)a[i][c]=R.add(a[i][c],R.mul(q,a[j][c]));for(let c=0;c<m;c++)u[i][c]=R.add(u[i][c],R.mul(q,u[j][c]));};
 const rowSwap=(i,j)=>{[a[i],a[j]]=[a[j],a[i]];[u[i],u[j]]=[u[j],u[i]];};
 const colAdd=(i,j,q)=>{for(let r=0;r<m;r++)a[r][i]=R.add(a[r][i],R.mul(q,a[r][j]));for(let r=0;r<n;r++)v[r][i]=R.add(v[r][i],R.mul(q,v[r][j]));};
 const colSwap=(i,j)=>{for(const row of [...a,...v])[row[i],row[j]]=[row[j],row[i]];};
 let rank=0;
 while(rank<Math.min(m,n)){
  const k=rank;let pivot=null;for(let i=k;i<m&&!pivot;i++)for(let j=k;j<n;j++)if(!R.empty(a[i][j])){pivot=[i,j];break;}
  if(!pivot)break;rowSwap(k,pivot[0]);colSwap(k,pivot[1]);show('非零pivotを選ぶ',k);
  for(;;){
   for(let i=k+1;i<m;i++)while(!R.empty(a[i][k])){const q=R.divmod(a[i][k],a[k][k])[0];rowAdd(i,k,R.sub(R.zero(),q));if(!R.empty(a[i][k]))rowSwap(i,k);show('行のEuclid変形',k);}
   for(let j=k+1;j<n;j++)while(!R.empty(a[k][j])){const q=R.divmod(a[k][j],a[k][k])[0];colAdd(j,k,R.sub(R.zero(),q));if(!R.empty(a[k][j]))colSwap(j,k);show('列のEuclid変形',k);}
   if(a.some((row,i)=>i>k&&!R.empty(row[k])))continue;
   let bad=-1;for(let i=k+1;i<m&&bad<0;i++)for(let j=k+1;j<n;j++)if(!R.empty(R.divmod(a[i][j],a[k][k])[1])){bad=i;break;}
   if(bad<0)break;rowAdd(k,bad,R.one());show('割り切れない成分をpivot行へ',k);
  }
  const unit=R.unit(a[k][k]);a[k]=a[k].map(x=>R.mul(unit,x));u[k]=u[k].map(x=>R.mul(unit,x));show('不変因子を確定',k);rank++;
 }
 return {a,u,v,rank};
}
function trace(id,input){
 check(input&&typeof input==='object'&&!Array.isArray(input),'入力はJSON objectです');check(modes.some(m=>m.id===id),'不明なモードです');
 const steps=[];let result=[],meta={};
 const step=(title,detail,values,pivot=null)=>{
  if(steps.length>=600)return;
  const matrix=Array.isArray(values)&&values.every(Array.isArray)?values.map(row=>row.map(String)):null;
  const rows=matrix?matrix.map((row,i)=>[i,...row]):values.map((x,i)=>[i,String(x)]);
  steps.push({title,detail,matrix,pivot,columns:matrix?['行 / 列',...Array.from({length:Math.max(0,...matrix.map(r=>r.length))},(_,i)=>i)]:['添字','値'],rows,values:[{label:matrix?'行数':'項数',value:String(values.length)}]});
 };
 const matrix=(value,max=6)=>{check(Array.isArray(value)&&value.length<=max&&value.every(row=>Array.isArray(row)&&row.length<=(max)&&row.length===(value[0]?.length||0)),`行列は${max}×${max}以下の長方形です`);return value.map(row=>row.map(integer));};
 const square=a=>check(!a.length||a.length===a[0].length,'正方行列を指定してください');
 const field=()=>{const p=integer(input.prime??998244353);check(p>=2n&&p<=998244353n,'素数法は2〜998244353です');for(let d=2n;d*d<=p;d++)check(p%d!==0n,'素数法を指定してください');return p;};
 step('入力を確認','整数の大きい値は10進文字列で入力できます。',Object.entries(input).map(([k,v])=>[k,JSON.stringify(v)]));
 if(id==='det_mod'){
  let a=matrix(input.a);square(a);const m=integer(input.modulus);check(m>0n&&m<(1n<<64n),'法は正のuint64です');a=a.map(row=>row.map(x=>mod(x,m)));let r=1n%m;
  for(let k=0;k<a.length;k++){
   for(let i=k+1;i<a.length;i++)while(a[i][k]){const q=a[k][k]/a[i][k];for(let j=k;j<a.length;j++){a[k][j]=mod(a[k][j]-q*a[i][j],m);[a[k][j],a[i][j]]=[a[i][j],a[k][j]];}r=mod(-r,m);step(`列 ${k} のEuclid消去`,'法の逆元は使用しません。行交換で符号が反転します。',a,[k,k]);}
   r=r*a[k][k]%m;
  }result=[r];
 }else if(id==='det_integer'){
  const a=matrix(input.a);square(a);let previous=1n,sign=1n,singular=false;
  for(let k=0;k+1<a.length;k++){
   let pivot=k;while(pivot<a.length&&!a[pivot][k])pivot++;if(pivot===a.length){singular=true;break;}
   if(pivot!==k){[a[k],a[pivot]]=[a[pivot],a[k]];sign=-sign;}
   for(let i=k+1;i<a.length;i++){for(let j=k+1;j<a.length;j++){const value=a[i][j]*a[k][k]-a[i][k]*a[k][j];check(value%previous===0n,'Bareissの割り切り不変条件');a[i][j]=value/previous;}a[i][k]=0n;}previous=a[k][k];step(`pivot ${k} のBareiss消去`,'前段pivotで割り切れる整数だけを扱います。',a,[k,k]);
  }result=[singular?0n:a.length?sign*a.at(-1).at(-1):1n];
 }else if(id==='semiring'){
  check(['min','max','boolean'].includes(input.operation),'operationはmin/max/booleanです');check(Number.isSafeInteger(input.exponent)&&input.exponent>=0&&input.exponent<=32,'指数は0〜32です');
  const raw=input.a;check(Array.isArray(raw)&&raw.length<=5&&raw.every(row=>Array.isArray(row)&&row.length===raw.length),'5×5以下の正方行列です');
  const boolean=input.operation==='boolean',zero=boolean?0n:null,one=boolean?1n:0n;
  let a=raw.map(row=>row.map(x=>x===null?zero:boolean?(integer(x)?1n:0n):integer(x))),r=Array.from({length:a.length},(_,i)=>Array.from({length:a.length},(_,j)=>i===j?one:zero));
  const plus=(x,y)=>boolean?(x||y?1n:0n):x===null?y:y===null?x:input.operation==='min'?(x<y?x:y):(x>y?x:y);
  const times=(x,y)=>boolean?(x&&y?1n:0n):x===null||y===null?null:x+y;
  const multiply=(x,y)=>x.map((row,i)=>row.map((_,j)=>row.reduce((sum,v,k)=>plus(sum,times(v,y[k][j])),zero)));
  for(let k=input.exponent;k;k=Math.floor(k/2)){if(k%2){r=multiply(r,a);step('現在の答えに掛ける','選択した半環の加算・乗算で行列積を取ります。',r.map(row=>row.map(x=>x===null?'inf':x)));}if(k>1)a=multiply(a,a);}
  result=r.flat().map(x=>x===null?'inf':x);
 }else if(id==='permanent'){
  const a=matrix(input.a);square(a);let r=0n;const n=a.length;
  if(!n)r=1n;
  for(let mask=1;mask<(1<<n);mask++){let bits=0;for(let x=mask;x;x&=x-1)bits++;const sums=a.map(row=>row.reduce((s,x,j)=>s+((mask>>j)&1?x:0n),0n)),term=sums.reduce((p,x)=>p*x,1n);r+=(n-bits)%2?-term:term;step(`列集合 ${mask.toString(2).padStart(n,'0')}`,'説明モデルは集合順。C++はGray codeで行和を差分更新します。',[...sums,r]);}result=[r];
 }else if(id==='pfaffian'||id==='hafnian'){
  const a=matrix(input.a,8);square(a);const n=a.length,p=id==='pfaffian'?field():null;
  for(let i=0;i<n;i++){if(p)check(mod(a[i][i],p)===0n,'交代行列の対角は0です');for(let j=0;j<i;j++)check(p?mod(a[i][j]+a[j][i],p)===0n:a[i][j]===a[j][i],'行列の交代／対称条件を満たしてください');}
  const memo=new Map([[0,1n]]);const solve=mask=>{if(memo.has(mask))return memo.get(mask);let i=0;while(!((mask>>i)&1))i++;let r=0n,pos=0;for(let j=i+1;j<n;j++)if((mask>>j)&1){const term=a[i][j]*solve(mask^(1<<i)^(1<<j));r+=p&&pos%2?-term:term;pos++;}if(p)r=mod(r,p);memo.set(mask,r);step(`頂点集合 ${mask.toString(2).padStart(n,'0')}`,'最小頂点の相手を列挙します。PfaffianのC++版は消去法です。',[r]);return r;};result=[n%2?0n:solve((1<<n)-1)];
 }else if(id==='smith'){
  const a=matrix(input.a,5),m=integer(input.modulus);check(m>0n,'合同式の法は正です');check(Array.isArray(input.b)&&input.b.length===a.length,'右辺の次元が一致しません');
  const rhs=input.b.map(integer);const s=smith(a,Z,(title,a,pivot)=>step(title,'D=UAVを保つ整数基本変形。pivotは残りの成分を割り切ります。',a,pivot));meta=s;
  const n=a.length?a[0].length:0,c=s.u.map(row=>row.reduce((v,x,j)=>v+x*rhs[j],0n)),y=Array(n).fill(0n);let ok=true,count=1n;
  for(let i=0;i<s.rank;i++){if(c[i]%s.a[i][i])ok=false;else y[i]=c[i]/s.a[i][i];const g=gcd(s.a[i][i],m);if(c[i]%g)count=0n;else count*=g;}
  for(let i=s.rank;i<a.length;i++){if(c[i])ok=false;if(c[i]%m)count=0n;}
  for(let i=s.rank;i<n;i++)count*=m;
  const particular=s.v.map(row=>row.reduce((v,x,j)=>v+x*y[j],0n));result=[...Array.from({length:Math.min(a.length,n)},(_,i)=>s.a[i][i]),...(ok?particular:['no integer solution']),count];
  step('対角上で線形系を解く','整数解は割り切り、合同式はgcd条件を検査します。最後の値は剰余解の総数です。',result);
 }else if(id==='hermite'){
  const a=matrix(input.a,5),m=a.length,n=m?a[0].length:0,u=identity(m,Z);let r=0;
  const swap=(i,j)=>{[a[i],a[j]]=[a[j],a[i]];[u[i],u[j]]=[u[j],u[i]];};
  const sub=(i,j,q)=>{a[i]=a[i].map((x,k)=>x-q*a[j][k]);u[i]=u[i].map((x,k)=>x-q*u[j][k]);};
  for(let c=0;c<n&&r<m;c++){
   let p=r;while(p<m&&!a[p][c])p++;if(p===m)continue;swap(r,p);
   for(let i=r+1;i<m;i++)while(a[i][c]){const q=a[i][c]/a[r][c];sub(i,r,q);if(a[i][c])swap(i,r);step('行のEuclid消去','行格子を保ったままpivotを作ります。',a,[r,c]);}
   if(a[r][c]<0n){a[r]=a[r].map(x=>-x);u[r]=u[r].map(x=>-x);}
   for(let i=0;i<r;i++){let q=a[i][c]/a[r][c];if(a[i][c]%a[r][c]<0n)q--;sub(i,r,q);}step('正のpivotと標準剰余','上側の同じ列は0以上pivot未満です。',a,[r,c]);r++;
  }result=a.flat();meta={a,u};
 }else if(id==='gf2'){
  const a=integer(input.a),b=integer(input.b);check(a>=0n&&b>0n&&a<(1n<<256n)&&b<(1n<<256n),'aは非負・bは正、各256bit未満です');
  const degree=x=>x===0n?-1:x.toString(2).length-1;
  const divmod=(x,y,show=false)=>{let q=0n;while(x&&degree(x)>=degree(y)){let shift=BigInt(degree(x)-degree(y));q^=1n<<shift;x^=y<<shift;if(show)step('最高次の項をXORで消去','carryなしの多項式除算です。',[q,x]);}return[q,x];};
  let product=0n;for(let x=a,y=b;y;y>>=1n,x<<=1n)if(y&1n)product^=x;
  const[q,r]=divmod(a,b,true);let x=a,y=b;while(y)[x,y]=[y,divmod(x,y)[1]];result=[product,q,r,x];
 }else if(id==='extension'||id==='frobenius'||id==='charpoly'){
  const p=field(),R=polynomialRing(p);const vector=a=>{check(Array.isArray(a)&&a.length<=8,'多項式は8係数以下です');return R.trim(a.map(x=>mod(integer(x),p)));};
  if(id==='extension'){
   const f=R.monic(vector(input.modulus));check(f.length>=2&&f.length<=5&&p<=17n,'拡大体図解は素数17以下、拡大次数1〜4です');const n=f.length-1,x=R.divmod([0n,1n],f)[1];let h=x;
   for(let i=1;i<=n;i++){h=R.power(h,p,f);if(i<=Math.floor(n/2))check(R.extended(R.sub(h,x),f)[0].length===1,'法多項式が可約です');}check(!R.sub(h,x).length,'法多項式が可約です');
   const a=R.divmod(vector(input.a),f)[1],b=R.divmod(vector(input.b),f)[1];check(a.length,'aの逆元が存在しません');const product=R.divmod(R.mul(a,b),f)[1],inverse=R.divmod(R.extended(a,f)[1],f)[1];
   const pad=v=>Array.from({length:n},(_,i)=>v[i]||0n);result=[...pad(product),...pad(inverse)];step('法多項式で積と逆元を還元','出力は拡大次数と同じ固定長の係数列です。',[pad(product),pad(inverse)]);
  }else{
   const a=matrix(input.a,5).map(row=>row.map(x=>mod(x,p)));square(a);const n=a.length,pencil=a.map((row,i)=>row.map((x,j)=>R.trim(i===j?[mod(-x,p),1n]:[mod(-x,p)])));
   if(id==='frobenius'){
    const s=smith(pencil,R,(title,a,pivot)=>step(title,'xI−Aを多項式Smith形へ変形します。係数は昇冪順です。',a,pivot));const factors=s.a.map((row,i)=>row[i]).filter(f=>f.length>1),form=Array.from({length:n},()=>Array(n).fill(0n));let offset=0;
    for(const f of factors){let d=f.length-1;for(let j=0;j<d;j++){if(j+1<d)form[offset+j+1][offset+j]=1n;form[offset+j][offset+d-1]=mod(-f[j],p);}offset+=d;}
    result=[...factors.flat(),...form.flat()];meta={factors,form};step('不変因子のcompanion block','C++はこの標準形に加え、実際の相似基底と逆行列も返します。',form);
   }else{
    let polynomial=[];const perm=[],used=Array(n).fill(false);
    const visit=()=>{if(perm.length===n){let term=[1n],odd=false;for(let i=0;i<n;i++){term=R.mul(term,pencil[i][perm[i]]);for(let j=0;j<i;j++)if(perm[j]>perm[i])odd=!odd;}polynomial=odd?R.sub(polynomial,term):R.add(polynomial,term);step('順列 '+perm.join(','),'説明モデルはdet(xI−A)の直接展開です。C++はKrylov倍増・BM・Strassenとfallbackを使います。',polynomial);return;}for(let j=0;j<n;j++)if(!used[j]){used[j]=true;perm.push(j);visit();perm.pop();used[j]=false;}};visit();result=polynomial;
   }
  }
 }
 result=result.map(String);step('結果','計算結果の数値は省略せず表示します。',result);
 return {steps,result,meta};
}
const api={modes,trace};if(typeof module!=='undefined')module.exports=api;else root.LinearAlgebraExtensionsModel=api;
})(typeof window!=='undefined'?window:globalThis);
