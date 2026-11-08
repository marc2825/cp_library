(function(root){
'use strict';
const defaults = {
 segmented_sieve:{left:100,right:111,x:108},multiplicative_order:{modulus:360,base:7,order_modulus:40},
 min_mod_linear:{left:0,right:10,a:7,b:3,modulus:11},floor_sum_polynomial:{n:6,m:5,a:3,b:1},
 batch_inversion:{values:[3,5,7],modulus:16},prime_power_binomial:{n:100,k:30,modulus:72},
 large_factorial:{n:1000,prime:1009},discrete_log_table:{base:2,modulus:12,targets:[1,2,4,8,3]},
 dirichlet_prefix_product:{n:20,a:0,b:0},dirichlet_inverse:{n:20,first:1,tail:1},mod_kth_root:{a:4,k:4,prime:17},
 gaussian_integer:{a:[17,4],b:[4,1]},continued_fraction:{numerator:355,denominator:113,bound:100},
 pell_equation:{D:2,sign:1,exponent:3},sum_two_squares:{n:65},prime_sum:{n:100},
 min25_sieve:{n:100,kind:'phi'},multiplicative_summatory:{n:100,kind:'divisors'},
 mod_sum_polynomial:{left:0,right:10,a:7,b:3,modulus:11,p:1,q:2}
};
const titles=['区間篩','乗法位数・Carmichael関数','一次式modの最小値','多項式Floor Sum','Batch Inversion','大引数二項係数','巨大なnの階乗','再利用BSGS','商prefixのDirichlet積','Dirichlet逆元','mod k乗根','Gaussian Integer','連分数・有理数近似','Pell方程式','二平方数和','高速素数和','Min_25 Sieve','乗法的関数の総和','重み付きmod和'];
const convolution=new Set(['dirichlet_prefix_product','dirichlet_inverse','prime_sum','min25_sieve','multiplicative_summatory']);
const modes=Object.keys(defaults).map((id,i)=>({id,title:titles[i],sample:defaults[id],doc:`${convolution.has(id)?'number_theory_convolution':['prime_power_binomial','large_factorial'].includes(id)?'combinatorics':'number_theory_basic'}/${id}.md`}));
function integer(x,lo,hi,name){if(!Number.isSafeInteger(x)||x<lo||x>hi)throw Error(`${name}: ${lo}..${hi} の整数を指定してください`);return x;}
const gcd=(a,b)=>{while(b){[a,b]=[b,a%b];}return a;};
const mod=(a,m)=>(a%m+m)%m;
const pow=(a,k,m)=>{a=BigInt(a);k=BigInt(k);m=BigInt(m);let r=1n%m;for(;k;k>>=1n,a=a*a%m)if(k&1n)r=r*a%m;return r;};
const factor=n=>{const r=[];for(let p=2;p*p<=n;p++)if(n%p===0){let e=0;while(n%p===0){n/=p;e++;}r.push([p,e]);}if(n>1)r.push([n,1]);return r;};
const isPrime=n=>n>1&&factor(n).length===1&&factor(n)[0][1]===1;
function inv(a,m){a=mod(a,m);for(let x=0;x<m;x++)if(a*x%m===1%m)return x;return null;}
const quotients=n=>{const r=[];for(let l=1;l<=n;){const x=Math.floor(n/l);r.push(x);l=Math.floor(n/x)+1;}return r;};
const fd=(a,b)=>{let q=a/b;if(a%b<0n)q--;return q;};
const norm=z=>z[0]*z[0]+z[1]*z[1];
const gm=(a,b)=>[a[0]*b[0]-a[1]*b[1],a[0]*b[1]+a[1]*b[0]];
function gd(a,b){const d=norm(b);if(!d)throw Error('除数は0以外にしてください');const t=gm(a,[b[0],-b[1]]),q=t.map(x=>fd(2n*x+d,2n*d)),v=gm(b,q);return [q,a.map((x,i)=>x-v[i])];}
function trace(id,input){
 if(!modes.some(m=>m.id===id)||!input||typeof input!=='object'||Array.isArray(input))throw Error('モードと入力オブジェクトを確認してください');
 const steps=[];let result=[];
 function step(title,detail,columns=[],rows=[],values={},plot=null){steps.push({title,detail,columns,rows:rows.map(row=>row.map(String)),values:Object.entries(values).map(([label,value])=>({label,value:String(value)})),plot,state:null});}
 step('入力を確認','値を編集して実行すると、この手順を先頭から再計算します。',[],[],{モード:modes.find(m=>m.id===id).title});
 const I=(key,lo,hi)=>integer(input[key],lo,hi,key);
 if(id==='segmented_sieve'){
  const l=I('left',0,1000000),r=I('right',l,l+60),x=I('x',Math.max(l,1),r-1),remaining=Array.from({length:r-l},(_,i)=>i+l),factors=remaining.map(()=>[]);
  for(let p=2;p*p<r;p++)if(isPrime(p)){
   for(let i=0;i<remaining.length;i++)if(remaining[i]>1&&remaining[i]%p===0){let e=0;while(remaining[i]%p===0){remaining[i]/=p;e++;}factors[i].push([p,e]);}
   step(`素数 ${p} で除く`,'区間内の倍数だけを割り、指数を記録します。',['整数','未分解の因子','記録した素因数'],remaining.map((v,i)=>[l+i,v,factors[i].map(([p,e])=>`${p}^${e}`).join(' × ')]));
  }
  remaining.forEach((v,i)=>{if(v>1)factors[i].push([v,1]);});
  const primes=remaining.map((_,i)=>l+i).filter(isPrime);result=[...primes,...factors[x-l].flat()];
  step('区間の分解を完了','残った1より大きい因子は素数です。',['整数','分解'],factors.map((f,i)=>[l+i,f.map(([p,e])=>`${p}^${e}`).join(' × ')]),{素数の個数:primes.length});
 }else if(id==='multiplicative_order'){
  const m=I('modulus',1,10000),a=I('base',0,10000),om=I('order_modulus',1,1000);let lambda=1;
  for(const [p,e]of factor(m)){let v=(p-1)*p**(e-1);if(p===2&&e>=3)v/=2;lambda=lambda/gcd(lambda,v)*v;step(`λ(${p}^${e})=${v}`,'各素数冪のλをLCMで合成します。',[],[],{累積λ:lambda});}
  let order=-1;
  if(gcd(a,om)===1){let value=1%om;for(let k=1;k<=om;k++){value=value*a%om;step(`指数 ${k}`,'単位元へ初めて戻る正の指数が位数です。',['k','a^k mod m'],[[k,value]]);if(value===1%om){order=k;break;}}}
  result=[lambda,order];
 }else if(['min_mod_linear','floor_sum_polynomial','mod_sum_polynomial'].includes(id)){
  const l=id==='floor_sum_polynomial'?0:I('left',-100,100),r=id==='floor_sum_polynomial'?I('n',0,60):I('right',l,l+60),m=id==='floor_sum_polynomial'?I('m',1,1000):I('modulus',1,1000),a=I('a',-100,100),b=I('b',-1000,1000);
  let sum=0n,weighted=0n,squares=0n,best=m,index=l;const rows=[],plot=[];
  const p=id==='mod_sum_polynomial'?I('p',0,3):0,q=id==='mod_sum_polynomial'?I('q',0,3):0;
  for(let i=l;i<r;i++){const y=Math.floor((a*i+b)/m),rem=mod(a*i+b,m);let term;
   if(id==='floor_sum_polynomial'){sum+=BigInt(y);weighted+=BigInt(i*y);squares+=BigInt(y*y);term=y;rows.push([i,y,i*y,y*y]);}
   else if(id==='mod_sum_polynomial'){term=BigInt(i)**BigInt(p)*BigInt(rem)**BigInt(q);sum+=term;rows.push([i,rem,term,sum]);}
   else{term=rem;if(rem<best){best=rem;index=i;}rows.push([i,a*i+b,y,rem]);}
   plot.push([i,Number(term)]);step(`i = ${i}`,id==='min_mod_linear'?'剰余の点を追って、最初の最小値を確認します。':'添字と床値・剰余を使って部分和を更新します。',id==='floor_sum_polynomial'?['i','floor','i·floor','floor²']:id==='mod_sum_polynomial'?['i','剰余','寄与','部分和']:['i','a*i+b','floor','剰余'],rows.slice(),{部分和:sum,最小剰余:best},plot.slice());
  }
  result=id==='floor_sum_polynomial'?[sum,weighted,squares]:id==='mod_sum_polynomial'?[sum]:l===r?['empty']:[best,index];
 }else if(id==='batch_inversion'){
  const m=I('modulus',1,1000);if(!Array.isArray(input.values)||input.values.length>30)throw Error('values は長さ30以下の配列です');
  const a=input.values.map(x=>integer(x,0,10000,'value')),prefix=[1%m];
  for(const x of a)prefix.push(prefix.at(-1)*x%m);
  step('prefix積を構築','積全体の逆元を1回だけ求めます。',['i','a[i]','prefix[i+1]'],a.map((v,i)=>[i,v,prefix[i+1]]));
  let current=inv(prefix.at(-1),m);result=Array(a.length);
  if(current===null)result=['none'];else for(let i=a.length-1;i>=0;i--){result[i]=current*prefix[i]%m;step(`逆順 i=${i}`,'prefixと残りの積の逆元から、1要素の逆元が決まります。',['i','a[i]','逆元'],[[i,a[i],result[i]]]);current=current*a[i]%m;}
 }else if(id==='prime_power_binomial'){
  const n=I('n',0,1000000),k=I('k',0,100),m=I('modulus',1,1000);let choose=1n;
  if(k>n)choose=0n;else for(let i=1;i<=Math.min(k,n-k);i++)choose=choose*BigInt(n-i+1)/BigInt(i);
  function valuation(n,p){let v=0;while(n){n=Math.floor(n/p);v+=n;}return v;}
  for(const [p,e]of factor(m)){const pe=p**e,prefix=[1];for(let i=1;i<=pe;i++)prefix.push(i%p?prefix.at(-1)*i%pe:prefix.at(-1));
   function unit(n){let r=1;while(n){r=r*Number(pow(prefix[pe],Math.floor(n/pe),pe))%pe*prefix[n%pe]%pe;n=Math.floor(n/p);}return r;}
   const v=k>n?e:valuation(n,p)-valuation(k,p)-valuation(n-k,p),numerator=unit(n),denominator=k>n?1:unit(k)*unit(n-k)%pe,residue=v>=e?0:numerator*inv(denominator,pe)%pe*Number(pow(p,v,pe))%pe;
   step(`素数冪 ${pe}`,'pの指数と、pを除いた階乗の比を別々に計算します。',['p','e','二項係数のp指数','分子unit','分母unit','剰余'],[[p,e,v,numerator,denominator,residue]]);
  }result=[choose%BigInt(m)];
 }else if(id==='large_factorial'){
  const n=I('n',0,5000),p=I('prime',2,100000);if(!isPrime(p))throw Error('prime は素数です');let answer=1;
  if(n>=p)answer=0;else{const complement=n>(p-1)/2,r=Math.min(n,p-1-n),block=Math.max(1,Math.floor(Math.sqrt(r)));let rows=[];
   for(let start=1;start<=r;start+=block){let product=1;const end=Math.min(r,start+block-1);for(let j=start;j<=end;j++)product=product*j%p;answer=answer*product%p;rows.push([start,end,product,answer]);step(`ブロック ${start}..${end}`,'C++は積多項式の多点評価で全ブロックを求めます。図解では同じ積を直接計算します。',['先頭','末尾','ブロック積','累積積'],rows.slice());}
   if(complement){answer=Number(pow(answer,p-2,p));if(r%2===0)answer=p-answer;step('Wilsonの定理で復元','n!·(-1)^r·r! ≡ -1。',[],[],{短い側:r,結果:answer});}
  }result=[answer];
 }else if(id==='discrete_log_table'){
  const original=I('modulus',1,1000),base=I('base',0,10000)%original;if(!Array.isArray(input.targets)||input.targets.length>20)throw Error('targets は20個以下です');
  const targets=input.targets.map(x=>integer(x,0,10000,'target'));let m=original,factor=1%m;const stages=[];
  for(;;){const g=gcd(base,m);stages.push({m,g,factor});if(g===1)break;m/=g;factor=factor*(base/g)%m;}
  const width=Math.ceil(Math.sqrt(m)),baby=new Map();let value=1%m;for(let j=0;j<width;j++){if(!baby.has(value))baby.set(value,j);value=value*base%m;}
  step('共通のbaby table','底と法を固定し、各問い合わせでこの表を再利用します。',['剰余','最小指数'],[...baby]);
  result=targets.map(target=>{let b=target%original,offset=0,answer=-1,stopped=false;for(const s of stages){if(b===s.factor){answer=offset;stopped=true;break;}if(s.g===1)break;if(b%s.g){stopped=true;break;}b/=s.g;offset++;}
   if(!stopped){let current=b*inv(factor,m)%m;const stepValue=inv(value,m);for(let i=0;i<=width;i++){if(baby.has(current)){answer=offset+i*width+baby.get(current);break;}current=current*stepValue%m;}}
   step(`target=${target}`,'gcdの列で前半を確認した後、giant stepでbaby tableと照合します。',['target','最小指数'],[[target,answer]]);return answer;});
 }else if(id==='dirichlet_prefix_product'){
  const n=I('n',0,200),a=I('a',0,2),b=I('b',0,2),f=[0n],g=[0n];for(let i=1;i<=n;i++){f.push(f.at(-1)+BigInt(i)**BigInt(a));g.push(g.at(-1)+BigInt(i)**BigInt(b));}
  const table=[];for(const x of quotients(n)){const s=Math.floor(Math.sqrt(x));let value=0n;for(let i=1;i<=s;i++)value+=(f[i]-f[i-1])*g[Math.floor(x/i)]+(g[i]-g[i-1])*f[Math.floor(x/i)];const correction=f[s]*g[s];value-=correction;table.push([x,s,correction,value]);step(`商 x=${x}`,'双曲線を2方向から数え、正方形の重複を1回引きます。',['x','√x','重複分','積のprefix'],table.slice());}result=[table[0]?.[3]??0];
 }else if(id==='dirichlet_inverse'){
  const n=I('n',0,100),first=I('first',-1,1),tail=I('tail',-3,3);if(first===0&&n)throw Error('整数環でf(1)は±1です');const g=Array(n+1).fill(0n),prefix=Array(n+1).fill(0n);
  for(let i=1;i<=n;i++){let sum=i===1?1n:0n;for(let d=2;d<=i;d++)if(i%d===0)sum-=BigInt(tail)*g[i/d];g[i]=sum*BigInt(first);prefix[i]=prefix[i-1]+g[i];step(`g(${i}) を復元`,'f*g=δとなるよう、既知の小さい添字の寄与を引きます。',['i','g(i)','prefix'],[[i,g[i],prefix[i]]]);}result=[...g.slice(1,11),prefix[n]];
 }else if(id==='mod_kth_root'){
  const p=I('prime',2,200),k=I('k',0,100),a=I('a',0,10000)%p;if(!isPrime(p))throw Error('prime は素数です');const roots=[];for(let x=0;x<p;x++){const y=pow(x,k,p);if(y===BigInt(a))roots.push(x);step(`候補 x=${x}`,'図解では全候補のべき乗を確認します。C++は離散対数の線形合同式を解きます。',['x','x^k mod p','一致'],[[x,y,y===BigInt(a)?'✓':'']],{解の個数:roots.length});}result=roots;
 }else if(id==='gaussian_integer'){
  function z(key){if(!Array.isArray(input[key])||input[key].length!==2)throw Error('Gaussian整数は[実部,虚部]です');return input[key].map(x=>BigInt(integer(x,-10000,10000,key)));}
  let a=z('a'),b=z('b');const [quotient,remainder]=gd(a,b);step('最近整数へ丸める','実部と虚部を別々に丸め、normを減らします。',['q.real','q.imag','r.real','r.imag'],[[...quotient,...remainder]]);
  while(norm(b)){const [q,r]=gd(a,b);step('Gaussian Euclid','a=b*q+r、次は(b,r)です。',['a','b','q','r','norm(r)'],[[a,b,q,r,norm(r)]]);a=b;b=r;}
  if(norm(a))while(a[0]<=0n||a[1]<0n)a=[-a[1],a[0]];result=[...quotient,...remainder,...a];
 }else if(id==='continued_fraction'){
  const numerator=I('numerator',-1000000,1000000),denominator=I('denominator',1,1000000),bound=I('bound',1,200);let n=BigInt(numerator),d=BigInt(denominator),p0=0n,p1=1n,q0=1n,q1=0n;const coefficients=[],rows=[];
  while(d){const a=fd(n,d),r=n-a*d,p=a*p1+p0,q=a*q1+q0;coefficients.push(a);rows.push([a,p,q]);step('Euclidの商と収束分数','p=a*p₋₁+p₋₂、q=a*q₋₁+q₋₂。',['係数','分子','分母'],rows.slice());[n,d]=[d,r];[p0,p1,q0,q1]=[p1,p,q1,q];}
  let best=null;for(let q=1;q<=bound;q++){const near=Number(fd(BigInt(numerator*q),BigInt(denominator)));for(let p=near;p<=near+1;p++){const g=gcd(Math.abs(p),q),v=[p/g,q/g],error=BigInt(Math.abs(v[0]*denominator-numerator*v[1]));if(!best||error*BigInt(best[1])<best[2]*BigInt(v[1])||(error*BigInt(best[1])===best[2]*BigInt(v[1])&&(v[1]<best[1]||(v[1]===best[1]&&v[0]<best[0]))))best=[...v,error];}}
  step('分母制限で近似','小さいモデルは全分母を比較します。C++は収束分数と半収束分数だけを比較します。',[],[],{近似:`${best[0]}/${best[1]}`});result=[...coefficients,`${best[0]}/${best[1]}`];
 }else if(id==='pell_equation'){
  const D=I('D',2,1000),sign=I('sign',-1,1),exponent=I('exponent',0,10),root=Math.floor(Math.sqrt(D));if(root*root===D||sign===0)throw Error('Dは非平方数、signは±1です');let m=0n,d=1n,a=BigInt(root),p0=0n,p1=1n,q0=1n,q1=0n,solution=null;
  for(let i=0;i<1000;i++){const p=a*p1+p0,q=a*q1+q0,norm=p*p-BigInt(D)*q*q;step(`収束分数 ${i+1}`,'x²−D*y²が指定した符号になる最初の解を探します。',['a','x','y','norm'],[[a,p,q,norm]]);if(norm===BigInt(sign)){solution=[p,q];break;}if(norm===1n&&sign===-1)break;[p0,p1,q0,q1]=[p1,p,q1,q];m=d*a-m;d=(BigInt(D)-m*m)/d;a=(BigInt(root)+m)/d;}
  if(!solution)result=['none'];else{let [x,y]=[1n,0n];for(let i=0;i<exponent;i++)[x,y]=[x*solution[0]+BigInt(D)*y*solution[1],x*solution[1]+y*solution[0]];result=[...solution,x,y];}
 }else if(id==='sum_two_squares'){
  const n=I('n',0,10000),rows=[],points=[];step('素因数条件','3 mod 4の素因数は偶数指数が必要です。',['素数','指数'],factor(n));
  for(let x=0;x*x<=n;x++){const y=Math.floor(Math.sqrt(n-x*x));if(y<x)break;if(x*x+y*y===n){rows.push([x,y]);points.push([x,y]);}step(`x=${x}`,'図解では平方根を直接確認します。C++はGaussian整数の因子分配から全表示を生成します。',['x','y'],rows.slice(),{表示数:rows.length},points.length?points.slice():null);}result=rows.flat();
 }else if(['prime_sum','min25_sieve','multiplicative_summatory'].includes(id)){
  const n=I('n',0,10000),qs=quotients(n),primes=Array.from({length:Math.max(0,n-1)},(_,i)=>i+2).filter(isPrime),kind=id==='prime_sum'?'prime':input.kind;
  if(!['prime','phi','divisors','sigma','squares','residue'].includes(kind))throw Error('kindはphi/divisors/sigma/squares/residueです');
  const primeWeight=p=>kind==='prime'?BigInt(p):kind==='phi'?BigInt(p-1):kind==='divisors'?2n:kind==='sigma'?BigInt(p+1):kind==='squares'?BigInt(p*p):p%4===1?2n:-1n;
  const primePower=(p,e,power)=>kind==='phi'?BigInt(power/p*(p-1)):kind==='divisors'?BigInt(e+1):kind==='sigma'?(BigInt(p)**BigInt(e+1)-1n)/BigInt(p-1):kind==='squares'?BigInt(power)**2n:(p%4===1?2n:-1n)**BigInt(e);
  let table=new Map();const small=primes.filter(p=>p*p<=n);
  if(kind==='prime'){
   for(const x of qs)table.set(x,BigInt(x)*(BigInt(x)+1n)/2n-1n);table.set(0,0n);
   for(const p of small){const before=table.get(p-1);for(const x of qs){if(x<p*p)break;table.set(x,table.get(x)-BigInt(p)*(table.get(Math.floor(x/p))-before));}step(`素数 ${p} の倍数を除く`,'pより小さい素因数を持たない倍数だけを引きます。',['商','素数候補の和'],qs.map(x=>[x,table.get(x)]));}
  }else{
   const prefix=Array(n+1).fill(0n);let at=0;for(let x=1;x<=n;x++){prefix[x]=prefix[x-1];if(primes[at]===x){prefix[x]+=primeWeight(x);at++;}}
   for(const x of qs)table.set(x,prefix[x]);step('素数項のprefixを用意','Min_25は多項式の素数和、汎用版は指定された素数和を使います。',['商','素数prefix'],qs.map(x=>[x,table.get(x)]));
   for(const p of small.reverse()){for(const x of qs){if(x<p*p)break;let v=table.get(x);for(let power=p,e=1;power*p<=x;power*=p,e++)v+=primePower(p,e,power)*(table.get(Math.floor(x/power))-prefix[p])+primePower(p,e+1,power*p);table.set(x,v);}step(`最小素因数 ${p} を戻す`,'素数を逆順に処理し、pの冪と、それより大きい素因数の部分を合成します。',['商','1を除く総和'],qs.map(x=>[x,table.get(x)]));}
   for(const x of qs)table.set(x,table.get(x)+1n);
  }result=[table.get(n)??0n];
 }
 result=result.map(String);step('計算結果','出力は実行例と同じ順序です。C++と図解のサンプル値を検証で照合しています。',['出力'],result.map(x=>[x]));return {steps,result};
}
const api={modes,trace};if(typeof module!=='undefined'&&module.exports)module.exports=api;root.NumberTheoryExtensionsModel=api;
})(typeof window!=='undefined'?window:globalThis);
