(function(root){
'use strict';
const MODES=['ntt','convolution','integer','fft-real','fft-complex','convolution-fft'];
const primes={'17':3,'97':5,'998244353':3};
const clone=x=>JSON.parse(JSON.stringify(x,(_,v)=>typeof v==='bigint'?String(v):v));
const mod=(x,p)=>(x%p+p)%p;
function power(x,e,p){let r=1n;for(;e;e>>=1n,x=x*x%p)if(e&1n)r=r*x%p;return r;}
const add=(x,y)=>[x[0]+y[0],x[1]+y[1]],sub=(x,y)=>[x[0]-y[0],x[1]-y[1]],mul=(x,y)=>[x[0]*y[0]-x[1]*y[1],x[0]*y[1]+x[1]*y[0]];
function defaults(mode){return {mode,a:mode==='fft-complex'?'1,1 2,-1 3,0 4,2':mode==='convolution-fft'?'0.5 -1 2':'1 2 3 4',b:mode==='convolution-fft'?'2 3':'4 5',modulus:'17',direction:'forward',kind:'real',focus:'2'};}
function scenario(mode,name){const c=defaults(mode);
 if(name==='empty'){c.a='';c.b='1';}
 if(name==='one'){c.a=mode==='fft-complex'?'2,-1':'2';c.b='-3';}
 if(name==='impulse'){c.a='0 1 0 0';c.b='1 0 0';}
 if(name==='negative'){c.a=mode==='fft-complex'?'1,-1 -2,3 3,-2 -4,0':'1 -2 3 -4';c.b='-2 3 -1';}
 if(name==='padding'){c.a='1 2 3';c.b='4 5 6';}
 if(name==='inverse'){c.a=mode==='fft-complex'?'10,0 -2,-2 -2,0 -2,2':'10 6 15 7';c.direction='inverse';}
 if(name==='mod-large'){c.modulus='998244353';c.a='998244352 -2 3 0';}
 if(name==='integer-limit'){c.a='-9223372036854775808';c.b='1';}
 if(name==='overflow'){c.a='9223372036854775807';c.b='2';}
 if(name==='complex'){c.kind='complex';c.a='1,1 2,-1 0,2';c.b='0,1 1,0';}
 if(name==='cancellation'){c.a='1 0 -1 0';c.b='1 1';}
 return c;
}
function parse(raw){
 const c={...defaults(raw.mode),...raw};if(!MODES.includes(c.mode))throw Error('モードが不正です。');
 const integer=['ntt','convolution','integer'].includes(c.mode),single=['ntt','fft-real','fft-complex'].includes(c.mode);
 const complex=c.mode==='fft-complex'||c.mode==='convolution-fft'&&c.kind==='complex';
 if(!['forward','inverse'].includes(c.direction)||!['real','complex'].includes(c.kind))throw Error('計算方法が不正です。');
 function row(value){const tokens=String(value).trim()?String(value).trim().split(/\s+/):[];
  if(tokens.length>(single?16:8))throw Error(single?'変換は16要素までです。':'畳み込みの各列は8要素までです。');
  return tokens.map(token=>{if(integer){if(!/^[+-]?\d+$/.test(token))throw Error('整数を空白で区切って入力してください。');const x=BigInt(token);if(x<-(1n<<63n)||x>=(1n<<63n))throw Error('係数は符号付き64ビットの範囲にしてください。');return String(x);}
   const parts=token.split(',');if(parts.length>(complex?2:1)||parts.some(x=>!x.trim()||!Number.isFinite(Number(x))||Math.abs(Number(x))>1e6))throw Error(complex?'係数は実部,虚部で、各成分を±1000000以内にしてください。':'実数係数を±1000000以内で入力してください。');return [Number(parts[0]),Number(parts[1]||0)];});
 }
 c.a=row(c.a);c.b=single?[]:row(c.b);c.single=single;c.integer=integer;c.complex=complex;
 if(single&&c.a.length&&(c.a.length&(c.a.length-1)))throw Error('変換の長さは2冪にしてください。畳み込みは自動0埋めします。');
 if(!Object.hasOwn(primes,String(c.modulus)))throw Error('法は17、97、998244353から選んでください。');c.modulus=String(c.modulus);
 c.focus=Number(c.focus);if(!Number.isInteger(c.focus)||c.focus<0||c.focus>15)throw Error('注目する係数は0〜15にしてください。');
 c.size=single?c.a.length:c.a.length&&c.b.length?c.a.length+c.b.length-1:0;c.padded=c.size?1:0;while(c.padded<c.size)c.padded*=2;
 if(c.mode==='integer'&&c.size){
  const abs=x=>x<0n?-x:x,stats=a=>a.map(BigInt).map(abs).reduce((r,x)=>[r[0]+x,r[1]>x?r[1]:x],[0n,0n]);const a=stats(c.a),b=stats(c.b);c.bound=String(a[0]*b[1]<b[0]*a[1]?a[0]*b[1]:b[0]*a[1]);
  if(BigInt(c.bound)>1n<<63n)throw Error('整数版の絶対値上限が2^63を超えます。打ち消しがあっても拒否します。');
 }
 return c;
}
function run(raw){const c=parse(raw),frames=[],arrays={},state={stage:'initial',phase:'入力',title:'入力と長さを確認',description:'係数は定数項から昇冪順です。',bank:'A',arrays,pair:[],length:0,first:0,j:0,inverse:false,modulus:null,root:null,twiddle:null,u:null,v:null,left:null,right:null,result:[],residues:[],crt:[],checkpoint:true};
 const emit=(title,description,checkpoint=false)=>{state.title=title;state.description=description;state.checkpoint=checkpoint;frames.push(clone(state));};
 arrays.A=clone(c.a);arrays.B=clone(c.b);emit(state.title,state.description,true);
 function trace(input,bank,inverse,p=null,g=3){
  let a=p?input.map(x=>mod(BigInt(x),p)):clone(input);arrays[bank]=a;Object.assign(state,{bank,inverse,modulus:p,root:g,stage:'normalize',phase:inverse?'逆変換':'順変換',pair:[],length:0,twiddle:null,u:null,v:null,left:null,right:null});
  emit(`${bank}: ${inverse?'逆':'順'}変換の開始`,p?'係数を法の範囲へ正規化します。':'複素数の実部・虚部を両方保持します。',true);
  const n=a.length;
  for(let i=1,j=0;i<n;++i){let bit=n>>1;for(;j&bit;bit>>=1)j^=bit;j^=bit;if(i<j){[a[i],a[j]]=[a[j],a[i]];state.stage='bit-reversal';state.pair=[i,j];emit(`${bank}: 添字 ${i} と ${j} を交換`,'添字のビットを反転した順へ並べます。');}}
  state.stage='permuted';state.pair=[];emit(`${bank}: ビット反転の完了`,'これから長さ2、4、8…のブロックを合成します。',true);
  for(let len=2;len<=n;len*=2){state.length=len;const root=p?power(BigInt(g),(p-1n)/BigInt(len),p):[Math.cos((inverse?-2:2)*Math.PI/len),Math.sin((inverse?-2:2)*Math.PI/len)];const step=p&&inverse?power(root,p-2n,p):root;
   for(let first=0;first<n;first+=len){let w=p?1n:[1,0];for(let j=0;j<len/2;++j){const i=first+j,k=i+len/2,u=a[i],v=p?a[k]*w%p:mul(a[k],w),left=p?mod(u+v,p):add(u,v),right=p?mod(u-v,p):sub(u,v);a[i]=left;a[k]=right;Object.assign(state,{stage:'butterfly',first,j,pair:[i,k],twiddle:w,u,v,left,right});emit(`${bank}: 長さ${len} / (${i}, ${k})`,'上はu+w×v、下はu−w×v。同じ2要素を同時に更新します。');w=p?w*step%p:mul(w,step);}}
   state.stage='stage-end';state.pair=[];state.twiddle=null;emit(`${bank}: 長さ${len}の合成を完了`,'各ブロックは、ビット反転で割り当てられた部分列の変換になっています。',true);
  }
  if(inverse){state.stage='scale';state.pair=[];const scale=p?power(BigInt(n),p-2n,p):n;for(let j=0;j<n;++j)a[j]=p?a[j]*scale%p:[a[j][0]/n,a[j][1]/n];emit(`${bank}: ${p?'nの逆元を掛ける':'nで割る'}`,'逆変換の正規化を行います。',true);}
  state.stage='transform-end';emit(`${bank}: 変換の完了`,'自然な添字順に全成分が得られます。',true);return a;
 }
 const pad=(a,p)=>{const r=p?a.map(BigInt):clone(a);while(r.length<c.padded)r.push(p?0n:[0,0]);return r;};
 function convolve(p=null,g=3){
  const a=pad(c.a,p),b=pad(c.b,p);arrays.A=a;arrays.B=b;arrays.C=[];Object.assign(state,{stage:'padding',phase:'0埋め',modulus:p,pair:[],bank:'A'});emit(`結果長${c.size} → 変換長${c.padded}`,'必要な結果が折り返さないよう、十分な2冪長へ0を追加します。',true);
  const x=trace(a,'A',false,p,g),y=trace(b,'B',false,p,g);arrays.C=[];state.bank='C';state.phase='各点積';state.stage='pointwise';state.pair=[];state.twiddle=null;
  for(let i=0;i<c.padded;++i){arrays.C.push(p?x[i]*y[i]%p:mul(x[i],y[i]));state.j=i;emit(`各点積: C[${i}]=A[${i}]×B[${i}]`,'同じ単位根で評価した値を掛けます。');}
  const z=trace(arrays.C,'C',true,p,g);z.length=c.size;state.stage='trim';state.pair=[];emit('結果長へ戻す','0埋めした余分な末尾を取り除きます。',true);return z;
 }
 if(!c.size){state.stage='done';state.phase='完了';state.result=[];emit('空の結果','空の変換、または空を含む畳み込みは空です。',true);return {config:c,frames};}
 if(c.single){state.result=trace(c.a,'A',c.mode!=='fft-real'&&c.direction==='inverse',c.mode==='ntt'?BigInt(c.modulus):null,primes[c.modulus]);}
 else if(c.mode==='integer'){
  const ps=[167772161n,469762049n,1224736769n],rows=[];state.phase='CRT';
  for(const p of ps){const row=convolve(p);rows.push([...row]);state.residues.push({modulus:p,values:[...row]});emit(`mod ${p} の結果を保存`,'3つの異なる素数で同じ畳み込みを計算します。',true);}
  const [p0,p1,p2]=ps,P=p0*p1*p2;state.result=[];state.bank='C';arrays.C=[];state.modulus=null;state.stage='crt';state.phase='CRT復元';state.pair=[];state.twiddle=null;
  for(let i=0;i<c.size;++i){const r0=rows[0][i],t1=mod((rows[1][i]-r0)*power(p0,p1-2n,p1),p1),low=r0+p0*t1,t2=mod((rows[2][i]-low)*power(p0*p1%p2,p2-2n,p2),p2),residue=low+p0*p1*t2,value=residue>P/2n?residue-P:residue;
   if(value<-(1n<<63n)||value>=(1n<<63n))throw Error('復元した係数がlong longの範囲を超えます。');state.result.push(value);arrays.C.push(value);state.crt.push({i,r0,t1,t2,residue,value});state.j=i;emit(`係数${i}をCRTで復元`,'混合基数の桁を求め、P/2より大きければPを引いて負の値へ戻します。',true);}
 }else {
  const result=convolve(c.mode==='convolution'?BigInt(c.modulus):null,primes[c.modulus]);
  state.result=c.mode==='convolution-fft'&&c.kind==='real'?result.map(x=>x[0]):result;
 }
 state.stage='done';state.phase='完了';state.pair=[];state.twiddle=null;emit('計算の完了',c.integer?'整数演算による正確な結果です。':'浮動小数点の近似値です。小さい虚部や誤差が残る場合があります。',true);return {config:c,frames};
}
const api={MODES,defaults,scenario,parse,run};if(typeof module!=='undefined'&&module.exports)module.exports=api;else root.ConvolutionModel=api;
})(typeof globalThis!=='undefined'?globalThis:this);
