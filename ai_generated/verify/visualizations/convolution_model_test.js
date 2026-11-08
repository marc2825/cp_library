'use strict';
const assert=require('node:assert/strict'),M=require('../../visualizations/convolution/model.js');
const mod=(x,p)=>(x%p+p)%p,add=(x,y)=>[x[0]+y[0],x[1]+y[1]],mul=(x,y)=>[x[0]*y[0]-x[1]*y[1],x[0]*y[1]+x[1]*y[0]];
function power(x,e,p){let r=1n;while(e){if(e%2n)r=r*x%p;e/=2n;x=x*x%p;}return r;}
function near(a,b){if(Array.isArray(a))return Math.hypot(a[0]-b[0],a[1]-b[1])<=2e-7*(1+Math.hypot(...b));if(typeof a==='number')return Math.abs(a-b)<=2e-7*(1+Math.abs(b));return BigInt(a)===BigInt(b);}
function rowEqual(a,b){assert.equal(a.length,b.length);for(let i=0;i<a.length;++i)assert.ok(near(a[i],b[i]),`${a[i]} != ${b[i]}`);}
function direct(a,p,root,inverse=false){const n=a.length,out=[];for(let k=0;k<n;++k){let sum=p?0n:[0,0];for(let j=0;j<n;++j){if(p){const e=BigInt((inverse?(n-j*k%n)%n:j*k%n));sum=mod(sum+BigInt(a[j])*power(BigInt(root),BigInt(Number(p-1n)/n)*e,p),p);}else{const angle=(inverse?-2:2)*Math.PI*j*k/n;sum=add(sum,mul(a[j],[Math.cos(angle),Math.sin(angle)]));}}out.push(inverse?(p?sum*power(BigInt(n),p-2n,p)%p:[sum[0]/n,sum[1]/n]):sum);}return out;}
function product(a,b,p=null){if(!a.length||!b.length)return [];const field=typeof a[0]==='string'||typeof a[0]==='bigint',result=Array.from({length:a.length+b.length-1},()=>field?0n:[0,0]);for(let i=0;i<a.length;++i)for(let j=0;j<b.length;++j){result[i+j]=field?result[i+j]+BigInt(a[i])*BigInt(b[j]):add(result[i+j],mul(a[i],b[j]));}return p?result.map(x=>mod(x,p)):result;}
function reverse(x,bits){let r=0;for(let j=0;j<bits;++j){r=r*2+(x&1);x>>=1;}return r;}
let batches=0,snapshots=0,rejections=0;
function verify(raw){const batch=M.run(raw),c=batch.config,last=batch.frames.at(-1);++batches;snapshots+=batch.frames.length;assert.equal(last.stage,'done');
 let expected=c.single?direct(c.a,c.mode==='ntt'?BigInt(c.modulus):null,{'17':3,'97':5,'998244353':3}[c.modulus],c.mode!=='fft-real'&&c.direction==='inverse'):product(c.a,c.b,c.mode==='convolution'?BigInt(c.modulus):null);if(c.mode==='convolution-fft'&&c.kind==='real')expected=expected.map(x=>x[0]);rowEqual(last.result,expected);
 let input=null,permuted=null,lastFrame=null;
 for(const f of batch.frames){const p=f.modulus?BigInt(f.modulus):null,a=f.arrays[f.bank]||[];
  if(f.stage==='normalize') {input=a.map(x=>p?BigInt(x):x.slice());const bits=Math.log2(a.length);permuted=Array.from({length:a.length},(_,i)=>input[reverse(i,bits)]);}
  if(f.stage==='permuted')rowEqual(a,permuted);
  if(f.stage==='stage-end'){
   const L=f.length,nbits=Math.log2(a.length),lbits=Math.log2(L),oracle=[];
   for(let first=0;first<a.length;first+=L){const original=Array.from({length:L},(_,j)=>input[reverse(first+reverse(j,lbits),nbits)]);oracle.push(...direct(original,p,f.root,f.inverse));}
   // Unscaled inverse sub-transforms: the direct inverse includes 1/L.
   rowEqual(a,f.inverse?oracle.map(x=>p?x*BigInt(L)%p:[x[0]*L,x[1]*L]):oracle);
  }
  if(f.stage==='transform-end')rowEqual(a,direct(input,p,f.root,f.inverse));
  if(f.stage==='butterfly'){
   const [i,k]=f.pair;assert.equal(k-i,f.length/2);assert.equal(i,f.first+f.j);rowEqual([a[i],a[k]],[f.left,f.right]);
   const u=p?BigInt(f.u):f.u,v=p?BigInt(f.v):f.v;rowEqual([f.left,f.right],p?[mod(u+v,p),mod(u-v,p)]:[add(u,v),[u[0]-v[0],u[1]-v[1]]]);
   if(lastFrame&&lastFrame.bank===f.bank){for(let x=0;x<a.length;++x)if(x!==i&&x!==k)assert.deepEqual(a[x],lastFrame.arrays[f.bank][x]);}
  }
  for(const r of f.residues){rowEqual(r.values,product(c.a,c.b,BigInt(r.modulus)));}
  for(const t of f.crt){assert.equal(BigInt(t.value),expected[t.i]);const P=167772161n*469762049n*1224736769n;assert.equal(mod(BigInt(t.value),P),BigInt(t.residue));}
  lastFrame=f;
 }
 const first=JSON.stringify(batch.frames[0]);if(batch.frames.length>1){last.arrays.A[0]='modified';assert.equal(JSON.stringify(batch.frames[0]),first);}
 return batch;
}
let seed=316016;const random=n=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed%n;};
for(const mode of M.MODES){for(const scenario of ['basic','empty','one','impulse','negative','cancellation'])verify(M.scenario(mode,scenario));
 for(let trial=0;trial<160;++trial){const c=M.defaults(mode),single=['ntt','fft-real','fft-complex'].includes(mode),n=single?[0,1,2,4,8,16][random(6)]:random(9),m=random(9),complex=mode==='fft-complex'||mode==='convolution-fft'&&trial%2;
  const values=n=>Array.from({length:n},()=>{const x=random(41)-20;if(complex)return `${x/4},${(random(41)-20)/4}`;return ['ntt','convolution','integer'].includes(mode)?String(x):String(x/4);}).join(' ');c.a=values(n);c.b=values(m);c.modulus=['17','97','998244353'][random(3)];c.direction=trial%2?'inverse':'forward';c.kind=complex?'complex':'real';verify(c);
 }
}
for(const mode of ['convolution','integer','convolution-fft'])verify(M.scenario(mode,'padding'));
for(const mode of ['ntt','convolution'])verify(M.scenario(mode,'mod-large'));
verify(M.scenario('convolution-fft','complex'));verify(M.scenario('integer','integer-limit'));
for(const x of ['9223372036854775807','-9223372036854775808','-1','0','1'])for(const y of ['-1','0','1']){
 const c={...M.defaults('integer'),a:x,b:y};if(x==='-9223372036854775808'&&y==='-1'){assert.throws(()=>M.run(c),/long long/);++rejections;}else verify(c);
}
for(const raw of [M.scenario('integer','overflow'),{mode:'ntt',a:'1 2 3'},{mode:'fft-real',a:'1 2 3'},{mode:'fft-complex',a:'1,NaN'},{mode:'fft-real',a:'Infinity'},{mode:'integer',a:'9223372036854775808'},{mode:'integer',a:'0.5'},{mode:'ntt',a:Array(17).fill('1').join(' ')},{mode:'convolution',b:Array(9).fill('1').join(' ')},{mode:'fft-real',a:'1000001'},{mode:'convolution-fft',kind:'real',a:'1,2'},{mode:'ntt',modulus:'12'},{mode:'ntt',focus:-1},{mode:'ntt',direction:'bad'},{mode:'ntt',kind:'bad'},{mode:'unknown'}]){assert.throws(()=>M.run(raw));++rejections;}
console.log(`convolution models: ${batches} batches / ${snapshots} snapshots / ${rejections} rejected inputs; direct field/complex DFT, every completed block, butterflies, zero padding, signed CRT, exact products, inverse normalization and snapshot independence passed`);
