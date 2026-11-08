'use strict';
const assert=require('node:assert/strict'),model=require('../../visualizations/combinatorics_extensions/model.js'),expected=require('../combinatorics_extensions/visualization_expected.json');
let checks=0,seed=51631;const rnd=n=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed%n;};
const run=(id,input)=>model.trace(id,input).result;
const equal=(a,b)=>{assert.deepEqual(a,b.map(String));checks++;};
for(const m of model.modes)equal(run(m.id,m.sample),expected[m.id]);
for(const p of[2,3,5,7,11])for(let q=0;q<p;q++)for(let n=0;n<=8;n++){
 const values=Array(n+1).fill(0),powers=[1];for(let i=1;i<=n*n;i++)powers[i]=powers[i-1]*q%p;
 for(let mask=0;mask<(1<<n);mask++){let k=0,sum=0;for(let i=0;i<n;i++)if(mask>>i&1){k++;sum+=i;}values[k]=(values[k]+powers[sum-k*(k-1)/2])%p;}
 for(let k=0;k<=n;k++){assert.equal(run('q_binomial',{prime:p,q,n,k})[0],String(values[k]));checks++;}
}
for(let t=0;t<500;t++){
 const n=rnd(6),lower=Array.from({length:n},()=>rnd(4)-1),upper=Array.from({length:n},()=>rnd(4)-1),strict=!!rnd(2);let count=0;
 function visit(i,prev){if(i===n){count++;return;}for(let x=lower[i];x<=upper[i];x++)if(!i||x>prev||(!strict&&x===prev))visit(i+1,x);}visit(0,0);
 equal(run('bounded_monotone',{lower,upper,strict}),[count]);
}
for(const p of[2,3,5,7,11,13]){
 let c=[1],s=[1];for(let n=0;n<=80;n++){
  for(let sample=0;sample<4;sample++){let k=rnd(n+3);equal(run('stirling_small_prime',{prime:p,n,k}),[c[k]||0,s[k]||0]);}
  const nextC=Array(n+2).fill(0),nextS=[...nextC];for(let k=1;k<=n+1;k++){nextC[k]=((c[k-1]||0)+n*(c[k]||0))%p;nextS[k]=((s[k-1]||0)+k*(s[k]||0))%p;}c=nextC;s=nextS;
 }
}
equal(run('unlabeled_trees',{n:10}),[1,1,2,4,9,20,48,115,286,719,1,1,1,2,3,6,11,23,47,106]);
for(let t=0;t<500;t++){
 const word=Array.from({length:rnd(10)},()=>rnd(5)),dp=word.map(()=>1);let longest=0;
 word.forEach((x,i)=>{for(let j=0;j<i;j++)if(word[j]<=x)dp[i]=Math.max(dp[i],dp[j]+1);longest=Math.max(longest,dp[i]);});
 const trace=model.trace('rsk',{word}),r=trace.result.map(Number),shape=r.slice(0,r.length-word.length);assert.deepEqual(r.slice(shape.length),word);assert.equal(shape[0]||0,longest);
 for(const step of trace.steps.filter(s=>s.title.includes('行末へ追加'))){const [P,Q]=step.state.grids.map(g=>g.cells);assert.deepEqual(P.map(r=>r.length),Q.map(r=>r.length));for(let i=0;i<P.length;i++)for(let j=0;j<P[i].length;j++){if(j)assert.ok(Number(P[i][j-1])<=Number(P[i][j]));if(i)assert.ok(Number(P[i-1][j])<Number(P[i][j]));}}
 checks++;
}
for(const shape of[[],[1],[3],[1,1,1],[3,2],[2,2,1],[4,2,1]]){
 const n=shape.reduce((a,b)=>a+b,0),cells=[];shape.forEach((len,r)=>{for(let c=0;c<len;c++)cells.push([r,c]);});let count=0;
 function visit(a,used){if(a.length===n){for(let i=0;i<n;i++){const[r,c]=cells[i];for(let j=0;j<i;j++){const[x,y]=cells[j];if(((x===r&&y<c)||(y===c&&x<r))&&a[j]>a[i])return;}}count++;return;}
  for(let k=1;k<=n;k++)if(!(used>>k&1)){a.push(k);visit(a,used|(1<<k));a.pop();}}
 visit([],0);assert.equal(run('young_tableaux',{shape})[0],String(count));assert.equal(run('hook_length',{shape,alphabet:3,modulus:12})[0],String(count));checks++;
}
const factorial=n=>{let v=1n;for(let k=2;k<=n;k++)v*=BigInt(k);return v;};const choose=(n,k)=>k<0||k>n?0n:factorial(n)/factorial(k)/factorial(n-k);
for(let it=0;it<150;it++){
 const n=1+rnd(15),values=Array.from({length:n},(_,i)=>i?rnd(4):0),e=Array(n).fill(0n),lah=[0n];e[0]=1n;
 for(let k=1;k<n;k++)for(let copy=0;copy<values[k];copy++)for(let j=k;j<n;j++)e[j]=(e[j]+e[j-k])%998244353n;
 for(let i=1;i<n;i++){let sum=0n;for(let k=1;k<=i;k++)sum+=BigInt(values[k])*factorial(i)/factorial(k)*choose(i-1,k-1);lah[i]=sum%998244353n;}
 equal(run('combinatorial_transforms',{values}),[...lah,...e,...values]);
}
for(const[id,input]of[['q_binomial',{prime:4,q:1,n:2,k:1}],['bounded_monotone',{lower:[0],upper:[],strict:false}],['stirling_small_prime',{prime:5,n:'18446744073709551616',k:0}],['young_tableaux',{shape:[2,3]}],['hook_length',{shape:[2],alphabet:3,modulus:0}],['combinatorial_transforms',{values:[1]}],['rsk',{word:[NaN]}]]){assert.throws(()=>run(id,input));checks++;}
console.log(`combinatorics extensions model: ${checks} independent comparisons and rejection checks passed`);
