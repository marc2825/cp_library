'use strict';
const assert=require('node:assert/strict');const {specs,build}=require('../../visualizations/b_12_15/model.js');let seed=312152026,cases=0,states=0;const random=n=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed%n;};
function run(mode,input){const before=JSON.stringify(input),batch=build(mode,input);assert.equal(JSON.stringify(input),before);assert.ok(batch.frames.length>=2);assert.equal(batch.frames.at(-1).title,'完了');assert.deepEqual(JSON.parse(batch.frames.at(-1).rows[0][0]),batch.result);for(const f of batch.frames){assert.ok(f.rows.length&&f.columns.length);assert.ok(Array.isArray(f.grid));}cases++;states+=batch.frames.length;return batch.result;}
for(const[mode,[,input]]of Object.entries(specs)){const first=run(mode,input);assert.deepEqual(run(mode,input),first);}
assert.deepEqual(run('factorial_basis',{a:[0,0,0,1],mod:101}),[0,1,3,1]);
const modulo=(x,m)=>(x%m+m)%m;
function choose(n,k){if(k<0||k>n)return 0n;k=Math.min(k,n-k);let x=1n;for(let i=1;i<=k;i++)x=x*BigInt(n-i+1)/BigInt(i);return x;}
function evalPoly(a,x,m){let y=0;for(let k=a.length-1;k>=0;k--)y=modulo(y*x+a[k],m);return y;}
function gcd(a,b){while(b)[a,b]=[b,a%b];return a;}
for(let trial=0;trial<90;trial++){
 const n=1+random(7),m=1+random(8),a=Array.from({length:n},()=>random(31)-15),b=[];let value=0,slope=20;for(let i=0;i<m;i++){slope-=random(5);value+=slope;b.push(value);}const expected=Array.from({length:n+m-1},(_,k)=>Math.min(...a.flatMap((x,i)=>k-i>=0&&k-i<m?[x+b[k-i]]:[])));assert.deepEqual(run('min_plus_convolution',{a,b,kind:'arbitrary_concave'}),expected);
 const x=[0],bias=[0];for(let i=1;i<=n;i++){x.push(x.at(-1)+random(4));bias.push(random(15)-7);}const cost=(i,j)=>(x[j]-x[i])**2+bias[j],byCount=Array(n+1).fill(null),dp=[0];let best=null;
 for(let end=1;end<=n;end++){let minimum=null;for(let mask=0;mask<2**(end-1);mask++){const path=[0];for(let v=1;v<end;v++)if(mask>>(v-1)&1)path.push(v);path.push(end);let total=0;for(let i=1;i<path.length;i++)total+=cost(path[i-1],path[i]);minimum=minimum===null?total:Math.min(minimum,total);if(end===n){const k=path.length-1;byCount[k]=byCount[k]===null?total:Math.min(byCount[k],total);}}dp.push(minimum);}const r=run('monge_shortest_path',{x,vertex_cost:bias});assert.deepEqual(r.distance,dp);const k=random(n+2),exact=run('monge_shortest_path_k_edges',{x,vertex_cost:bias,k});assert.equal(exact?.cost??null,byCount[k]??null);const argmin=run('larsch',{x,vertex_cost:bias});for(let i=0;i<n;i++)assert.equal(dp[argmin[i]]+cost(argmin[i],i+1),dp[i+1]);
 const items=Array.from({length:random(13)},()=>[random(10),random(21)-5]),capacity=random(25);let opt=0;for(let mask=0;mask<2**items.length;mask++){let w=0,v=0;items.forEach(([a,b],i)=>{if(mask>>i&1){w+=a;v+=b;}});if(w<=capacity)opt=Math.max(opt,v);}assert.equal(run('branch_bound_knapsack',{items,capacity}).value,opt);
 const initial=Array.from({length:n+5},()=>random(15)-7),kernel=Array.from({length:n+2},()=>random(11)-5),convolution=[];for(let i=0;i<initial.length;i++){let v=initial[i];for(let j=0;j<i;j++)v+=convolution[j]*(kernel[i-j]??0);convolution.push(modulo(v,101));}assert.deepEqual(run('online_offline_dp',{initial,kernel,mod:101}),convolution);
 const rows=random(8),inner=random(8),cols=inner?random(8):0,A=Array.from({length:rows},()=>Array.from({length:inner},()=>random(2))),B=Array.from({length:inner},()=>Array.from({length:cols},()=>random(2))),C=A.map(row=>Array.from({length:cols},(_,j)=>Number(row.some((a,k)=>a&&B[k][j]))));assert.deepEqual(run('four_russians',{a:A,b:B,block:1+random(4)}),C);
}
for(let trial=0;trial<160;trial++){
 const m=1+random(160),a=random(m*2)-m,b=random(m*2)-m,seen=new Set();let v=1%m,e=0,log=null;while(!seen.has(v)){if(v===modulo(b,m)){log=e;break;}seen.add(v);v=modulo(v*a,m);e++;}assert.equal(run('discrete_log',{a,b,mod:m}),log);
 const solutions=[];for(let x=0;x<m;x++)if(modulo(a*x-b,m)===0)solutions.push(x);const r=run('linear_congruence',{a,b,mod:m});assert.deepEqual(r?Array.from({length:r.count},(_,t)=>r.first+t*r.step):[],solutions);
 const p=[2,3,5,7,11,13,17,101][random(8)],y=random(2*p)-p;let sqrt=null;for(let x=0;x<p;x++)if(x*x%p===modulo(y,p)){sqrt=x;break;}assert.equal(run('mod_sqrt',{a:y,prime:p}),sqrt);
 const n=random(260),k=random(n+3);assert.equal(run('lucas',{n:String(n),k:String(k),prime:p}),Number(choose(n,k)%BigInt(p)));
}
for(let n=0;n<=90;n++){
 let primes=0,totients=0;const mu=[0,1];for(let i=1;i<=n;i++){if(i>=2){let divisor=false;for(let d=2;d*d<=i;d++)if(i%d===0)divisor=true;if(!divisor)primes++;let value=0;for(let d=1;d<i;d++)if(i%d===0)value-=mu[d];mu[i]=value;}for(let x=1;x<=i;x++)totients+=Number(gcd(i,x)===1);}
 assert.equal(run('prime_count',{n}),primes);assert.equal(run('prime_counting',{n}),primes);assert.equal(run('summatory_totient',{n,precompute:1+random(9)}),totients);assert.equal(run('mertens',{n,precompute:1+random(9)}),mu.slice(1,n+1).reduce((a,b)=>a+b,0));
 let divisorCount=0,squareFree=0;const tau=[0],prefix=[0];for(let i=1;i<=n;i++){let count=0,good=true;for(let d=1;d<=i;d++)if(i%d===0)count++;for(let d=2;d*d<=i;d++)if(i%(d*d)===0)good=false;divisorCount+=count;squareFree+=good;tau.push(count);prefix.push(divisorCount);}assert.equal(run('divisor_count_sum',{n}),divisorCount);assert.equal(run('square_free_counting',{n}),squareFree);assert.deepEqual(run('multiplicative_function_sum',{n,kind:'tau'}),{values:tau,prefix});const blocks=run('quotient_enumeration',{n});assert.deepEqual(blocks.flatMap(([l,r,q])=>Array.from({length:r-l+1},(_,j)=>[l+j,q])),Array.from({length:n},(_,j)=>[j+1,Math.floor(n/(j+1))]));
}
for(let trial=0;trial<80;trial++){
 const n=BigInt(random(1000000)),k=1+random(13),r=BigInt(run('integer_kth_root',{n:n.toString(),k}));assert.ok(r**BigInt(k)<=n&&(r+1n)**BigInt(k)>n);
 const a=BigInt(random(5)),h=random(5),m=1+random(10000);let exponent=1n;if(h>0)for(let j=0;j<h-1;j++)exponent=a**exponent;const pow=(a,e,m)=>{let r=1n%m;for(;e;e>>=1n,a=a*a%m)if(e&1n)r=r*a%m;return r;};const value=h===0?1n%BigInt(m):pow(a,exponent,BigInt(m));assert.equal(run('mod_tetration',{a:a.toString(),height:String(h),mod:m}),value.toString());
}
assert.equal(run('integer_kth_root',{n:'18446744073709551615',k:2}),'4294967295');assert.equal(run('mod_tetration',{a:'0',height:'18446744073709551615',mod:100}),'0');
for(let trial=0;trial<100;trial++){
 const m=trial%3===0?12:trial%3===1?1:101,n=random(10),a=Array.from({length:n},()=>random(21)-10),normalized=a.map(x=>modulo(x,m));const transformed=run('binomial_transform',{a,mod:m});for(let i=0;i<n;i++){let sum=0;for(let mask=0;mask<2**i;mask++){let count=0;for(let b=mask;b;b&=b-1)count++;sum+=a[count];}assert.equal(transformed[i],modulo(sum,m));}assert.deepEqual(run('binomial_transform',{a:transformed,mod:m,inverse:true}),normalized);const s=run('stirling_transform',{a,mod:m});assert.deepEqual(run('stirling_transform',{a:s,mod:m,inverse:true}),normalized);
 for(const rising of [false,true]){const b=run('factorial_basis',{a,mod:m,rising,direction:'to_factorial'});assert.deepEqual(run('factorial_basis',{a:b,mod:m,rising,direction:'to_power'}),normalized);for(let x=0;x<8;x++){let value=0,product=1%m;for(let k=0;k<n;k++){value=modulo(value+b[k]*product,m);product=modulo(product*(rising?x+k:x-k),m);}assert.equal(value,evalPoly(a,x,m));}}
 const samples=Array.from({length:n},(_,x)=>evalPoly(a,x,101)),first=random(160)-30,count=random(15),shift=run('shift_sample_points',{samples,first,count,prime:101});assert.deepEqual(shift,Array.from({length:count},(_,j)=>evalPoly(a,modulo(first+j,101),101)));
}
for(let n=0;n<=8;n++){
 const a=Array.from({length:n+1},(_,k)=>k*k+3),hist=Array(n+1).fill(0);const partitions=(at,blocks)=>{if(at===n){hist[blocks]++;return;}for(let color=0;color<=blocks;color++)partitions(at+1,Math.max(blocks,color+1));};partitions(0,0);assert.equal(run('stirling_transform',{a,mod:101})[n],hist.reduce((s,c,k)=>s+c*a[k],0)%101);
 const narayana=Array(n+1).fill(0);for(let mask=0;mask<2**(2*n);mask++){let height=0,peaks=0,good=true;for(let j=0;j<2*n;j++){const up=Boolean(mask>>j&1);height+=up?1:-1;if(height<0)good=false;if(j&&!up&&(mask>>(j-1)&1))peaks++;}if(good&&height===0)narayana[peaks]++;}for(let k=0;k<=n;k++)assert.equal(run('narayana',{n,k,mod:101}),narayana[k]%101);
 const prefix=Array.from({length:n+1},(_,k)=>Number(choose(n,k)%101n));for(let k=-1;k<=n+1;k++)assert.equal(run('binomial_prefix_sum',{n,k,mod:101}),prefix.slice(0,Math.max(0,k+1)).reduce((a,b)=>a+b,0)%101);
}
const euler=run('eulerian_numbers',{n:7,mod:101});for(let n=0;n<=7;n++){const count=Array(n+1).fill(0),permutation=[];const visit=used=>{if(permutation.length===n){let ascents=0;for(let j=1;j<n;j++)ascents+=permutation[j]>permutation[j-1];count[ascents]++;return;}for(let j=0;j<n;j++)if(!(used>>j&1)){permutation.push(j);visit(used|(1<<j));permutation.pop();}};visit(0);assert.deepEqual(euler[n],count.map(x=>x%101));}
const bernoulli=run('bernoulli_numbers',{n:12,mod:101});for(let k=0;k<10;k++)for(let bound=1;bound<8;bound++){let left=0,right=0;for(let x=0;x<bound;x++)left=(left+x**k)%101;for(let j=0;j<=k;j++)right=(right+Number(choose(k+1,j)%101n)*bernoulli[j]*(bound**(k+1-j)%101))%101;assert.equal((left*(k+1))%101,right);}
for(let trial=0;trial<60;trial++){
 const n=random(10),phi=Array.from({length:1+random(5)},()=>random(6)),f=Array.from({length:1+random(8)},()=>random(6));
 const multiply=(a,b)=>{const out=Array(n+1).fill(0);for(let i=0;i<a.length;i++)for(let j=0;j<b.length&&i+j<=n;j++)out[i+j]=(out[i+j]+a[i]*b[j])%101;return out;};
 const compose=(a,g)=>{let power=Array(n+1).fill(0),out=Array(n+1).fill(0);power[0]=1;for(const c of a){for(let i=0;i<=n;i++)out[i]=(out[i]+c*power[i])%101;power=multiply(power,g);}return out;};
 let g=Array(n+1).fill(0);for(let iteration=0;iteration<=n;iteration++){const h=compose(phi,g);g=[0,...h.slice(0,n)];}
 assert.equal(run('lagrange_inversion',{phi,function:f,n,mod:101}),compose(f,g)[n]);
}
const inv=build('quotient_enumeration',{n:36});assert.equal(inv.frames[1].rows.length,1);assert.ok(inv.frames.at(-2).rows.length>1);
const rejects=[['mod_sqrt',{a:1,prime:15}],['linear_congruence',{a:1,b:1,mod:0}],['lucas',{n:'5',k:'2',prime:4}],['integer_kth_root',{n:'10',k:0}],['integer_kth_root',{n:'18446744073709551616',k:2}],['four_russians',{a:[[1]],b:[[1],[0]],block:2}],['min_plus_convolution',{a:[0],b:[0,1,5],kind:'arbitrary_concave'}],['monge_shortest_path',{x:[0,2,1]}],['branch_bound_knapsack',{capacity:3,items:[[-1,5]]}],['bernoulli_numbers',{n:4,mod:5}],['lagrange_inversion',{phi:[1],function:[0,1],n:5,mod:5}],['prime_count',{n:100001}],['narayana',{n:3,k:1,mod:4}],['integer_kth_root',{n:18446744073709551615,k:2}],['missing',{}]];for(const[mode,input]of rejects)assert.throws(()=>build(mode,input));
console.log(`${cases} model inputs, ${states} snapshots, ${rejects.length} rejection conditions passed`);
