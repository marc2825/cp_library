'use strict';
const assert=require('node:assert/strict');
const model=require('../../visualizations/number_theory_extensions/model.js');
const expected=require('../number_theory_extensions/visualization_expected.json');
let checks=0;
function check(id,input,want){const trace=model.trace(id,input);assert.deepEqual(trace.result,want.map(String),`${id} ${JSON.stringify(input)}`);assert.ok(trace.steps.length>=2);for(const s of trace.steps){assert.ok(s.title&&s.detail);assert.ok(s.columns.length===0||s.rows.every(r=>r.length===s.columns.length));}checks++;}
for(const m of model.modes)check(m.id,m.sample,expected[m.id]);
const gcd=(a,b)=>b?gcd(b,a%b):a;
const prime=x=>x>=2&&Array.from({length:x-2},(_,i)=>i+2).every(d=>x%d);
const pow=(a,k,m)=>{let r=1%m;while(k--)r=r*a%m;return r;};
const fact=n=>{let r=1n;for(let i=1;i<=n;i++)r*=BigInt(i);return r;};
for(let n=0;n<=35;n++){
 for(const p of [2,3,5,17,101])check('large_factorial',{n,prime:p},[fact(n)%BigInt(p)]);
 for(let k=0;k<=Math.min(7,n+1);k++)for(const modulus of [1,8,12,25])check('prime_power_binomial',{n,k,modulus},[k>n?0:fact(n)/fact(k)/fact(n-k)%BigInt(modulus)]);
 let primes=0;for(let i=2;i<=n;i++)if(prime(i))primes+=i;check('prime_sum',{n},[primes]);
 let divisorCount=0,divisorSum=0,totient=0,squares=0,custom=0;
 for(let i=1;i<=n;i++){squares+=i*i;for(let j=1;j<=i;j++){if(i%j===0){divisorCount++;divisorSum+=j;}if(gcd(i,j)===1)totient++;}let v=i,weight=1;for(let p=2;p<=v;p++)while(v%p===0){v/=p;weight*=p%4===1?2:-1;}custom+=weight;}
 for(const [kind,value]of [['phi',totient],['divisors',divisorCount],['sigma',divisorSum],['squares',squares],['residue',custom]]){check('min25_sieve',{n,kind},[value]);check('multiplicative_summatory',{n,kind},[value]);}
 check('dirichlet_prefix_product',{n,a:0,b:0},[divisorCount]);
 for(const [a,b]of [[1,0],[1,1],[2,1]]){let value=0;for(let i=1;i<=n;i++)for(let j=1;i*j<=n;j++)value+=i**a*j**b;check('dirichlet_prefix_product',{n,a,b},[value]);}
 const pairs=[];for(let a=0;a*a<=n;a++)for(let b=a;a*a+b*b<=n;b++)if(a*a+b*b===n)pairs.push(a,b);check('sum_two_squares',{n},pairs);
}
for(let m=1;m<=25;m++)for(let a=0;a<m;a++){
 let lambda=1;for(let x=1;x<=m;x++)if(gcd(x,m)===1){let k=1;while(pow(x,k,m)!==1%m)k++;lambda=lambda/gcd(lambda,k)*k;}
 let order=-1;if(gcd(a,m)===1){order=1;while(pow(a,order,m)!==1%m)order++;}check('multiplicative_order',{modulus:m,base:a,order_modulus:m},[lambda,order]);
 const targets=Array.from({length:Math.min(m,20)},(_,i)=>i),logs=Array(m).fill(-1);let v=1%m;for(let i=0;logs[v]===-1;i++){logs[v]=i;v=v*a%m;}check('discrete_log_table',{base:a,modulus:m,targets},targets.map(b=>logs[b]));
 const values=[a,1%m];if(gcd(a,m)!==1)check('batch_inversion',{values,modulus:m},['none']);else check('batch_inversion',{values,modulus:m},values.map(x=>Array.from({length:m},(_,i)=>i).find(y=>x*y%m===1%m)));
 if(prime(m))for(let k=0;k<=4;k++)check('mod_kth_root',{a,k,prime:m},Array.from({length:m},(_,i)=>i).filter(x=>pow(x,k,m)===a));
}
for(let left=0;left<25;left++){
 const right=left+15,x=left+8,primes=[];for(let i=left;i<right;i++)if(prime(i))primes.push(i);let value=x,factors=[];for(let p=2;p<=value;p++)if(value%p===0){let e=0;while(value%p===0){value/=p;e++;}factors.push(p,e);}check('segmented_sieve',{left,right,x},[...primes,...factors]);
}
for(let a=-4;a<=4;a++)for(let b=-4;b<=4;b++)for(let m=1;m<=8;m++){
 const n=8;let sum=0,weighted=0,squares=0,best=m,index=0,total=0;
 for(let i=0;i<n;i++){const f=Math.floor((a*i+b)/m),r=((a*i+b)%m+m)%m;sum+=f;weighted+=i*f;squares+=f*f;if(r<best){best=r;index=i;}total+=i*r*r;}
 check('floor_sum_polynomial',{n,m,a,b},[sum,weighted,squares]);check('min_mod_linear',{left:0,right:n,a,b,modulus:m},[best,index]);check('mod_sum_polynomial',{left:0,right:n,a,b,modulus:m,p:1,q:2},[total]);
}
for(let n=0;n<=35;n++)for(const first of [-1,1]){
 const tail=2,g=Array(n+1).fill(0);for(let i=1;i<=n;i++){g[i]=i===1?1:0;for(let d=2;d<=i;d++)if(i%d===0)g[i]-=tail*g[i/d];g[i]*=first;}check('dirichlet_inverse',{n,first,tail},[...g.slice(1,11),g.reduce((a,b)=>a+b,0)]);
}
for(const [D,sign,fundamental]of [[2,1,[3,2]],[2,-1,[1,1]],[3,1,[2,1]],[3,-1,null],[5,1,[9,4]],[13,1,[649,180]]])for(let exponent=0;exponent<4;exponent++){
 if(!fundamental){check('pell_equation',{D,sign,exponent},['none']);continue;}let x=1n,y=0n;for(let i=0;i<exponent;i++)[x,y]=[x*BigInt(fundamental[0])+BigInt(D)*y*BigInt(fundamental[1]),x*BigInt(fundamental[1])+y*BigInt(fundamental[0])];check('pell_equation',{D,sign,exponent},[...fundamental,x,y]);
}
check('continued_fraction',{numerator:-1,denominator:2,bound:1},[-1,2,'-1/1']);
check('continued_fraction',{numerator:0,denominator:3,bound:1},[0,'0/1']);
check('gaussian_integer',{a:[0,0],b:[0,2]},[0,0,0,0,2,0]);
for(const [id,input]of [['mod_kth_root',{a:1,k:2,prime:9}],['large_factorial',{n:3,prime:10}],['pell_equation',{D:4,sign:1,exponent:1}],['min_mod_linear',{left:1,right:0,a:1,b:1,modulus:2}],['prime_sum',{n:10001}],['gaussian_integer',{a:[1,1],b:[0,0]}],['batch_inversion',{values:[],modulus:0}]])assert.throws(()=>model.trace(id,input));
console.log(`number theory extensions model: ${checks} independent comparisons and invalid-input checks passed`);
