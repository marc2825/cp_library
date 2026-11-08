'use strict';
const assert=require('node:assert/strict');
const {specs,build}=require('../../visualizations/math_a/model.js');
let cases=0,snapshots=0,state=917281;
const random=n=>{state=(Math.imul(state,1664525)+1013904223)>>>0;return state%n;};
function run(mode,input,expected){const batch=build(mode,input);cases++;snapshots+=batch.frames.length;assert.deepEqual(batch.result,expected,mode+' '+JSON.stringify(input));assert.equal(batch.frames.at(-1).title,'完了');for(const f of batch.frames){assert.ok(f.title&&Array.isArray(f.rows)&&f.rows.length&&f.columns.length);if(f.series)for(const s of f.series)for(const point of s.points)assert.ok(point.every(Number.isFinite));}return batch;}
for(const [mode,[,sample]]of Object.entries(specs)){const r=build(mode,sample);run(mode,sample,r.result);}
for(let trial=0;trial<180;trial++){
 const n=random(9),values=Array.from({length:n},()=>random(6)),groups=random(11),cost=(l,r)=>values.slice(l,r).reduce((s,v)=>s+v,0)**2;
 const dp=Array.from({length:groups+1},()=>Array(n+1).fill(null)),parent=Array.from({length:groups+1},()=>Array(n+1).fill(-1));dp[0][0]=0;
 for(let k=1;k<=groups;k++)for(let r=1;r<=n;r++)for(let l=0;l<r;l++)if(dp[k-1][l]!==null){const v=dp[k-1][l]+cost(l,r);if(dp[k][r]===null||v<dp[k][r]){dp[k][r]=v;parent[k][r]=l;}}
 let partitions=null;if(dp[groups][n]!==null){partitions=[];let end=n;for(let k=groups;k>0;k--){partitions.push([parent[k][end],end]);end=parent[k][end];}partitions.reverse();}run('divide_conquer_dp',{values,groups},{dp,partitions});
 const interval=Array.from({length:n+1},()=>Array(n+1).fill(0)),split=Array.from({length:n+1},()=>Array(n+1).fill(-1));
 for(let len=2;len<=n;len++)for(let l=0;l+len<=n;l++){const r=l+len;for(let k=l+1;k<r;k++){const v=interval[l][k]+interval[k][r]+values.slice(l,r).reduce((s,v)=>s+v,0);if(split[l][r]<0||v<interval[l][r]){interval[l][r]=v;split[l][r]=k;}}}
 const merges=[];const visit=(l,r)=>{const k=split[l][r];if(k<0)return;visit(l,k);visit(k,r);merges.push([l,k,r]);};visit(0,n);run('knuth_optimization',{values},{value:interval[0][n],split,merges});
 const heights=Array.from({length:n},()=>random(31)-15),constant=random(31)-10,quadratic=[];for(let i=0;i<n;i++)quadratic.push(i?Math.min(...quadratic.map((d,j)=>d+(heights[i]-heights[j])**2+constant)):0);
 run('convex_hull_trick_dp',{heights,constant},quadratic);run('li_chao_dp',{heights,constant},quadratic);
 if(n){const k=1+random(n),batch=build('aliens_trick',{values,groups:k,low:-10000,high:10000});cases++;snapshots+=batch.frames.length;assert.equal(batch.result.value,dp.length>k?dp[k][n]:(()=>{const a=Array.from({length:k+1},()=>Array(n+1).fill(Infinity));a[0][0]=0;for(let c=1;c<=k;c++)for(let r=1;r<=n;r++)for(let l=0;l<r;l++)a[c][r]=Math.min(a[c][r],a[c-1][l]+cost(l,r));return a[k][n];})());}
}
for(let trial=0;trial<110;trial++){
 const bound=120;let grid=Array(2*bound+1).fill(0);const operations=[];
 for(let op=0;op<20;op++){const type=random(7),a=random(15)-7;if(type<3){const kind=['abs','left','right'][type];operations.push([kind,a]);grid=grid.map((v,i)=>v+(type===0?Math.abs(i-bound-a):type===1?Math.max(a-i+bound,0):Math.max(i-bound-a,0)));}else if(type===3){operations.push(['prefix']);for(let i=1;i<grid.length;i++)grid[i]=Math.min(grid[i],grid[i-1]);}else if(type===4){operations.push(['suffix']);for(let i=grid.length-2;i>=0;i--)grid[i]=Math.min(grid[i],grid[i+1]);}else if(type===5){const l=random(3)-1,r=l+random(3);operations.push(['shift',l,r]);const next=grid.slice();for(let x=-bound;x<=bound;x++)next[x+bound]=Math.min(...Array.from({length:r-l+1},(_,j)=>grid[x-r+j+bound]??Infinity));grid=next;}else{operations.push(['constant',a]);grid=grid.map(x=>x+a);}}
 const batch=build('slope_trick',{operations});cases++;snapshots+=batch.frames.length;assert.deepEqual(batch.result.values,grid.slice(bound-10,bound+11));assert.equal(batch.result.minimum,Math.min(...grid.slice(50,-50)));
}
const gcd=(a,b)=>b?gcd(b,a%b):a;
for(let n=0;n<=60;n++){
 const phi=[0],mu=[0];for(let i=1;i<=n;i++){phi[i]=Array.from({length:i},(_,j)=>gcd(i,j+1)===1?1:0).reduce((a,b)=>a+b,0);let x=i,count=0,square=false;for(let p=2;p<=x;p++)if(x%p===0){let e=0;while(x%p===0){x/=p;e++;}count++;square ||= e>1;}mu[i]=square?0:count%2?-1:1;}
 run('euler_phi',{limit:n},phi);run('mobius',{limit:n},mu);
}
for(let trial=0;trial<300;trial++){
 const m=1+random(20),n=1+random(20),a=random(41)-20,b=random(41)-20;let x=null;for(let v=0;v<m*n/gcd(m,n);v++)if((v-a)%m===0&&(v-b)%n===0){x=v;break;}
 run('crt',{residues:[a,b],moduli:[m,n]},x===null?null:{remainder:String(x),modulus:String(m*n/gcd(m,n))});
 if(gcd(m,n)===1){const target=1+random(20);run('garner',{residues:[a,b],moduli:[m,n],target},String(x%target));}
 const count=random(22);let sum=0;for(let i=0;i<count;i++)sum+=Math.floor((a*i+b)/m);run('floor_sum',{n:count,m,a,b},String(sum));
}
run('garner',{residues:[],moduli:[],target:7},'0');run('crt',{residues:[],moduli:[]},{remainder:'0',modulus:'1'});
run('floor_sum',{n:'4294967295',m:'4294967295',a:'4294967294',b:'4294967294'},'9223372030412324865');
run('floor_sum',{n:1,m:1,a:'-9223372036854775808',b:'-9223372036854775808'},'-9223372036854775808');
run('crt',{residues:['-9223372036854775808'],moduli:['9223372036854775807']},{remainder:'9223372036854775806',modulus:'9223372036854775807'});
for(let p=2;p<100;p++)if(Array.from({length:p-2},(_,i)=>i+2).every(d=>p%d!==0)){let root=1;for(;;root++){let x=1;const seen=new Set();for(let i=0;i<p-1;i++){seen.add(x);x=x*root%p;}if(seen.size===p-1)break;}run('primitive_root',{prime:p},root);}
for(const modulus of[1,2,8,97,1000000000])for(let n=0;n<=8;n++){
 const first=Array.from({length:n+1},(_,i)=>Array(i+1).fill(0)),second=first.map(r=>r.slice()),derange=Array(n+1).fill(0);
 const perm=(a,left)=>{if(!left.length){const used=Array(a.length).fill(false);let count=0,deranged=true;for(let i=0;i<a.length;i++){deranged &&= a[i]!==i;if(!used[i]){count++;let x=i;do{used[x]=true;x=a[x];}while(x!==i);}}first[a.length][count]++;derange[a.length]+=Number(deranged);return;}for(const x of left)perm([...a,x],left.filter(v=>v!==x));};
 const sets=(pos,length,blocks)=>{if(pos===length){second[length][blocks]++;return;}for(let b=0;b<=blocks;b++)sets(pos+1,length,Math.max(blocks,b+1));};
 for(let size=0;size<=n;size++){perm([],Array.from({length:size},(_,i)=>i));sets(0,size,0);}
 const modular=a=>a.map(r=>r.map(x=>x%modulus));run('stirling',{n,kind:'first',modulus},modular(first));run('stirling',{n,kind:'second',modulus},modular(second));run('stirling',{n,kind:'signed',modulus},first.map((r,i)=>r.map((x,k)=>((i-k)%2&&x%modulus?modulus-x%modulus:x%modulus))));
 run('bell',{n,modulus},second.map(r=>r.reduce((a,b)=>a+b,0)%modulus));run('derangements',{n,modulus},derange.map(x=>x%modulus));
 const parts=(sum,minimum)=>{if(!sum)return 1;let value=0;for(let p=minimum;p<=sum;p++)value+=parts(sum-p,p);return value;};run('partition_numbers',{n,modulus},Array.from({length:n+1},(_,i)=>parts(i,1)%modulus));
}
for(let n=0;n<=7;n++)for(const dihedral of[false,true]){
 const weights=[1,2],group=[];if(!n)group.push([]);else for(let flip=0;flip<=Number(dihedral);flip++)for(let shift=0;shift<n;shift++)group.push(Array.from({length:n},(_,i)=>(shift+(flip?n-i:i))%n));
 const representatives=new Set(),fixed=group.map(()=>0);let total=0;
 for(let mask=0;mask<2**n;mask++){const a=Array.from({length:n},(_,i)=>mask>>i&1);let best=a.join('');for(let id=0;id<group.length;id++){const b=Array(n);for(let i=0;i<n;i++)b[group[id][i]]=a[i];const key=b.join('');if(key===a.join(''))fixed[id]++;if(key<best)best=key;}if(!representatives.has(best)){representatives.add(best);total+=a.reduce((s,c)=>s*weights[c],1);}}
 const result=build('polya',{n,dihedral,weights,modulus:1009}).result;assert.equal(result.value,total%1009);cases++;const batch=run('burnside',{fixed},String(representatives.size));assert.ok(batch.frames.length);
}
for(const [mode,input]of[['knuth_optimization',{values:[-1]}],['slope_trick',{operations:[['shift',2,1]]}],['garner',{residues:[1,2],moduli:[4,6],target:7}],['floor_sum',{n:1,m:0,a:0,b:0}],['primitive_root',{prime:15}],['burnside',{fixed:[0,1]}],['polya',{n:2,dihedral:false,weights:[1,1],modulus:8}],['crt',{residues:[0,0],moduli:['9223372036854775807',2]}]])assert.throws(()=>build(mode,input));
console.log(`18 modes: ${cases} inputs, ${snapshots} snapshots, independent DP/grid/integer/permutation/orbit oracles passed`);
