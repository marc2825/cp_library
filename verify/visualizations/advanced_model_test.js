'use strict';
const assert=require('node:assert/strict');
const {specs,build}=require('../../visualizations/advanced/model.js');
let state=20261017;const random=n=>{state=(Math.imul(state,1664525)+1013904223)>>>0;return state%n;};
const arr=n=>Array.from({length:n},()=>random(11)-5);
let batches=0,frames=0;
function run(mode,p){const output=build(mode,p);batches++;frames+=output.frames.length;assert.ok(output.frames.length);const before=JSON.stringify(output.frames[0]);output.frames.at(-1).rows.push(['mutation']);if(output.frames.length>1)assert.equal(JSON.stringify(output.frames[0]),before);return output.result;}
function path(adj,u,v){const parent=Array(adj.length).fill(-1),queue=[u];parent[u]=u;for(const x of queue)for(const y of adj[x])if(parent[y]<0){parent[y]=x;queue.push(y);}if(parent[v]<0)return[];let result=[];for(let x=v;;x=parent[x]){result.push(x);if(x===u)break;}return result.reverse();}
function trialPrime(n){if(n<2)return false;for(let d=2;d*d<=n;d++)if(n%d===0)return false;return true;}
for(const [mode,[,sample]]of Object.entries(specs))run(mode,sample);
for(let trial=0;trial<120;trial++){
  const n=random(13),a=arr(n),b=arr(random(10));let operations=[],want=a.slice();
  for(let q=0;q<15;q++){let l=random(n+1),r=random(n+1);if(l>r)[l,r]=[r,l];const kind=['add','chmin','chmax'][random(3)],x=random(21)-10;operations.push([kind,l,r,x]);for(let i=l;i<r;i++)want[i]=kind==='add'?want[i]+x:kind==='chmin'?Math.min(want[i],x):Math.max(want[i],x);}
  assert.deepEqual(run('beats',{a,ops:operations}),want);
  let l=random(n+1),r=random(n+1);if(l>r)[l,r]=[r,l];const k=random(r-l+2)-1,sorted=a.slice(l,r).sort((a,b)=>a-b),expected=sorted[k]??null;
  for(const mode of ['wavelet','persistent-kth'])assert.equal(run(mode,{a,l,r,k}),expected);
  const mod=[1,2,6,1000000007,2000000000][random(5)],m=BigInt(mod),normal=x=>(BigInt(x)%m+m)%m,out=Array(a.length&&b.length?a.length+b.length-1:0).fill(0n);
  for(let i=0;i<a.length;i++)for(let j=0;j<b.length;j++)out[i+j]=(out[i+j]+normal(a[i])*normal(b[j]))%m;
  assert.deepEqual(run('crt',{a,b,mod}),out.map(Number));
  const convex=[0];let slope=-8;for(let i=1;i<b.length;i++){slope+=random(5);convex.push(convex.at(-1)+slope);}if(!b.length)convex.length=0;const mp=Array(a.length&&convex.length?a.length+convex.length-1:0).fill(Infinity);
  for(let i=0;i<a.length;i++)for(let j=0;j<convex.length;j++)mp[i+j]=Math.min(mp[i+j],a[i]+convex[j]);
  assert.deepEqual(run('minplus',{a,b:convex}),mp);
  const rows=random(10),cols=random(10),offsets=arr(cols),minima=Array.from({length:rows},(_,r)=>{let best=-1;for(let c=0;c<cols;c++)if(best<0||(r-c)**2+offsets[c]<(r-best)**2+offsets[best])best=c;return best;});
  for(const mode of ['monotone','smawk'])assert.deepEqual(run(mode,{rows,cols,offsets}),minima);
  const count=1<<random(5),sa=arr(count),sb=arr(count),sc=Array(count).fill(0);
  for(let s=0;s<count;s++)for(let t=s;;t=(t-1)&s){sc[s]+=sa[t]*sb[s^t];if(!t)break;}
  assert.deepEqual(run('subset',{a:sa,b:sb}),sc);
  const lines=Array.from({length:random(12)},()=>[random(11)-5,random(21)-10]),xs=[-5,-3,0,2,7],x=xs[random(xs.length)];
  assert.equal(run('lichao',{xs,lines,x}),lines.length?Math.min(...lines.map(([a,b])=>a*x+b)):null);
}
for(let trial=0;trial<70;trial++){
  const n=1+random(12),initial=arr(n),histories=[initial.slice()],updates=[];
  for(let q=0;q<12;q++){const version=random(histories.length),p=random(n),x=random(21)-10,next=histories[version].slice();next[p]=x;histories.push(next);updates.push([version,p,x]);}
  assert.deepEqual(run('persistent',{a:initial,ops:updates}),histories.map(a=>a.reduce((x,y)=>x+y,0)));
  let a=initial.slice(),operations=[];
  for(let q=0;q<15;q++){let kind=random(4);if(!a.length)kind=0;if(kind===0){const p=random(a.length+1),x=random(20);a.splice(p,0,x);operations.push(['insert',p,x]);}else if(kind===1){const p=random(a.length);a.splice(p,1);operations.push(['erase',p]);}else{let l=random(a.length+1),r=random(a.length+1);if(l>r)[l,r]=[r,l];if(kind===2){a.splice(l,r-l,...a.slice(l,r).reverse());operations.push(['reverse',l,r]);}else{const p=random(a.length-(r-l)+1),part=a.splice(l,r-l);a.splice(p,0,...part);operations.push(['move',l,r,p]);}}}
  assert.deepEqual(run('treap',{a:initial,ops:operations}),a);
  const adj=Array.from({length:n},()=>new Set()),lct=[],answers=[],values=arr(n);
  for(let q=0;q<25;q++){const u=random(n),v=random(n),kind=['link','cut','path'][random(3)],route=path(adj,u,v);if(kind==='link'&&!route.length){adj[u].add(v);adj[v].add(u);}if(kind==='cut'){adj[u].delete(v);adj[v].delete(u);}if(kind==='path')answers.push(route.length?route.reduce((s,v)=>s+values[v],0):null);lct.push([kind,u,v]);}
  assert.deepEqual(run('lct',{n,values,ops:lct}),answers);
  a=initial.slice();operations=[];const moAnswers=[];
  for(let q=0;q<25;q++){if(random(2)){const p=random(n),x=random(6);a[p]=x;operations.push(['set',p,x]);}else{let l=random(n+1),r=random(n+1);if(l>r)[l,r]=[r,l];moAnswers.push(new Set(a.slice(l,r)).size);operations.push(['query',l,r]);}}
  assert.deepEqual(run('mo',{a:initial,ops:operations}),moAnswers);
  const parent=Array.from({length:n},(_,v)=>v&&random(5)?random(v):-1),colors=arr(n),isAncestor=(u,v)=>{while(v>=0){if(u===v)return true;v=parent[v];}return false;};
  assert.deepEqual(run('sack',{parent,colors}),parent.map((_,v)=>new Set(colors.filter((_,w)=>isAncestor(v,w))).size));
  const important=parent.flatMap((_,v)=>random(3)?[]:[v]),closure=new Set(important),lca=(a,b)=>{const seen=new Set();for(let v=a;v>=0;v=parent[v])seen.add(v);for(let v=b;v>=0;v=parent[v])if(seen.has(v))return v;return -1;};
  for(const a of important)for(const b of important){const v=lca(a,b);if(v>=0)closure.add(v);}
  const vt=run('virtual',{parent,important});assert.deepEqual([...vt.vertices].sort((a,b)=>a-b),[...closure].sort((a,b)=>a-b));
  vt.vertices.forEach((v,i)=>{let p=parent[v];while(p>=0&&!closure.has(p))p=parent[p];assert.equal(vt.parent[i],p);});
  const edges=new Map(),dc=[],dcAnswers=[];
  for(let q=0;q<25;q++){const u=random(n),v=random(n),key=[Math.min(u,v),Math.max(u,v)].join(','),count=edges.get(key)||0,kind=random(3);if(kind===0){edges.set(key,count+1);dc.push(['add',u,v]);}else if(kind===1&&count){edges.set(key,count-1);dc.push(['remove',u,v]);}else{const adj=Array.from({length:n},()=>new Set());for(const [key,count]of edges)if(count){const [a,b]=key.split(',').map(Number);adj[a].add(b);adj[b].add(a);}dcAnswers.push(path(adj,u,v).length>0);dc.push(['query',u,v]);}}
  assert.deepEqual(run('connectivity',{n,ops:dc}),dcAnswers);
  assert.deepEqual(run('rollback',{n,ops:[['save'],['merge',0,n-1],['merge',0,n-1],['undo'],['restore']]}),Array.from({length:n},(_,i)=>i));
}
for(let n=1;n<500;n++)assert.equal(run('primality',{n:String(n)}),trialPrime(n));
for(const n of [1,2,4,6,9,15,21,25,77,341,8051,10403]){const factors=run('rho',{n:String(n)});assert.ok(factors);assert.equal(factors.reduce((x,y)=>x*BigInt(y),1n),BigInt(n));if(!trialPrime(n)&&n>1)assert.equal(factors.length,2);}
for(let trial=0;trial<150;trial++){
  const text=Array.from({length:random(20)},()=>String.fromCharCode(97+random(3))).join(''),patterns=Array.from({length:random(10)},()=>Array.from({length:random(5)},()=>String.fromCharCode(97+random(3))).join(''));
  assert.deepEqual(run('aho',{text,patterns}),patterns.map(p=>{let count=0;for(let i=0;i<=text.length;i++)if(text.startsWith(p,i))count++;return count;}));
  const want=new Map();for(let i=0;i<text.length;i++)for(let j=i+1;j<=text.length;j++){const s=text.slice(i,j);if(s===[...s].reverse().join(''))want.set(s,(want.get(s)||0)+1);}
  const result=run('eertree',{text});assert.deepEqual(Object.fromEntries(result.map(x=>[x.text,x.count])),Object.fromEntries(want));
}
assert.throws(()=>build('subset',{a:[],b:[]}));assert.throws(()=>build('subset',{a:[1001],b:[1]}));assert.throws(()=>build('crt',{a:[1],b:[1],mod:0}));assert.throws(()=>build('lichao',{xs:[],lines:[],x:0}));assert.throws(()=>build('minplus',{a:[1],b:[0,2,1]}));assert.throws(()=>build('rho',{n:'18446744073709551615'}));assert.throws(()=>build('virtual',{parent:[0],important:[]}));
console.log(`${batches} batches / ${frames} snapshots: 21 modes, independent arrays/BFS/submasks/strings/trial primes passed`);
