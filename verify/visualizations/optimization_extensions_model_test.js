'use strict';
const assert=require('node:assert/strict'),model=require('../../visualizations/optimization_extensions/model.js');
const expected=require('../optimization_extensions/visualization_expected.json');
let cases=0,states=0,checks=0,seed=731;
const rnd=n=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed%n;};
const close=(a,b)=>{assert.ok(Math.abs(a-b)<1e-7*(1+Math.abs(b)),`${a} != ${b}`);checks++;};
const dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0);
function run(id,changes={}){
 const input={...structuredClone(model.modes.find(m=>m.id===id).sample),...changes},before=JSON.stringify(input),out=model.trace(id,input);cases++;states+=out.steps.length;
 assert.equal(JSON.stringify(input),before);assert.ok(out.steps.length>=2);
 for(const step of out.steps){assert.equal(typeof step.title,'string');assert.equal(typeof step.detail,'string');assert.ok(step.rows.every(r=>r.length===step.columns.length));for(const p of [...(step.scene.points||[]),...(step.scene.segments||[]).flat(),...(step.scene.polygons||[]).flat()])assert.ok(p.every(Number.isFinite));}
 return out;
}
for(const mode of model.modes)assert.deepEqual(run(mode.id).result,expected[mode.id]);
// Independent 2D pair-of-lines enumeration, including axis constraints.
function vertex(A,b,c){let best=-Infinity;for(let i=0;i<A.length;i++)for(let j=0;j<i;j++){const d=A[i][0]*A[j][1]-A[i][1]*A[j][0];if(!d)continue;const p=[(b[i]*A[j][1]-A[i][1]*b[j])/d,(A[i][0]*b[j]-b[i]*A[j][0])/d];if(A.every((r,k)=>dot(r,p)<=b[k]+1e-8))best=Math.max(best,dot(c,p));}return best;}
for(let trial=0;trial<150;trial++){
 const A=[[1,0],[0,1],[-1,0],[0,-1]],b=[4,4,0,0],c=[rnd(11)-5,rnd(11)-5];for(let j=0;j<3;j++){A.push([rnd(7)-3,rnd(7)-3]);b.push(rnd(15)-6);}
 const want=vertex(A,b,c);
 for(const id of ['linear_program','low_dimensional_lp']){const out=run(id,{A,b,c,max_bases:256});assert.equal(out.meta.status,Number.isFinite(want)?'optimal':'infeasible');if(Number.isFinite(want)){close(out.meta.value,want);out.meta.x.forEach(v=>assert.ok(v>=-1e-8));}}
}
for(const id of ['linear_program','low_dimensional_lp']){
 assert.equal(run(id,{A:[],b:[],c:[1,0]}).meta.status,'unbounded');
 assert.equal(run(id,{A:[[0,0]],b:[-1],c:[1,0]}).meta.status,'infeasible');
 assert.equal(run(id,{max_pivots:0,max_bases:0}).meta.status,'iteration_limit');
}
const subset=(mask,n)=>Array.from({length:n},(_,i)=>i).filter(i=>mask>>i&1);
function independent(set,groups,caps){return caps.every((v,g)=>set.filter(e=>groups[e]===g).length<=v);}
for(let trial=0;trial<100;trial++){
 const n=1+rnd(7),groups1=Array.from({length:n},()=>rnd(3)),groups2=Array.from({length:n},()=>rnd(3)),capacities1=[rnd(3),rnd(3),rnd(3)],capacities2=[rnd(3),rnd(3),rnd(3)],weights=Array.from({length:n},()=>rnd(21)-10),bySize=[0];
 for(let mask=0;mask<1<<n;mask++){const s=subset(mask,n);if(independent(s,groups1,capacities1)&&independent(s,groups2,capacities2))bySize[s.length]=Math.max(bySize[s.length]??-Infinity,s.reduce((v,e)=>v+weights[e],0));}
 const out=run('weighted_matroid_intersection',{weights,groups1,groups2,capacities1,capacities2});assert.deepEqual(out.meta.by_size,bySize);close(out.meta.weight,Math.max(...bySize));assert.ok(independent(out.meta.selected,groups1,capacities1)&&independent(out.meta.selected,groups2,capacities2));
 const capacities=[capacities1,capacities2];let best=0;
 for(let mask=0;mask<3**n;mask++){let q=mask,sets=[[],[]];for(let e=0;e<n;e++){const j=q%3;q=Math.floor(q/3);if(j<2)sets[j].push(e);}if(sets.every((s,j)=>independent(s,groups1,capacities[j])))best=Math.max(best,sets[0].length+sets[1].length);}
 const u=run('matroid_union',{groups:groups1,capacities});close(u.meta.cardinality,best);for(let j=0;j<2;j++)assert.ok(independent(u.meta.assignment.flatMap((v,e)=>v===j?[e]:[]),groups1,capacities[j]));
}
for(let trial=0;trial<80;trial++){
 const n=1+rnd(6),modular=Array.from({length:n},()=>rnd(15)-7),edges=[];for(let i=0;i<n;i++)for(let j=0;j<i;j++)if(rnd(2))edges.push([i,j,rnd(4)]);
 const f=s=>s.reduce((v,e)=>v+modular[e],0)+edges.reduce((v,[a,b,w])=>v+(s.includes(a)!==s.includes(b)?w:0),0);
 let best=0;for(let mask=0;mask<1<<n;mask++)best=Math.min(best,f(subset(mask,n)));
 const out=run('submodular_optimization',{modular,edges});assert.equal(out.meta.status,'optimal');close(out.meta.value,best);close(out.meta.lower,best);close(f(out.meta.selected),best);
 const partial=run('submodular_optimization',{modular,edges,max_iterations:0});assert.ok(partial.meta.lower<=best+1e-8&&partial.meta.value>=best-1e-8);
 const sets=Array.from({length:n},()=>Array.from({length:5},(_,i)=>i).filter(()=>rnd(2))),weights=Array.from({length:n},()=>rnd(13)-6),base=run('base_polyhedron',{sets,weights}).meta;
 const coverage=s=>new Set(s.flatMap(e=>sets[e])).size;
 for(let mask=0;mask<1<<n;mask++){const s=subset(mask,n);assert.ok(s.reduce((v,e)=>v+base.base[e],0)<=coverage(s));checks++;}
 close(base.base.reduce((a,b)=>a+b,0),coverage(subset((1<<n)-1,n)));
 // Every greedy vertex of a base polyhedron corresponds to a permutation.
 let max=-Infinity;function visit(s,left,prev,value){if(!left.length){max=Math.max(max,value);return;}for(const e of left){const next=[...s,e],v=coverage(next);visit(next,left.filter(a=>a!==e),v,value+weights[e]*(v-prev));}}
 visit([],Array.from({length:n},(_,i)=>i),0,0);close(base.value,max);
}
for(let trial=0;trial<100;trial++){
 const A=Array.from({length:3},()=>[rnd(7)-3,rnd(7)-3]),b=Array.from({length:3},()=>rnd(15)/2-3),c=[rnd(9)-4,rnd(9)-4],lower=[-2,-2],upper=[3,3];let best=-Infinity;
 for(let a=-2;a<=3;a++)for(let z=-2;z<=3;z++)if(A.every((r,i)=>dot(r,[a,z])<=b[i]))best=Math.max(best,dot(c,[a,z]));
 const out=run('integer_program',{A,b,c,lower,upper});assert.equal(out.meta.status,Number.isFinite(best)?'optimal':'infeasible');if(Number.isFinite(best))close(out.meta.value,best);
 const points=Array.from({length:7},()=>[rnd(31)-15,1+rnd(9)]),ratio=Math.max(...points.map(p=>p[0]/p[1])),r=run('fractional_programming',{points,denominator_lower_bound:1,lower:-20,upper:20,max_iterations:60});close(r.meta.ratio,ratio);assert.ok(r.meta.lower<=ratio+1e-7&&r.meta.upper>=ratio-1e-7);
}
assert.equal(run('integer_program',{max_nodes:0}).meta.status,'node_limit');
function multiply(a,b){const c=Array(a.length+b.length-1).fill(0);a.forEach((x,i)=>b.forEach((y,j)=>c[i+j]+=x*y));return c;}
for(let a=-6;a<=6;a++)for(let b=-3;b<=3;b++){
 const common=[b,1],out=run('omega_rational',{numerator:multiply([a,1],common),denominator:common}).meta;
 assert.deepEqual(out.numerator,[String(a),'1']);assert.deepEqual(out.denominator,['1']);assert.equal(out.sign,1);checks++;
}
assert.equal(run('omega_rational',{numerator:[-1],denominator:[0,-1]}).meta.sign,1);
assert.deepEqual(run('omega_rational',{numerator:[0,0],denominator:[-1,2]}).meta.numerator,[]);
for(const [id,x]of [['linear_program',{b:[]}],['linear_program',{c:[Infinity]}],['low_dimensional_lp',{c:[1]}],['fractional_programming',{points:[[1,0]]}],['fractional_programming',{denominator_lower_bound:100}],['matroid_union',{groups:[3],capacities:[[1]]}],['weighted_matroid_intersection',{weights:[]}],['base_polyhedron',{sets:[]}],['submodular_optimization',{edges:[[0,1,-1]]}],['integer_program',{lower:[]}],['omega_rational',{denominator:[0]}],['omega_rational',{numerator:[.5]}]])assert.throws(()=>run(id,x));
console.log(`optimization model: ${cases} inputs, ${states} states, ${checks} independent comparisons, limits and 12 invalid inputs passed`);
