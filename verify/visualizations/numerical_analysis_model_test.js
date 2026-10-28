'use strict';
const assert=require('node:assert/strict'),model=require('../../visualizations/numerical_analysis/model.js');
const expected=require('../numerical_analysis/visualization_expected.json');let checks=0;
function close(a,b,tol=1e-7){assert.ok(Math.abs(a-b)<=tol*(1+Math.abs(b)),`${a} != ${b}`);checks++;}
for(const mode of model.modes){const input=JSON.stringify(mode.sample),out=model.trace(mode.id,mode.sample);assert.deepEqual(out.result,expected[mode.id]);assert.equal(JSON.stringify(mode.sample),input);assert.ok(out.steps.length>1);for(const step of out.steps){assert.equal(typeof step.title,'string');assert.equal(typeof step.detail,'string');assert.equal(step.rows.every(row=>row.length===step.columns.length),true);}checks++;}
const run=(id,changes={})=>model.trace(id,{...structuredClone(model.modes.find(m=>m.id===id).sample),...changes});
for(let a=-8;a<=8;a++)for(let b=a;b<=8;b++){
 const roots=a===b?[a]:[a,b];const out=run('quadratic',{a:1,b:-a-b,c:a*b});assert.equal(out.meta.roots.length,roots.length);out.meta.roots.forEach((r,i)=>close(r,roots[i]));
}
for(let i=1;i<=50;i++)for(const id of ['bisection','brent','newton']){const a=i/5;const out=run(id,{coefficients:[-a,0,1],lower:0,upper:Math.max(2,a),start:Math.max(1,a/2),tolerance:1e-12});assert.equal(out.meta.status,'converged');close(out.meta.value,Math.sqrt(a));}
assert.equal(run('bisection',{coefficients:[1,0,1]}).meta.status,'invalid_bracket');
assert.equal(run('newton',{coefficients:[1,0,1],start:0}).meta.status,'zero_derivative');
assert.equal(run('brent',{max_iterations:0}).meta.status,'max_iterations');
for(let k=0;k<=7;k++){
 const p=Array(k+1).fill(0);p[k]=1;
 const a=run('adaptive_simpson',{function:'polynomial',coefficients:p,tolerance:1e-10,max_depth:16});assert.equal(a.meta.status,'converged');close(a.meta.value,1/(k+1));
 if(k<=3)close(run('simpson',{coefficients:p}).meta.value,1/(k+1));
 close(run('adaptive_simpson',{function:'polynomial',coefficients:p,lower:1,upper:0,tolerance:1e-10,max_depth:16}).meta.value,-1/(k+1));
}
assert.equal(run('adaptive_simpson',{max_depth:0,tolerance:0}).meta.status,'max_depth');
for(let i=-40;i<=40;i++)close(run('summation',{values:[1e30,i,-1e30]}).meta.compensated,i,0);
for(let i=1;i<=64;i++)close(run('log_sum_exp',{values:Array(i).fill(1000)}).meta.value,1000+Math.log(i));
assert.deepEqual(run('log_sum_exp',{values:[]}).result,['-Infinity']);
assert.equal(run('tolerance',{a:0,b:1e-9,absolute:1e-8,relative:0}).meta.accepted,true);
assert.equal(run('tolerance',{a:1,b:2,absolute:0,relative:.1}).meta.accepted,false);
function fromRoots(roots){let p=[1];for(const r of roots){const q=Array(p.length+1).fill(0);p.forEach((c,i)=>{q[i]-=r*c;q[i+1]+=c;});p=q;}return p;}
for(let i=-4;i<=4;i++){
 const roots=[i,i,i+1,i+2],out=run('polynomial_roots',{coefficients:fromRoots(roots)});assert.equal(out.meta.status,'converged');assert.equal(out.meta.count,3);out.meta.intervals.forEach((v,j)=>{assert.equal(v.count,1);close(v.value,i+j);});
}
assert.equal(run('polynomial_roots',{coefficients:[1,0,1]}).meta.count,0);
assert.equal(run('polynomial_roots',{coefficients:[0,0]}).meta.all_real,true);
const cluster=run('polynomial_roots',{max_subdivisions:0});assert.equal(cluster.meta.status,'max_iterations');assert.equal(cluster.meta.intervals.reduce((s,i)=>s+i.count,0),3);
for(let n=1;n<=12;n++){
 const diagonal=Array(n).fill(4),lower=Array(n-1).fill(-1),upper=Array(n-1).fill(1),solution=Array.from({length:n},(_,i)=>i-3),rhs=solution.map((s,i)=>4*s-(i?solution[i-1]:0)+(i+1<n?solution[i+1]:0));
 const out=run('tridiagonal',{diagonal,lower,upper,rhs});out.meta.solution.forEach((v,i)=>close(v,solution[i]));
}
assert.equal(run('tridiagonal',{diagonal:[0],lower:[],upper:[],rhs:[1]}).meta.status,'singular');
assert.equal(run('tridiagonal').meta.swaps[0],true);
for(const [id,changes]of [['simpson',{subdivisions:3}],['tolerance',{relative:-1}],['tridiagonal',{lower:[]}],['polynomial_roots',{coefficients:[.5,1]}],['summation',{values:[Infinity]}],['summation',{values:[1e308,1e308]}],['brent',{max_iterations:10000}]])assert.throws(()=>run(id,changes));
console.log(`numerical model: ${checks} sample/analytic comparisons, repeated roots, pivoting, limits and invalid inputs passed`);
