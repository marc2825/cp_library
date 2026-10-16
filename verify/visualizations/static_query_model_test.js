"use strict";
const assert=require("node:assert/strict"),M=require("../../visualizations/static_query/model.js");
let seed=34017;
const random=n=>{seed^=seed<<13;seed^=seed>>>17;seed^=seed<<5;return(seed>>>0)%n;};
let operations=0;
function operate(s,op){
  const before=JSON.stringify(s),r=M.run(s,op);++operations;
  assert.equal(JSON.stringify(s),before);assert.deepEqual(r.frames.at(-1).state,r.state);
  assert.equal(r.frames[0].stable,true);assert.equal(r.frames.at(-1).stable,true);
  return r;
}
for(let round=0;round<160;++round){
  const a=Array.from({length:random(17)},()=>random(9)-4);
  for(const order of ["min","max"]){
    const isBetter=(x,y)=>order==="min"?x<y:x>y;
    let s=M.create("sparse",a,{order});s=operate(s,{type:"build"}).state;
    for(let l=0;l<=a.length;++l)for(let r=l;r<=a.length;++r){
      const expected=l===r?(order==="min"?"∞":"−∞"):(order==="min"?Math.min(...a.slice(l,r)):Math.max(...a.slice(l,r)));
      const q=operate(s,{type:"query",l,r});assert.equal(q.result,expected);s=q.state;
    }
    for(const strict of [false,true])for(const direction of ["previous","next"]){
      s=M.create("stack",a,{order,strict,direction});
      for(let step=0;step<a.length;++step){
        const i=direction==="previous"?step:a.length-1-step,delta=direction==="previous"?-1:1;let expected=-1;
        for(let j=i+delta;j>=0&&j<a.length;j+=delta)if(strict?isBetter(a[j],a[i]):!isBetter(a[i],a[j])){expected=j;break;}
        const r=operate(s,{type:"step"});assert.equal(r.result,expected);s=r.state;assert.equal(s.answers[i],expected);
      }
    }
    for(let k=1;k<=a.length+1;++k){
      s=M.create("window",a,{order,k});const expected=[];
      for(let i=0;i<a.length;++i){s=operate(s,{type:"step"}).state;if(i+1>=k){let best=i+1-k;for(let j=best+1;j<=i;++j)if(isBetter(a[j],a[best]))best=j;expected.push({l:i+1-k,r:i+1,index:best,value:a[best]});}}
      assert.deepEqual(s.outputs,expected);
    }
    s=M.create("queue",a,{order});const queue=[];let left=0,next=0;
    for(let i=0;i<100;++i){
      let op;
      if(queue.length&&random(3)===0){op={type:"pop"};queue.shift();++left;}
      else{const value=random(11)-5;op={type:"push",value};queue.push(value);++next;}
      s=operate(s,op).state;assert.deepEqual(M.logical(s),queue);
      if(queue.length){let best=0;for(let j=1;j<queue.length;++j)if(isBetter(queue[j],queue[best]))best=j;
        assert.equal(s.candidates[0].index,left+best);assert.equal(s.candidates[0].value,queue[best]);
        const r=operate(s,{type:"query"});assert.equal(r.result,queue[best]);s=r.state;}
      assert.equal(s.items.length,next);
    }
  }
  for(const aggregation of ["sum","concat"]){
    let s=M.create("swag",a,{aggregation});const queue=[];
    for(let i=0;i<100;++i){
      const before=queue.slice();
      let op;
      if(queue.length&&random(3)===0){op={type:"pop"};queue.shift();}
      else{const value=random(11)-5;op={type:"push",value};queue.push(aggregation==="concat"?String(value):value);}
      const mutation=operate(s,op);s=mutation.state;assert.deepEqual(M.logical(s),queue);
      const expected=aggregation==="concat"?queue.join(""):queue.reduce((x,y)=>x+y,0),r=operate(s,{type:"query"});
      assert.equal(r.result,expected);assert.equal(M.fold(r.state),expected);s=r.state;
      // Transferring between stacks must preserve the complete queue before the final pop.
      if(op.type==="pop")for(const frame of mutation.frames){
        const after=frame.title.startsWith("先頭")||frame.title==="操作完了";
        assert.deepEqual(M.logical(frame.state),after?queue:before);
        const items=after?queue:before;
        assert.equal(M.fold(frame.state),aggregation==="concat"?items.join(""):items.reduce((x,y)=>x+y,0));
      }
    }
  }
}
assert.equal(M.run(M.create("swag",[]),{type:"query"}).result,"");
assert.throws(()=>M.run(M.create("queue",[]),{type:"pop"}),/空/);
assert.throws(()=>M.run(M.create("queue",[]),{type:"query"}),/空/);
assert.throws(()=>M.run(M.create("sparse",[1]),{type:"query",l:1,r:0}),/l ≤ r/);
assert.throws(()=>M.create("window",[1],{k:0}),/正の整数/);
assert.throws(()=>M.run(M.create("stack",[]),{type:"step"}),/すべて/);
console.log(`static visualization model: 160 seeded arrays, ${operations} operations, 5 modes, ties, nearest indices, FIFO order and snapshots passed`);
