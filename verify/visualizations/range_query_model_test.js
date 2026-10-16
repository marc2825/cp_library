"use strict";
const assert=require("node:assert/strict");
const M=require("../../visualizations/range_query/model.js");
let seed=33017;
const random=n=>{seed^=seed<<13;seed^=seed>>>17;seed^=seed<<5;return(seed>>>0)%n;};
for(const mode of M.modes)for(let round=0;round<100;++round){
  const n=random(17),a=Array.from({length:n},()=>random(11));let s=M.create(mode,a);
  assert.deepEqual(M.values(s),a);
  for(let step=0;step<100;++step){
    const l=random(n+1),r=l+random(n-l+1),p=n?random(n):0,kind=random(4);
    let op,expected=null;
    if(mode==="fenwick"){
      if(n&&kind<2){const x=random(11);op={type:kind===0?"add":"set",p,x};a[p]=kind===0?a[p]+x:x;}
      else if(kind===2){const w=random(300)-2;op={type:"lower",w};expected=0;let sum=0;if(w>0)while(expected<n&&sum+a[expected]<w)sum+=a[expected++];}
      else{op={type:"sum",l,r};expected=a.slice(l,r).reduce((x,y)=>x+y,0);}
    }else if(mode==="range-bit"){
      if(kind<2){const x=random(11)-5;op={type:"add",l,r,x};for(let i=l;i<r;++i)a[i]+=x;}
      else{op={type:"sum",l,r};expected=a.slice(l,r).reduce((x,y)=>x+y,0);}
    }else if(mode==="segtree"){
      if(n&&kind<2){const x=random(21)-10;op={type:"set",p,x};a[p]=x;}
      else{op={type:"sum",l,r};expected=a.slice(l,r).reduce((x,y)=>x+y,0);}
    }else{
      if(kind<2||!n){const f={a:random(3)-1,b:random(11)-5};op={type:"apply",l,r,...f};for(let i=l;i<r;++i)a[i]=f.a*a[i]+f.b;}
      else if(kind===2){const x=random(21)-10;op={type:"set",p,x};a[p]=x;}
      else if(mode==="dual"){op={type:"get",p};expected=a[p];}
      else{op={type:"sum",l,r};expected=a.slice(l,r).reduce((x,y)=>x+y,0);}
    }
    const before=JSON.stringify(s),result=M.run(s,op);
    assert.equal(JSON.stringify(s),before,"run must preserve the input snapshot");
    assert.equal(result.result,expected,JSON.stringify({mode,op}));
    assert.deepEqual(M.values(result.state),a,JSON.stringify({mode,op}));
    assert.deepEqual(result.frames.at(-1).state,result.state);
    for(const frame of result.frames)assert.ok(M.values(frame.state).every(Number.isSafeInteger));
    s=result.state;
  }
}
assert.throws(()=>M.run(M.create("fenwick",[1,-1]),{type:"lower",w:1}),/非負/);
assert.throws(()=>M.run(M.create("lazy",[1,2]),{type:"apply",l:2,r:1,a:1,b:2}),/l ≤ r/);
assert.throws(()=>M.run(M.create("dual",[1]),{type:"get",p:1}),/添字/);
assert.throws(()=>M.run(M.create("lazy",[100]),{type:"apply",l:0,r:1,a:Number.MAX_SAFE_INTEGER,b:1}),/範囲/);
assert.equal(M.run(M.create("fenwick",[0,0,4,0,2]),{type:"lower",w:5}).result,4);
assert.deepEqual(M.compose({a:0,b:2},{a:1,b:5}),{a:0,b:2});
assert.deepEqual(M.compose({a:1,b:5},{a:0,b:2}),{a:0,b:7});
console.log("range visualization model: 500 seeded sequences / 50000 operations, 5 modes, snapshots, bounds and composition passed");
