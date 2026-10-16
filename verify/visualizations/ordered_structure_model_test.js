"use strict";
const assert=require("node:assert/strict"),M=require("../../visualizations/ordered_structure/model.js");
let seed=35017,operations=0;
const random=n=>{seed^=seed<<13;seed^=seed>>>17;seed^=seed<<5;return(seed>>>0)%n;};
function operate(s,op){
  const before=JSON.stringify(s),r=M.run(s,op);++operations;
  assert.equal(JSON.stringify(s),before);assert.deepEqual(r.frames.at(-1).state,r.state);
  assert.equal(r.frames[0].stable,true);assert.equal(r.frames.at(-1).stable,true);
  return r;
}
for(let round=0;round<200;++round){
  const reverse=!!random(2),compare=(x,y)=>reverse?x>y:x<y,sorted=a=>a.slice().sort((x,y)=>reverse?y-x:x-y);
  const a=Array.from({length:random(17)},()=>random(11)-5);
  let s=M.create("compression",a,{reverse}),r=operate(s,{type:"build"});s=r.state;
  const coordinates=sorted([...new Set(a)]);assert.deepEqual(s.coordinates,coordinates);
  for(let x=-7;x<=7;++x)for(const type of ["index","lower","upper"]){
    const expected=type==="index"?(coordinates.includes(x)?coordinates.indexOf(x):null):coordinates.filter(v=>type==="lower"?compare(v,x):!compare(x,v)).length;
    r=operate(s,{type,x});assert.equal(r.result.value,expected);s=r.state;
  }
  for(const mode of ["set","multiset","map"]){
    s=M.create(mode,[],{reverse});let values=[];const mapping=new Map();
    const choices=mode==="map"?["insert","assign","erase","lookup","bracket","predecessor","successor"]:["insert","insert","erase","erase_all","count","predecessor","successor","pop_first","pop_last"];
    for(let step=0;step<140;++step){
      const type=choices[random(choices.length)],x=random(11)-5,inclusive=!!random(2),value=random(41)-20,op={type,x,inclusive,value};
      let expected;
      if(mode==="map"){
        const present=mapping.has(x);
        if(type==="insert"){expected=!present;if(!present)mapping.set(x,value);}
        else if(type==="assign"){expected=present?"updated":"inserted";mapping.set(x,value);}
        else if(type==="erase"){expected=present;mapping.delete(x);}
        else if(type==="lookup")expected=present?mapping.get(x):null;
        else if(type==="bracket"){if(!present)mapping.set(x,0);expected=mapping.get(x);}
        values=sorted([...mapping.keys()]);
      }else if(type==="insert"){expected=mode==="multiset"||!values.includes(x);if(expected)values.push(x);values=sorted(values);}
      else if(type==="erase"){const i=values.indexOf(x);expected=i>=0;if(i>=0)values.splice(i,1);}
      else if(type==="erase_all"){expected=values.filter(v=>v===x).length;values=values.filter(v=>v!==x);}
      else if(type==="count")expected=values.filter(v=>v===x).length;
      else if(type==="pop_first")expected=values.length?values.shift():null;
      else if(type==="pop_last")expected=values.length?values.pop():null;
      if(type==="predecessor"||type==="successor"){
        const candidates=values.filter(v=>type==="predecessor"?(inclusive?!compare(x,v):compare(v,x)):(inclusive?!compare(v,x):compare(x,v)));
        expected=(type==="predecessor"?candidates.at(-1):candidates[0])??null;
        if(mode==="map"&&expected!==null)expected={key:expected,value:mapping.get(expected)};
      }
      r=operate(s,op);assert.deepEqual(r.result.value,expected,JSON.stringify({mode,op}));s=r.state;
      if(mode==="map")assert.deepEqual(s.entries,values.map(key=>({key,value:mapping.get(key)})));
      else assert.deepEqual(s.values,values);
    }
  }
  const universe=[-3,0,2,5];s=M.create("statistics",[],{universe,reverse});let bag=[];
  for(let step=0;step<200;++step){
    const x=random(11)-5,count=random(5)-1,high=random(11)-5,k=random(bag.length+4)-2;
    const choices=["insert","insert","erase","erase_all","count","rank","rank_le","range","kth","predecessor","successor","clear"],type=choices[random(choices.length)],inclusive=!!random(2),op={type,x,count,high,k,inclusive};
    let expected;const before=JSON.stringify(s);
    if(type==="insert"){expected=universe.includes(x)&&count>=0;if(expected)for(let i=0;i<count;++i)bag.push(x);}
    else if(type==="erase"){expected=universe.includes(x)&&count>=0&&bag.filter(v=>v===x).length>=count;if(expected)for(let i=0;i<count;++i)bag.splice(bag.indexOf(x),1);}
    else if(type==="erase_all"){expected=bag.filter(v=>v===x).length;bag=bag.filter(v=>v!==x);}
    else if(type==="clear"){expected=true;bag=[];}
    else if(type==="count")expected=bag.filter(v=>v===x).length;
    else if(type==="rank")expected=bag.filter(v=>compare(v,x)).length;
    else if(type==="rank_le")expected=bag.filter(v=>!compare(x,v)).length;
    else if(type==="range")expected=compare(x,high)?bag.filter(v=>!compare(v,x)&&compare(v,high)).length:0;
    bag=sorted(bag);
    if(type==="kth")expected=k<0||k>=bag.length?null:bag[k];
    else if(type==="predecessor"||type==="successor"){
      const candidates=bag.filter(v=>type==="predecessor"?(inclusive?!compare(x,v):compare(v,x)):(inclusive?!compare(v,x):compare(x,v)));
      expected=(type==="predecessor"?candidates.at(-1):candidates[0])??null;
    }
    r=operate(s,op);assert.deepEqual(r.result.value,expected,JSON.stringify(op));s=r.state;
    if(["insert","erase"].includes(type)&&!expected)assert.equal(JSON.stringify(s),before,"rejected changes must preserve the complete state");
    assert.equal(s.total,bag.length);
    assert.deepEqual(s.frequency,s.coordinates.map(key=>bag.filter(x=>x===key).length));
    for(let i=1;i<s.bit.length;++i)assert.equal(s.bit[i],s.frequency.slice(i-(i&-i),i).reduce((a,b)=>a+b,0));
  }
}
assert.throws(()=>M.create("statistics",[7],{universe:[1,2]}),/登録候補/);
assert.throws(()=>M.run(M.create("set",[]),{type:"insert",x:1.5}),/整数/);
const huge=M.create("statistics",[],{universe:[1,2]});
let r=M.run(huge,{type:"insert",x:1,count:Number.MAX_SAFE_INTEGER});assert.equal(r.result.value,true);
const previous=JSON.stringify(r.state);r=M.run(r.state,{type:"insert",x:2,count:1});assert.equal(r.result.value,false);assert.equal(JSON.stringify(r.state),previous);
assert.equal(M.run(r.state,{type:"kth",k:Number.MAX_SAFE_INTEGER-1}).result.value,1);
console.log(`ordered visualization model: 200 seeded inputs / ${operations} operations, 5 modes, standard semantics, bounds, duplicates, ranks, BIT blocks and rejection invariants passed`);
