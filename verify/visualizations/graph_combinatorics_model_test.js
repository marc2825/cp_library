const assert=require('node:assert/strict');
const {trace}=require('../../visualizations/graph_combinatorics/model.js');
let batches=0,snapshots=0,seed=310010;
const random=n=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return Math.floor(seed/4294967296*n);};
const vertices=(mask,n)=>Array.from({length:n},(_,v)=>v).filter(v=>mask>>v&1);
const key=group=>group.slice().sort((a,b)=>a-b).join(',');
function sets(n,edges,independent=false){const adjacent=new Set(edges.map(([u,v])=>key([u,v]))),result=[];for(let mask=0;mask<2**n;++mask){const group=vertices(mask,n);if(group.every((u,i)=>group.slice(i+1).every(v=>adjacent.has(key([u,v]))!==independent)))result.push(group);}return result;}
function colors(group,edges){if(!group.length)return 0;const chosen=Array(Math.max(...group)+1).fill(-1),adjacent=new Set(edges.map(([u,v])=>key([u,v])));
  const search=(i,k)=>{if(i===group.length)return true;const v=group[i];for(let c=0;c<k;++c)if(group.slice(0,i).every(u=>chosen[u]!==c||!adjacent.has(key([u,v])))){chosen[v]=c;if(search(i+1,k))return true;}chosen[v]=-1;return false;};for(let k=1;k<=group.length;++k)if(search(0,k))return k;throw new Error('color oracle failed');}
function treeCount(n,edges,weighted,mod){if(!n)return 0;const p=BigInt(mod);let answer=0n;
  for(let mask=0;mask<2**edges.length;++mask){const selected=edges.filter((e,i)=>mask>>i&1);if(selected.length!==n-1)continue;const seen=new Set([0]);for(let round=0;round<n;++round)for(const[u,v]of selected)if(seen.has(u)||seen.has(v)){seen.add(u);seen.add(v);}if(seen.size!==n)continue;let product=1n;for(const[u,v,w=1]of selected)product=product*BigInt(weighted?w:1)%p;answer=(answer+product)%p;}
  return Number((answer+p)%p);
}
function test(mode,input){const original=JSON.stringify(input),result=trace(mode,input),s=result.state;assert.equal(JSON.stringify(input),original);++batches;snapshots+=result.frames.length;
  if(mode==='trees'){
    assert.equal(s.result,treeCount(input.n,input.edges,input.weighted,input.mod??998244353));const p=BigInt(s.mod),norm=x=>Number((x%p+p)%p);
    for(let i=1;i<result.frames.length;++i){const f=result.frames[i],prev=result.frames[i-1];if(f.phase==='eliminate')for(let j=f.pivot;j<f.matrix.length;++j)assert.equal(f.matrix[f.row][j],norm(BigInt(prev.matrix[f.row][j])-BigInt(f.factor)*BigInt(prev.matrix[f.pivot][j])));}
    const L=Array.from({length:s.n},()=>Array(s.n).fill(0n));for(const[u,v,w=1]of s.edges)if(u!==v){const weight=BigInt(s.weighted?w:1);L[u][u]+=weight;L[v][v]+=weight;L[u][v]-=weight;L[v][u]-=weight;}assert.deepEqual(s.laplacian,L.map(row=>row.map(norm)));
  }else{
    const all=sets(s.n,s.edges),independent=sets(s.n,s.edges,true),optimal=mode==='independent'?Math.max(...independent.map(g=>g.length)):Math.max(...all.map(g=>g.length)),oracleColors=colors(vertices((1<<s.n)-1,s.n),s.edges);
    if(mode==='triangles'||mode==='cliques'){
      let expected=mode==='triangles'?all.filter(g=>g.length===3):input.maximal?all.filter(g=>(g.length||input.includeEmpty)&&!all.some(h=>h.length>g.length&&g.every(v=>h.includes(v)))):all.filter(g=>g.length||input.includeEmpty);
      assert.equal(new Set(s.solutions.map(key)).size,s.solutions.length);for(const g of s.solutions)assert.ok(expected.some(h=>key(g)===key(h)));
      if(input.stopAfter==null){assert.deepEqual(s.solutions.map(key).sort(),expected.map(key).sort());assert.equal(s.stopped,false);}else{assert.equal(s.solutions.length,Math.min(expected.length,input.stopAfter));assert.equal(s.stopped,expected.length>=input.stopAfter);}
      assert.equal(s.result,s.solutions.length);
    }else if(mode==='coloring'){
      assert.equal(s.result,oracleColors);assert.equal(s.groups.length,oracleColors);assert.equal(s.colors.length,s.n);
      for(const[u,v]of s.edges)assert.notEqual(s.colors[u],s.colors[v]);assert.ok(s.colors.every(c=>c>=0&&c<oracleColors));
      for(let mask=0;mask<s.dp.length;++mask)assert.equal(s.dp[mask],colors(vertices(mask,s.n),s.edges));
      for(const f of result.frames)for(const[u,v]of s.edges)if(f.colors[u]>=0&&f.colors[v]>=0)assert.notEqual(f.colors[u],f.colors[v]);
    }else{
      assert.equal(s.result,optimal);assert.equal(s.best.length,optimal);assert.ok((mode==='independent'?independent:all).some(g=>key(g)===key(s.best)));
      for(const f of result.frames){assert.ok((mode==='independent'?independent:all).some(g=>key(g)===key(f.current)));
        if(f.phase==='bound'||f.phase==='prune'){
          const graph=f.searchAdj;for(const u of f.candidates)for(const v of f.candidates)if(u!==v&&f.greedyColor[u]===f.greedyColor[v])assert.equal(graph[u]>>v&1,0);
          if(f.proof){assert.ok(f.proof.size+f.proof.bound<=f.proof.bestSize);const available=new Set(f.candidates),bestExtension=Math.max(0,...(mode==='independent'?independent:all).filter(g=>g.every(v=>available.has(v))).map(g=>g.length));assert.ok(bestExtension<=f.proof.bound);}
        }
      }
    }
  }
  const initial=JSON.stringify(result.frames[0]);result.state.current.push(999);result.frames.at(-1).current.push(888);assert.equal(JSON.stringify(result.frames[0]),initial);return result.frames;
}
const modes=['triangles','cliques','maximum','independent','coloring','trees'];
for(let n=0;n<=4;++n){const pairs=[];for(let u=0;u<n;++u)for(let v=u+1;v<n;++v)pairs.push([u,v]);for(let mask=0;mask<2**pairs.length;++mask){const edges=pairs.filter((e,i)=>mask>>i&1);for(const mode of modes)test(mode,{n,edges,mod:101});test('cliques',{n,edges,maximal:true,includeEmpty:true});test('cliques',{n,edges,includeEmpty:true});}}
for(let trial=0;trial<100;++trial){const n=random(9),edges=[];for(let u=0;u<n;++u)for(let v=u+1;v<n;++v)if(random(3)===0)edges.push([u,v]);for(const mode of modes.filter(m=>m!=='trees'))test(mode,{n,edges,stopAfter:['triangles','cliques'].includes(mode)&&trial%3===0?1:null});test('cliques',{n,edges,maximal:true,includeEmpty:trial%2===0,stopAfter:trial%3===0?2:null});
  const size=random(7),multiple=size?Array.from({length:random(11)},()=>[random(size),random(size),random(15)-7]):[];for(const weighted of [false,true])for(const mod of [2,101,998244353])test('trees',{n:size,edges:multiple,weighted,mod,root:size?random(size):-1});}
const swap=test('trees',{n:3,edges:[[0,1,1],[0,2,4],[1,2,1]],mod:5,weighted:true,root:2});assert.ok(swap.some(s=>s.phase==='swap'));
assert.equal(trace('trees',{n:4,edges:[[0,1],[1,2],[2,3],[3,0]],mod:2}).state.result,0);
for(const [mode,input]of [['triangles',{n:2,edges:[[0,0]]}],['cliques',{n:2,edges:[[0,1],[1,0]]}],['trees',{n:2,edges:[],mod:4}],['maximum',{n:9,edges:[]}]])assert.throws(()=>trace(mode,input));
console.log(`combinatorics model: ${batches} batches / ${snapshots} snapshots; subset and coloring search, tree edge subsets, pruning bounds, modular elimination and snapshots passed`);
