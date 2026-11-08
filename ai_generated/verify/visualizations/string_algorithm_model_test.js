'use strict';
const assert=require('node:assert/strict'),M=require('../../visualizations/string_algorithm/model.js');
let seed=321021,inputs=0,snapshots=0,rejections=0;const random=n=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed%n;};
const eq=(a,b)=>a.length===b.length&&a.every((x,i)=>x===b[i]);
const lex=(a,b)=>{for(let i=0;i<Math.min(a.length,b.length);++i)if(a[i]!==b[i])return a[i]-b[i];return a.length-b.length;};
const common=(a,b)=>{let k=0;while(k<Math.min(a.length,b.length)&&a[k]===b[k])k++;return k;};
const palindrome=a=>eq(a,[...a].reverse());
function direct(a){const n=a.length,z=a.map((_,i)=>common(a,a.slice(i))),p=a.map((_,i)=>{let answer=0;for(let k=1;k<=i;++k)if(eq(a.slice(0,k),a.slice(i+1-k,i+1)))answer=k;return answer;}),sa=Array.from({length:n},(_,i)=>i).sort((i,j)=>lex(a.slice(i),a.slice(j))),lcp=sa.slice(1).map((j,i)=>common(a.slice(sa[i]),a.slice(j))),odd=a.map((_,i)=>{let k=1;while(k<=i&&k<n-i&&palindrome(a.slice(i-k,i+k+1)))k++;return k;}),even=a.map((_,i)=>{let k=0;while(k<i&&k<n-i&&palindrome(a.slice(i-k-1,i+k+1)))k++;return k;});return {z,p,sa,lcp,odd,even};}
const hash=(a,base)=>M.MOD.map(p=>a.reduce((sum,x,i)=>(sum+BigInt(x+1)*(base**BigInt(a.length-i-1)%p))%p,0n));
function trieAnswers(operations){let bag=new Map();const result=[];for(const {op,word}of operations){const key=word.join(','),before=bag.get(key)||0;let value=0;if(op==='+'){value=before+1;bag.set(key,value);}else if(op==='-'){value=Number(before>0);bag.set(key,before-value);}else if(op==='?')value=before;else if(op==='^'){for(const [k,n]of bag)if(eq((k?k.split(',').map(Number):[]).slice(0,word.length),word))value+=n;}else bag=new Map();result.push({op,word,value});}return result;}
function check(raw){const d=M.run(raw),c=d.config;inputs++;snapshots+=d.frames.length;const truth=c.s?direct(c.s):null;
 if(c.mode==='trie'){
  const answers=trieAnswers(c.operations);assert.deepEqual(d.result.answers,answers);
  for(const f of d.frames){assert.deepEqual(f.answers,answers.slice(0,f.answers.length));let bag=new Map();for(const {op,word}of c.operations.slice(0,f.answers.length)){const key=word.join(',');if(op==='+')bag.set(key,(bag.get(key)||0)+1);if(op==='-'&&(bag.get(key)||0)>0)bag.set(key,bag.get(key)-1);if(op==='!')bag=new Map();}
   for(let i=0;i<f.nodes.length;++i){const node=f.nodes[i],prefix=[];let at=i;while(at){prefix.unshift(f.nodes[at].byte);at=f.nodes[at].parent;}let passed=0;for(const [key,count]of bag){const bytes=key?key.split(',').map(Number):[];if(eq(bytes.slice(0,prefix.length),prefix))passed+=count;}assert.equal(node.passed,passed);assert.equal(node.terminal,bag.get(prefix.join(','))||0);for(const [byte,id]of Object.entries(node.edges)){assert.equal(f.nodes[id].parent,i);assert.equal(f.nodes[id].byte,Number(byte));}}
  }
  assert.equal(d.result.size,d.frames.at(-1).nodes[0].passed);
 }else if(c.mode==='hash'){
  for(const f of d.frames)for(let which=0;which<f.hashes.length;++which)if(f.hashes[which])for(let i=0;i<f.hashes[which].length;++i){assert.deepEqual(f.hashes[which][i],hash((which?c.other:c.s).slice(0,i),c.base));assert.deepEqual(f.powers[which][i],M.MOD.map(p=>c.base**BigInt(i)%p));}
  const a=c.s.slice(c.left,c.right),b=c.other.slice(c.otherLeft,c.otherRight);assert.deepEqual(d.result,{hash:hash(a,c.base),otherHash:hash(b,c.base),equal:eq(a,b),lcp:common(a,b)});
 }else if(c.mode==='kmp'){
  const want=[];for(let i=0;i<=c.s.length;++i)if(eq(c.s.slice(i,i+c.pattern.length),c.pattern))want.push(i);assert.deepEqual(d.result,want);
  for(const f of d.frames){const p=direct(c.pattern).p;if(f.p)f.p.forEach((x,i)=>{if(x>=0)assert.equal(x,p[i]);});if(f.positions)assert.deepEqual(f.positions,want.slice(0,f.positions.length));if(f.phase==='search'&&f.i!==undefined){if(f.stage==='compare'||f.stage==='fallback')assert.ok(eq(c.pattern.slice(0,f.length),c.s.slice(f.i-f.length,f.i)));else if(f.stage==='complete')assert.ok(eq(c.pattern.slice(0,f.length),c.s.slice(f.i+1-f.length,f.i+1)));}}
 }else if(c.mode==='z'){
  assert.deepEqual(d.result,truth.z);for(const f of d.frames){f.z.forEach((x,i)=>{if(x>=0)assert.equal(x,truth.z[i]);});assert.ok(eq(c.s.slice(0,f.right-f.left),c.s.slice(f.left,f.right)));if(f.i!==undefined)assert.ok(eq(c.s.slice(0,f.length),c.s.slice(f.i,f.i+f.length)));}
 }else if(c.mode==='prefix'){
  assert.deepEqual(d.result,truth.p);for(const f of d.frames){f.p.forEach((x,i)=>{if(x>=0)assert.equal(x,truth.p[i]);});if(f.i!==undefined&&['start','compare','fallback'].includes(f.stage))assert.ok(eq(c.s.slice(0,f.length),c.s.slice(f.i-f.length,f.i)));}
 }else if(c.mode==='suffix'){
  assert.deepEqual(d.result,truth.sa);for(const f of d.frames){assert.deepEqual([...f.sa].sort((a,b)=>a-b),Array.from({length:c.s.length},(_,i)=>i));if(f.stage==='sort')for(let i=1;i<f.sa.length;++i)assert.ok(lex(f.keys[f.sa[i-1]],f.keys[f.sa[i]])<=0);if(f.stage==='complete')for(let i=0;i<c.s.length;++i)for(let j=0;j<c.s.length;++j)assert.equal(Math.sign(f.rank[i]-f.rank[j]),Math.sign(lex(c.s.slice(i,i+f.covered),c.s.slice(j,j+f.covered))));}
 }else if(c.mode==='lcp'){
  assert.deepEqual(d.result,truth.lcp);for(const f of d.frames){assert.deepEqual(f.sa,truth.sa);f.lcp.forEach((x,i)=>{if(x>=0)assert.equal(x,truth.lcp[i]);});if(f.i!==undefined)assert.ok(eq(c.s.slice(f.i,f.i+f.length),c.s.slice(f.j,f.j+f.length)));}
 }else{
  assert.deepEqual(d.result,{odd:truth.odd,even:truth.even,palindrome:palindrome(c.s.slice(c.left,c.right))});for(const f of d.frames){for(const key of ['odd','even'])f[key].forEach((x,i)=>{if(x>=0)assert.equal(x,truth[key][i]);});if(f.radius!==undefined){assert.ok(f.radius<=truth[f.parity][f.i]);const l=f.i-f.radius+(f.parity==='odd'?1:0),r=f.i+f.radius;assert.ok(palindrome(c.s.slice(l,r)));}if(f.right>=f.left)assert.ok(palindrome(c.s.slice(f.left,f.right+1)));}
 }
 for(const f of d.frames)if(f.stage==='compare')assert.equal(f.match,c.mode==='kmp'&&f.phase==='search'?c.s[f.a]===c.pattern[f.b]:(f.sequence||c.s)[f.a]===(f.sequence||c.s)[f.b]);
 if(d.frames.length>1){const first=structuredClone(d.frames[0]),last=d.frames.at(-1);for(const key of ['z','p','sa','odd','lcp'])if(last[key]?.length){last[key][0]=999;break;}if(last.nodes)last.nodes[0].passed=999;if(last.hashes)last.hashes[0][0][0]=999n;assert.deepEqual(d.frames[0],first);}
 return d;
}
for(const mode of M.MODES)check(M.defaults(mode));
for(let n=0;n<=5;++n)for(let code=0;code<3**n;++code){let x=code;const s=Array.from({length:n},()=>{const a=x%3;x=Math.floor(x/3);return String.fromCharCode(97+a);}).join('');for(const mode of ['z','prefix','suffix','lcp','manacher'])check({...M.defaults(mode),s,left:'0',right:String(n)});}
const binary=[];for(let n=0;n<=5;++n)for(let code=0;code<2**n;++code)binary.push(Array.from({length:n},(_,i)=>code>>i&1?'b':'a').join(''));for(const s of binary)for(const p of binary.filter(x=>x.length<=3))check({...M.defaults('kmp'),s,pattern:p});
for(let trial=0;trial<180;++trial){const a=Array.from({length:random(17)},()=>random(256)),b=Array.from({length:random(17)},()=>random(256)),[l,r]=[random(a.length+1),random(a.length+1)].sort((x,y)=>x-y),[bl,br]=[random(b.length+1),random(b.length+1)].sort((x,y)=>x-y);check({...M.defaults('hash'),format:'hex',s:a.map(x=>x.toString(16).padStart(2,'0')).join(''),other:b.map(x=>x.toString(16).padStart(2,'0')).join(''),left:String(l),right:String(r),otherLeft:String(bl),otherRight:String(br),base:['257','911382323','1000000006'][trial%3]});}
for(let trial=0;trial<120;++trial){const operations=Array.from({length:12},(_,i)=>i===8&&trial%3===0?'!':['+','+','-','?','^'][random(5)]+' '+(random(4)?Array.from({length:1+random(3)},()=>String.fromCharCode(97+random(3))).join(''):'""')).join('\n');check({...M.defaults('trie'),operations});}
for(const mode of M.MODES){check(M.scenario(mode,'empty'));check(M.scenario(mode,'bytes'));if(mode!=='trie')check(M.scenario(mode,'utf8'));}
for(const [mode,change,message]of [
 ['z',{s:'a'.repeat(33)},'32バイト'],['z',{s:'あ'.repeat(11)},'32バイト'],['kmp',{pattern:'a'.repeat(17)},'16バイト'],['z',{format:'hex',s:'f'},'hex'],['z',{format:'hex',s:'gg'},'hex'],['hash',{base:'256'},'base'],['hash',{base:'1000000007'},'base'],['hash',{left:'-1'},'非負'],['hash',{left:'5',right:'4'},'l≤r'],['hash',{right:'12'},'範囲外'],['hash',{otherRight:'6'},'範囲外'],['manacher',{right:'8'},'範囲外'],['trie',{operations:'+ aaaaaaaaa'},'8バイト'],['trie',{operations:'bad'},'操作'],['trie',{operations:Array(17).fill('+ a').join('\n')},'16件'],['trie',{operations:Array.from({length:16},(_,i)=>'+ '+String.fromCharCode(65+i).repeat(8)).join('\n')},'60ノード']
]){assert.throws(()=>M.run({...M.defaults(mode),...change}),e=>e.message.includes(message));rejections++;}
console.log(`string algorithm model: ${inputs} inputs / ${snapshots} snapshots / ${rejections} rejections; direct prefixes/borders/matches/suffixes/radii, polynomial sums, trie dictionaries and every completed/intermediate state passed`);
