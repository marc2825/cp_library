'use strict';
const assert=require('node:assert/strict');
const model=require('../../visualizations/string_extensions/model.js');
const expected=require('../string_extensions/visualization_expected.json');
let checks=0;
const same=(a,b)=>{assert.deepEqual(a,b);checks++;};
const eq=(a,b)=>a.length===b.length&&a.every((v,i)=>v===b[i]);
const positions=(s,p)=>Array.from({length:s.length+1},(_,i)=>i).filter(i=>eq(s.slice(i,i+p.length),p));
const words=n=>{const out=[];for(let k=0;k<=n;k++)for(let mask=0;mask<2**k;mask++)out.push(Array.from({length:k},(_,i)=>(mask>>i)&1));return out;};
const subs=s=>{const out=new Set();for(let l=0;l<s.length;l++)for(let r=l+1;r<=s.length;r++)out.add(s.slice(l,r).join(','));return [...out].sort((a,b)=>{const x=a.split(',').map(Number),y=b.split(',').map(Number);for(let i=0;i<Math.min(x.length,y.length);i++)if(x[i]!==y[i])return x[i]-y[i];return x.length-y.length;});};
const format=s=>'hex:'+s.map(x=>x.toString(16).padStart(2,'0')).join('');
const lcs=(a,b)=>{let d=Array(b.length+1).fill(0);for(const x of a){const e=[0];for(let j=0;j<b.length;j++)e.push(x===b[j]?d[j]+1:Math.max(d[j+1],e[j]));d=e;}return d.at(-1);};
const result=(id,input)=>model.trace(id,input).result;
for(const m of model.modes){const input=structuredClone(m.sample),before=structuredClone(input),trace=model.trace(m.id,input);same(trace.result,expected[m.id]);same(input,before);assert.ok(trace.steps.length>=2);for(const step of trace.steps){assert.ok(step.title&&step.columns&&step.rows);}}
for(const s of words(6)){
 const all=new Set();for(let mask=0;mask<2**s.length;mask++)all.add(s.filter((_,i)=>mask>>i&1).join(','));
 for(const modulus of ['1','6','18446744073709551615'])same(result('subsequence',{text:s,pattern:[],modulus}),[String(BigInt(all.size)%BigInt(modulus))]);
 const b=model.trace('bwt',{text:s});same(b.meta.recovered,s);assert.equal(b.meta.bwt.last.filter(x=>x===0).length,1);
 const factors=model.trace('lz77',{text:s}).meta.factors;let at=0,restored=[];
 for(const [source,length,c] of factors){let best=0;for(let p=0;p<at;p++){let k=0;while(at+k<s.length&&s[p+k]===s[at+k])k++;best=Math.max(best,k);}assert.equal(length,Math.max(1,best));if(source<0)restored.push(c);else for(let k=0;k<length;k++)restored.push(restored[source+k]);at+=length;}same(restored,s);
 const allsubs=subs(s),p=s.slice(0,2),found=positions(s,p);
 for(const k of [0,Math.floor(allsubs.length/2),allsubs.length])same(result('sam_utilities',{text:s,pattern:p,k}),[k<allsubs.length?format(allsubs[k].split(',').map(Number)):'none',...found.map(String)]);
 for(const p of words(2))same(result('fm_index',{text:s,pattern:p,sample_rate:3}),[positions(s,p).length,...positions(s,p)].map(String));
 const periods=[],borders=[];for(let k=1;k<=s.length;k++){if(eq(s.slice(k),s.slice(0,s.length-k)))periods.push(k);if(k<s.length&&eq(s.slice(0,k),s.slice(s.length-k)))borders.push(k);}same(result('borders',{text:s}),[...borders,...periods,found.length].map(String));
 for(let i=0;i<=s.length;i++){const z=t=>{let n=0;while(i+n<t.length&&t[n]===t[i+n])n++;return n;},extended=s.concat([0]);same(result('online_z',{text:s,append:[0],position:i}).slice(0,2),[z(s),z(extended)].map(String));}
 if(s.length)for(let p=1;p<=s.length;p++){let root=1;while(s.length%root||!eq(s.slice(root),s.slice(0,s.length-root)))root++;same(result('period_queries',{text:s,left:0,right:s.length,period:p}),[Number(periods.includes(p)),root,s.length/root].map(String));}
}
for(const a of words(3))for(const b of words(3)){const n=Math.min(a.length,b.length);for(let p=0;p<=n;p++)for(let l=0;l<=n;l++)for(let r=l;r<=n;r++)same(result('semi_local',{a,b,prefix:p,left:l,right:r}),[lcs(a.slice(0,p),b.slice(l,r)),lcs(a,b.slice(l,r)),lcs(a.slice(l,r),b),lcs(a.slice(0,p),b.slice(l)),lcs(a.slice(l),b.slice(0,p))].map(String));}
let seed=20261027;const rand=n=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed%n;};
for(let trial=0;trial<120;trial++){
 const documents=Array.from({length:rand(5)},()=>Array.from({length:rand(9)},()=>rand(3))),p=Array.from({length:rand(4)},()=>rand(3)),count=documents.reduce((n,s)=>n+positions(s,p).length,0),distinct=new Set(documents.flatMap(subs)).size,ids=documents.flatMap((s,i)=>positions(s,p).length?[i]:[]);
 same(result('generalized_sam',{documents,pattern:p}),[count,distinct,...ids].map(String));
 const operations=[],s=[];for(let i=0;i<25;i++){const code=rand(s.length?4:2),name=['push_front','push_back','pop_front','pop_back'][code],c=rand(3);operations.push(code<2?[name,c]:[name]);if(code===0)s.unshift(c);if(code===1)s.push(c);if(code===2)s.shift();if(code===3)s.pop();}
 const palindrome=w=>eq(w,w.slice().reverse());let countPal=0,pre=0,suf=0;for(let l=0;l<s.length;l++)for(let r=l+1;r<=s.length;r++)if(palindrome(s.slice(l,r))){countPal++;if(l===0)pre=Math.max(pre,r);if(r===s.length)suf=Math.max(suf,r-l);}
 same(result('deque_eertree',{operations}),[subs(s).filter(x=>palindrome(x.split(',').map(Number))).length,countPal,pre,suf].map(String));
}
same(result('fm_index',{text:[0,255,0],pattern:[0],sample_rate:1}),['2','0','2']);
same(result('bwt',{text:''}),['0','0','1']);
same(result('generalized_sam',{documents:['',''],pattern:''}),['2','0','0','1']);
same(result('deque_eertree',{operations:[]}),['0','0','0','0']);
for(const [id,input] of [['subsequence',{text:'a',pattern:'',modulus:'0'}],['bwt',{text:[256]}],['fm_index',{text:'a',pattern:'a',sample_rate:0}],['semi_local',{a:'a',b:'a',prefix:2,left:0,right:1}],['deque_eertree',{operations:[['pop_back']]}],['online_z',{text:'a',append:'b',position:2}],['period_queries',{text:'a',left:0,right:1,period:0}],['generalized_sam',{documents:Array(9).fill(''),pattern:''}],['sam_utilities',{text:'a',pattern:'a',k:-1}]])assert.throws(()=>model.trace(id,input));
console.log(`string extensions model: ${checks} independent comparisons, 11 C++ fixtures, byte/empty/error boundaries passed`);
