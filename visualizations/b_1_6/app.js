(()=>{
'use strict';
const {specs,paths,build}=window.BModels,$=id=>document.getElementById(id);let mode,frames=[],index=0,timer;
for(const [key,[title]]of Object.entries(specs)){const option=document.createElement('option');option.value=key;option.textContent=title;$('mode').append(option);}
function stop(){clearInterval(timer);timer=null;$('play').textContent='再生';}
function svg(tag,attrs,text){const e=document.createElementNS('http://www.w3.org/2000/svg',tag);for(const[k,v]of Object.entries(attrs))e.setAttribute(k,v);if(text!==undefined)e.textContent=text;$('diagram').append(e);}
function render(){
 if(!frames.length)return;const f=frames[index];$('step-title').textContent=f.title;$('step-note').textContent=f.note;$('step-count').textContent=`${index+1} / ${frames.length}`;
 const tr=document.createElement('tr');for(const c of f.columns){const th=document.createElement('th');th.textContent=String(c);tr.append(th);}$('columns').replaceChildren(tr);$('rows').replaceChildren();
 for(const row of f.rows){const tr=document.createElement('tr');for(const c of row){const td=document.createElement('td');td.textContent=String(c);tr.append(td);}$('rows').append(tr);}
 $('diagram').replaceChildren();
 if(f.graph){
  const g=f.graph,columns=Math.min(5,Math.max(1,g.n)),rows=Math.ceil(g.n/columns);let width=Math.max(650,columns*140+50),height=Math.max(170,rows*115+55),positions=Array.from({length:g.n},(_,i)=>({x:90+(i%columns)*140,y:65+Math.floor(i/columns)*115}));
  if(g.tree){
   const indegree=Array(g.n).fill(0),children=Array.from({length:g.n},()=>[]),depth=Array(g.n).fill(0);for(const[u,v]of g.edges){indegree[v]++;children[u].push(v);}const queue=[];for(let i=0;i<g.n;i++)if(!indegree[i])queue.push(i);
   for(let p=0;p<queue.length;p++){const u=queue[p];for(const v of children[u]){depth[v]=Math.max(depth[v],depth[u]+1);if(--indegree[v]===0)queue.push(v);}}
   const levels=[];for(let i=0;i<g.n;i++)(levels[depth[i]]??=[]).push(i);width=Math.max(650,...levels.map(a=>a.length*120+60));height=Math.max(170,levels.length*110+30);positions=Array(g.n);levels.forEach((a,d)=>a.forEach((id,i)=>positions[id]={x:(i+1)*width/(a.length+1),y:65+d*110}));
  }
  $('diagram').setAttribute('viewBox',`0 0 ${width} ${height}`);
  const defs=document.createElementNS('http://www.w3.org/2000/svg','defs');defs.innerHTML='<marker id="arrow" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="5" markerHeight="5" orient="auto-start-reverse"><path d="M 0 0 L 10 5 L 0 10 z" fill="#61736f"/></marker>';$('diagram').append(defs);
  g.edges.forEach(([u,v,label],id)=>{const a=positions[u],b=positions[v],link=label==='link',color=label==='red'?'#b74e4e':label==='black'?'#304640':link?'#b36a36':'#719889';if(!a||!b)return;const dx=b.x-a.x,dy=b.y-a.y,len=Math.hypot(dx,dy)||1,offset=g.tree?0:(id%3-1)*14+15,cx=(a.x+b.x)/2-dy/len*offset,cy=(a.y+b.y)/2+dx/len*offset;
   if(u===v){svg('path',{d:`M ${a.x-16} ${a.y-15} C ${a.x-55} ${a.y-65},${a.x+55} ${a.y-65},${a.x+16} ${a.y-15}`,fill:'none',stroke:color,'marker-end':'url(#arrow)'});return;}
   svg('path',{d:`M ${a.x+dx/len*24} ${a.y+dy/len*24} Q ${cx} ${cy} ${b.x-dx/len*28} ${b.y-dy/len*28}`,fill:'none',stroke:color,'stroke-width':1.6,...(link?{'stroke-dasharray':'5 4'}:{}),'marker-end':'url(#arrow)'});
   if(label!==undefined)svg('text',{x:cx,y:cy-6,'font-size':10,'text-anchor':'middle',fill:color},String(label));
  });
  positions.forEach((p,i)=>{const active=g.activeVertices?.includes(i),color=g.colors?.[i];svg('circle',{cx:p.x,cy:p.y,r:24,fill:color??(active?'#f0cfb5':'#eaf3ed'),stroke:color??(active?'#bd6935':'#719889')});svg('text',{x:p.x,y:p.y+4,'font-size':11,'text-anchor':'middle',fill:color?'#fff':'#203332'},g.labels?.[i]??String(i));});
 }else{
  const rows=f.grid,cols=Math.max(1,...rows.map(row=>row.length)),longest=Math.max(1,...rows.flat().map(v=>String(v).length)),cellWidth=Math.max(55,Math.min(135,longest*6+12)),width=cols*cellWidth+70,height=Math.max(105,rows.length*42+60);$('diagram').setAttribute('viewBox',`0 0 ${width} ${height}`);
  for(let j=0;j<cols;j++)svg('text',{x:43+cellWidth/2+j*cellWidth,y:20,'font-size':12,'text-anchor':'middle',fill:'#61736f'},j);
  rows.forEach((row,i)=>{svg('text',{x:18,y:53+i*42,'font-size':12,fill:'#61736f'},i);row.forEach((value,j)=>{const active=f.active.some(([r,c])=>r===i&&c===j);svg('rect',{x:43+j*cellWidth,y:31+i*42,width:cellWidth-5,height:33,rx:5,fill:active?'#f0cfb5':'#eaf3ed',stroke:active?'#bd6935':'#a1b3a5'});svg('text',{x:43+(cellWidth-5)/2+j*cellWidth,y:52+i*42,'font-size':String(value).length>10?9:11,'text-anchor':'middle',fill:'#203332'},String(value));});});
  if(!rows.length)svg('text',{x:20,y:60,'font-size':13,fill:'#61736f'},'結果と説明は上と下の欄で確認できます。');
 }
 $('first').disabled=$('previous').disabled=index===0;$('finish').disabled=$('next').disabled=index===frames.length-1;$('play').disabled=frames.length<=1;
}
function run(){stop();try{const batch=build(mode,JSON.parse($('input').value));frames=batch.frames;index=0;$('result').textContent=JSON.stringify(batch.result);$('input-error').textContent='';render();}catch(e){$('input-error').textContent=e.message;}}
function load(){stop();mode=Object.hasOwn(specs,location.hash.slice(1))?location.hash.slice(1):'integer_kth_root';$('mode').value=mode;$('title').textContent=specs[mode][0];$('intro').textContent=specs[mode][2];$('input').value=JSON.stringify(specs[mode][1],null,2);$('documentation').href=`../../docs/${paths[mode]}.md`;run();}
$('mode').addEventListener('change',()=>location.hash=$('mode').value);window.addEventListener('hashchange',load);$('run').addEventListener('click',run);$('sample').addEventListener('click',load);
for(const[id,change]of [['first',()=>0],['previous',()=>Math.max(0,index-1)],['next',()=>Math.min(frames.length-1,index+1)],['finish',()=>frames.length-1]])$(id).addEventListener('click',()=>{stop();index=change();render();});
$('play').addEventListener('click',()=>{if(timer){stop();return;}if(index===frames.length-1)index=0;$('play').textContent='停止';timer=setInterval(()=>{index++;render();if(index===frames.length-1)stop();},450);});load();
})();
