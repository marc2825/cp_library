(()=>{
'use strict';
const {specs,paths,build}=window.TransformAModels,$=id=>document.getElementById(id);let mode,frames=[],index=0,timer;
for(const [key,[title]]of Object.entries(specs)){const option=document.createElement('option');option.value=key;option.textContent=title;$('mode').append(option);}
function stop(){clearInterval(timer);timer=null;$('play').textContent='再生';}
function svg(tag,attrs,text){const e=document.createElementNS('http://www.w3.org/2000/svg',tag);for(const[k,v]of Object.entries(attrs))e.setAttribute(k,v);if(text!==undefined)e.textContent=text;$('diagram').append(e);}
function render(){
 if(!frames.length)return;const f=frames[index];$('step-title').textContent=f.title;$('step-note').textContent=f.note;$('step-count').textContent=`${index+1} / ${frames.length}`;
 const tr=document.createElement('tr');for(const c of f.columns){const th=document.createElement('th');th.textContent=String(c);tr.append(th);}$('columns').replaceChildren(tr);$('rows').replaceChildren();
 for(const row of f.rows){const tr=document.createElement('tr');for(const c of row){const td=document.createElement('td');td.textContent=String(c);tr.append(td);}$('rows').append(tr);}
 $('diagram').replaceChildren();const rows=f.grid,cols=Math.max(1,...rows.map(row=>row.length)),width=cols*55+70,height=Math.max(105,rows.length*42+60);$('diagram').setAttribute('viewBox',`0 0 ${width} ${height}`);
 for(let j=0;j<cols;j++)svg('text',{x:68+j*55,y:20,'font-size':12,'text-anchor':'middle',fill:'#61736f'},j);
 rows.forEach((row,i)=>{svg('text',{x:18,y:53+i*42,'font-size':12,fill:'#61736f'},i);row.forEach((value,j)=>{const active=f.active.some(([r,c])=>r===i&&c===j);svg('rect',{x:43+j*55,y:31+i*42,width:50,height:33,rx:5,fill:active?'#f0cfb5':'#eaf3ed',stroke:active?'#bd6935':'#a1b3a5'});svg('text',{x:68+j*55,y:52+i*42,'font-size':11,'text-anchor':'middle',fill:'#203332'},String(value));});});
 if(!rows.length)svg('text',{x:20,y:60,'font-size':13,fill:'#61736f'},'空の状態');
 $('first').disabled=$('previous').disabled=index===0;$('finish').disabled=$('next').disabled=index===frames.length-1;$('play').disabled=frames.length<=1;
}
function run(){stop();try{const batch=build(mode,JSON.parse($('input').value));frames=batch.frames;index=0;$('result').textContent=JSON.stringify(batch.result);$('input-error').textContent='';render();}catch(e){$('input-error').textContent=e.message;}}
function load(){stop();mode=Object.hasOwn(specs,location.hash.slice(1))?location.hash.slice(1):'karatsuba';$('mode').value=mode;$('title').textContent=specs[mode][0];$('intro').textContent=specs[mode][2];$('input').value=JSON.stringify(specs[mode][1],null,2);$('documentation').href=`../../docs/${paths[mode]}.md`;run();}
$('mode').addEventListener('change',()=>location.hash=$('mode').value);window.addEventListener('hashchange',load);$('run').addEventListener('click',run);$('sample').addEventListener('click',load);
for(const[id,change]of [['first',()=>0],['previous',()=>Math.max(0,index-1)],['next',()=>Math.min(frames.length-1,index+1)],['finish',()=>frames.length-1]])$(id).addEventListener('click',()=>{stop();index=change();render();});
$('play').addEventListener('click',()=>{if(timer){stop();return;}if(index===frames.length-1)index=0;$('play').textContent='停止';timer=setInterval(()=>{index++;render();if(index===frames.length-1)stop();},450);});load();
})();
