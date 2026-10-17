(()=>{
  'use strict';
  const {specs,build}=window.APriorityModels,$=id=>document.getElementById(id),select=$('mode');
  const {paths}=window.APriorityModels;
  for(const [key,[title]]of Object.entries(specs)){const option=document.createElement('option');option.value=key;option.textContent=title;select.append(option);}
  let mode,frames=[],index=0,timer=null;
  function stop(){clearInterval(timer);timer=null;$('play').textContent='再生';}
  function svg(tag,attributes,text){const node=document.createElementNS('http://www.w3.org/2000/svg',tag);for(const [key,value]of Object.entries(attributes))node.setAttribute(key,value);if(text!==undefined)node.textContent=String(text);$('diagram').append(node);return node;}
  function draw(frame){
    $('diagram').replaceChildren();let height=150;
    if(frame.values?.length){const a=frame.values,width=680/Math.max(1,a.length);a.forEach((v,i)=>{const active=!frame.active||frame.active[0]<=i&&i<frame.active[1];svg('rect',{x:35+i*width,y:40,width:Math.max(8,width-5),height:48,rx:7,fill:active?'#e9c5a9':'#eaf3ed',stroke:active?'#bd6935':'#ccd9ce'});svg('text',{x:35+(i+.5)*width-2,y:68,'text-anchor':'middle','font-size':13,fill:'#203332'},v);svg('text',{x:35+(i+.5)*width-2,y:110,'text-anchor':'middle','font-size':11,fill:'#61736f'},i);});
    }else if(frame.grid){
      const grid=frame.grid,h=grid.length,w=grid[0]?.length||0,cell=Math.min(58,650/Math.max(1,w));height=Math.max(150,65+h*cell);
      grid.forEach((row,x)=>row.forEach((v,y)=>{svg('rect',{x:50+y*cell,y:35+x*cell,width:cell-3,height:cell-3,rx:4,fill:v?'#e9c5a9':'#eaf3ed',stroke:'#ccd9ce'});svg('text',{x:50+(y+.5)*cell,y:39+(x+.5)*cell,'text-anchor':'middle','font-size':13},v);}));
      svg('text',{x:30,y:20,'font-size':12,fill:'#61736f'},'点の重み（行がx、列がy）');
    }else if(frame.intervals?.length){
      const a=frame.intervals,min=Math.min(...a.map(p=>p[0])),max=Math.max(...a.map(p=>p[1])),scale=650/Math.max(1,max-min);height=70+a.length*52;
      a.forEach(([l,r],i)=>{const y=35+i*52;svg('rect',{x:50+(l-min)*scale,y,width:Math.max(2,(r-l)*scale),height:24,rx:5,fill:'#e9c5a9',stroke:'#bd6935'});svg('text',{x:50+(l-min)*scale,y:y+40,'font-size':12},l);svg('text',{x:50+(r-min)*scale,y:y+40,'font-size':12,'text-anchor':'end'},`${r}（含まない）`);});
    }else if(Array.isArray(frame.edges)){
      const ids=[...new Set(frame.edges.flat().concat(frame.rows.map(r=>r[0]).filter(Number.isInteger)))].sort((a,b)=>a-b);
      if(ids.length<=40&&ids.length){
        const reverse=mode.endsWith('disjoint_set_union')||mode==='merge_tree'||mode==='persistent_stack'||mode==='persistent_queue';
        const directed=frame.edges.map(([u,v])=>reverse?[v,u]:[u,v]),children=new Map(ids.map(v=>[v,[]])),incoming=new Map(ids.map(v=>[v,0]));
        for(const[u,v]of directed){children.get(u)?.push(v);incoming.set(v,(incoming.get(v)||0)+1);}
        const roots=ids.filter(v=>!incoming.get(v)),depth=new Map(),levels=[];let queue=roots.length?roots:[ids[0]];
        queue.forEach(v=>depth.set(v,0));for(let i=0;i<queue.length;i++){const v=queue[i],d=depth.get(v);(levels[d]??=[]).push(v);for(const c of children.get(v)||[])if(!depth.has(c)){depth.set(c,d+1);queue.push(c);}}
        for(const v of ids)if(!depth.has(v))(levels[0]??=[]).push(v);
        const positions=new Map();height=Math.max(150,levels.length*90+20);levels.forEach((row,d)=>row.forEach((v,i)=>positions.set(v,{x:35+(i+.5)*690/row.length,y:45+d*90})));
        for(const[u,v]of frame.edges){const a=positions.get(u),b=positions.get(v);svg('line',{x1:a.x,y1:a.y,x2:b.x,y2:b.y,stroke:'#a1b3a5','stroke-width':2});}
        for(const[v,p]of positions){const row=frame.rows.find(r=>r[0]===v),key=['treap','rbst','ordered_set'].includes(mode)&&row?row[1]:v;svg('circle',{cx:p.x,cy:p.y,r:23,fill:'#eaf3ed',stroke:'#1b7361'});svg('text',{x:p.x,y:p.y+5,'text-anchor':'middle','font-size':13,fill:'#203332'},key);if(key!==v)svg('text',{x:p.x,y:p.y+38,'text-anchor':'middle','font-size':10,fill:'#61736f'},`ID ${v}`);}
      }else svg('text',{x:25,y:55,'font-size':14},'共有関係は下のノードID表で確認できます。');
    }else svg('text',{x:25,y:55,'font-size':15,fill:'#61736f'},'下の表で、この段階の候補と状態を確認できます。');
    $('diagram').setAttribute('viewBox',`0 0 760 ${height}`);
  }
  function render(){if(!frames.length)return;const f=frames[index];$('step-title').textContent=f.title;$('step-note').textContent=f.note;$('step-count').textContent=`${index+1} / ${frames.length}`;const tr=document.createElement('tr');for(const c of f.columns){const th=document.createElement('th');th.textContent=String(c);tr.append(th);}$('columns').replaceChildren(tr);$('rows').replaceChildren();for(const row of f.rows){const tr=document.createElement('tr');for(const c of row){const td=document.createElement('td');td.textContent=String(c);tr.append(td);}$('rows').append(tr);}draw(f);$('first').disabled=$('previous').disabled=index===0;$('finish').disabled=$('next').disabled=index===frames.length-1;$('play').disabled=frames.length<=1;}
  function run(){stop();try{const batch=build(mode,JSON.parse($('input').value));frames=batch.frames;index=0;$('result').textContent=JSON.stringify(batch.result);$('input-error').textContent='';render();}catch(e){$('input-error').textContent=e.message;}}
  function load(){stop();mode=Object.hasOwn(specs,location.hash.slice(1))?location.hash.slice(1):'cyclic_sequence';select.value=mode;$('title').textContent=specs[mode][0];$('intro').textContent=specs[mode][2];$('input').value=JSON.stringify(specs[mode][1],null,2);$('documentation').href=`../../docs/${paths[mode]}.md`;run();}
  select.addEventListener('change',()=>{location.hash=select.value;});window.addEventListener('hashchange',load);
  $('run').addEventListener('click',run);$('sample').addEventListener('click',load);
  $('first').addEventListener('click',()=>{stop();index=0;render();});$('finish').addEventListener('click',()=>{stop();index=frames.length-1;render();});
  $('next').addEventListener('click',()=>{stop();index=Math.min(frames.length-1,index+1);render();});$('previous').addEventListener('click',()=>{stop();index=Math.max(0,index-1);render();});
  $('play').addEventListener('click',()=>{if(timer){stop();return;}if(index===frames.length-1)index=0;$('play').textContent='停止';timer=setInterval(()=>{index++;render();if(index===frames.length-1)stop();},450);});
  load();
})();
