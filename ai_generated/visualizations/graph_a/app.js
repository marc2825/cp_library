(()=>{
  'use strict';
  const {specs,build}=window.GraphAModels,$=id=>document.getElementById(id),select=$('mode');
  const {paths}=window.GraphAModels;
  for(const [key,[title]]of Object.entries(specs)){const option=document.createElement('option');option.value=key;option.textContent=title;select.append(option);}
  let mode,frames=[],index=0,timer=null;
  function stop(){clearInterval(timer);timer=null;$('play').textContent='再生';}
  function svg(tag,attributes,text){const node=document.createElementNS('http://www.w3.org/2000/svg',tag);for(const [key,value]of Object.entries(attributes))node.setAttribute(key,value);if(text!==undefined)node.textContent=String(text);$('diagram').append(node);return node;}
  function draw(frame){
    $('diagram').replaceChildren();const graph=frame.graph;if(!graph){svg('text',{x:25,y:55,'font-size':14,fill:'#61736f'},'下の表で候補と内部状態を確認できます。');$('diagram').setAttribute('viewBox','0 0 760 130');return;}
    const n=graph.n,height=n>10?400:320,positions=Array.from({length:n},(_,i)=>({x:380+265*Math.cos(2*Math.PI*i/Math.max(1,n)-Math.PI/2),y:height/2+115*Math.sin(2*Math.PI*i/Math.max(1,n)-Math.PI/2)}));
    const defs=svg('defs',{}),marker=document.createElementNS('http://www.w3.org/2000/svg','marker');for(const[k,v]of Object.entries({id:'arrow',viewBox:'0 0 10 10',refX:9,refY:5,markerWidth:5,markerHeight:5,orient:'auto-start-reverse'}))marker.setAttribute(k,v);const tip=document.createElementNS('http://www.w3.org/2000/svg','path');tip.setAttribute('d','M 0 0 L 10 5 L 0 10 z');tip.setAttribute('fill','#61736f');marker.append(tip);defs.append(marker);
    graph.edges.forEach((e,id)=>{const[u,v,w]=e,a=positions[u],b=positions[v];if(!a||!b)return;const selected=frame.highlightEdges?.includes(id),color=selected?'#bd6935':'#a1b3a5';
      if(u===v){svg('path',{d:`M ${a.x-13} ${a.y-18} C ${a.x-55} ${a.y-75},${a.x+55} ${a.y-75},${a.x+13} ${a.y-18}`,fill:'none',stroke:color,'stroke-width':selected?3:1.8,...(graph.directed?{'marker-end':'url(#arrow)'}:{})});if(w!==undefined)svg('text',{x:a.x,y:a.y-58,'text-anchor':'middle','font-size':11,fill:'#203332'},w);return;}
      const dx=b.x-a.x,dy=b.y-a.y,len=Math.hypot(dx,dy),x1=a.x+dx/len*23,y1=a.y+dy/len*23,x2=b.x-dx/len*26,y2=b.y-dy/len*26,offset=(id%3-1)*13,cx=(x1+x2)/2-dy/len*offset,cy=(y1+y2)/2+dx/len*offset;
      svg('path',{d:`M ${x1} ${y1} Q ${cx} ${cy} ${x2} ${y2}`,fill:'none',stroke:color,'stroke-width':selected?3:1.6,...(graph.directed?{'marker-end':'url(#arrow)'}:{})});if(w!==undefined)svg('text',{x:cx,y:cy-5,'text-anchor':'middle','font-size':11,fill:'#203332'},w);
    });
    positions.forEach((p,v)=>{const active=frame.activeVertices?.includes(v);svg('circle',{cx:p.x,cy:p.y,r:21,fill:active?'#e9c5a9':'#eaf3ed',stroke:active?'#bd6935':'#1b7361'});svg('text',{x:p.x,y:p.y+5,'text-anchor':'middle','font-size':13,fill:'#203332'},graph.labels?.[v]??v);});$('diagram').setAttribute('viewBox',`0 0 760 ${height}`);
  }
  function render(){if(!frames.length)return;const f=frames[index];$('step-title').textContent=f.title;$('step-note').textContent=f.note;$('step-count').textContent=`${index+1} / ${frames.length}`;const tr=document.createElement('tr');for(const c of f.columns){const th=document.createElement('th');th.textContent=String(c);tr.append(th);}$('columns').replaceChildren(tr);$('rows').replaceChildren();for(const row of f.rows){const tr=document.createElement('tr');for(const c of row){const td=document.createElement('td');td.textContent=String(c);tr.append(td);}$('rows').append(tr);}draw(f);$('first').disabled=$('previous').disabled=index===0;$('finish').disabled=$('next').disabled=index===frames.length-1;$('play').disabled=frames.length<=1;}
  function run(){stop();try{const batch=build(mode,JSON.parse($('input').value));frames=batch.frames;index=0;$('result').textContent=JSON.stringify(batch.result);$('input-error').textContent='';render();}catch(e){$('input-error').textContent=e.message;}}
  function load(){stop();mode=Object.hasOwn(specs,location.hash.slice(1))?location.hash.slice(1):'radix_heap_dijkstra';select.value=mode;$('title').textContent=specs[mode][0];$('intro').textContent=specs[mode][2];$('input').value=JSON.stringify(specs[mode][1],null,2);$('documentation').href=`../../docs/${paths[mode]}.md`;run();}
  select.addEventListener('change',()=>{location.hash=select.value;});window.addEventListener('hashchange',load);
  $('run').addEventListener('click',run);$('sample').addEventListener('click',load);
  $('first').addEventListener('click',()=>{stop();index=0;render();});$('finish').addEventListener('click',()=>{stop();index=frames.length-1;render();});
  $('next').addEventListener('click',()=>{stop();index=Math.min(frames.length-1,index+1);render();});$('previous').addEventListener('click',()=>{stop();index=Math.max(0,index-1);render();});
  $('play').addEventListener('click',()=>{if(timer){stop();return;}if(index===frames.length-1)index=0;$('play').textContent='停止';timer=setInterval(()=>{index++;render();if(index===frames.length-1)stop();},450);});
  load();
})();
