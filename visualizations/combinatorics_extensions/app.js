(function () {
  'use strict';
  const model=window.CombinatoricsExtensionsModel,$=id=>document.getElementById(id);
  let state=null,index=0,timer=null;
  for(const mode of model.modes){const option=document.createElement('option');option.value=mode.id;option.textContent=mode.title;$('mode').append(option);}
  function stop(){if(timer!==null)clearInterval(timer);timer=null;$('play').textContent='再生';}
  function render(){
    if(!state)return;
    const step=state.steps[index];$('step-count').textContent=`${index+1} / ${state.steps.length}`;
    $('progress').max=state.steps.length-1;$('progress').value=index;
    $('step-title').textContent=step.title;$('detail').textContent=step.detail;$('values').replaceChildren();
    for(const value of step.values){
      const card=document.createElement('div'),label=document.createElement('span'),number=document.createElement('span');
      card.className='value-card';label.className='value-label';number.className='value-number';
      label.textContent=value.label;number.textContent=value.value;card.append(label,number);$('values').append(card);
    }
    if(step.plot)drawPlot(step.plot);else drawGraph(step.state);
    $('diagram').replaceChildren();
    if(step.columns.length){
      const table=document.createElement('table'),head=document.createElement('tr');
      for(const value of step.columns){const cell=document.createElement('th');cell.textContent=String(value);head.append(cell);}table.append(head);
      for(const row of step.rows){const tr=document.createElement('tr');for(const value of row){const cell=document.createElement('td');cell.textContent=String(value);tr.append(cell);}table.append(tr);}
      $('diagram').append(table);
    }
    $('result').textContent=JSON.stringify(state.result,null,2);
    $('first').disabled=$('previous').disabled=index===0;$('next').disabled=$('finish').disabled=index===state.steps.length-1;
  }
  function drawPlot(points){
    const box=$('forest');box.replaceChildren();
    const ns='http://www.w3.org/2000/svg',svg=document.createElementNS(ns,'svg');
    svg.setAttribute('viewBox','0 0 560 300');svg.setAttribute('role','img');svg.setAttribute('aria-label','整数点とその値');
    const low=Math.min(...points.map(p=>p[1])),high=Math.max(...points.map(p=>p[1]));
    const x=v=>45+(v-points[0][0])*480/(points.at(-1)[0]-points[0][0]||1),y=v=>250-(v-low)*210/(high-low||1);
    const element=(tag,attrs,text)=>{const e=document.createElementNS(ns,tag);for(const [k,v]of Object.entries(attrs))e.setAttribute(k,v);if(text!==undefined)e.textContent=text;svg.append(e);return e;};
    element('line',{x1:45,y1:250,x2:525,y2:250,stroke:'#aebdce'});
    element('polyline',{points:points.map(p=>`${x(p[0])},${y(p[1])}`).join(' '),fill:'none',stroke:'#276ab0','stroke-width':3});
    for(const p of points)element('circle',{cx:x(p[0]),cy:y(p[1]),r:2.5,fill:'#276ab0'});
    element('text',{x:45,y:280,'text-anchor':'middle',fill:'#253e55'},points[0][0]);element('text',{x:525,y:280,'text-anchor':'middle',fill:'#253e55'},points.at(-1)[0]);
    element('text',{x:15,y:35,fill:'#253e55'},high);element('text',{x:15,y:250,fill:'#253e55'},low);box.append(svg);
  }
  function drawGraph(state){
    const box=$('forest');box.replaceChildren();if(!state)return;
    const ns='http://www.w3.org/2000/svg',svg=document.createElementNS(ns,'svg');
    const grids=state.grids||[],width=Math.max(1,...grids.map(g=>Math.max(1,...g.cells.map(r=>r.length))))*48+32;
    const height=grids.reduce((s,g)=>s+g.cells.length*40+48,12);
    svg.setAttribute('viewBox',`0 0 ${width} ${height}`);svg.setAttribute('role','img');svg.setAttribute('aria-label','計算途中のセル');
    const el=(tag,attrs,text)=>{const e=document.createElementNS(ns,tag);for(const[k,v]of Object.entries(attrs))e.setAttribute(k,v);if(text!==undefined)e.textContent=text;svg.append(e);};
    let y=28;for(const grid of grids){el('text',{x:12,y,fill:'#253e55','font-size':14},grid.title);y+=10;
      grid.cells.forEach((row,r)=>row.forEach((value,c)=>{const x=12+c*48,cy=y+r*40;
        el('rect',{x,y:cy,width:46,height:38,rx:4,fill:value===null?'#e2e8ee':grid.highlight?.[0]===r&&grid.highlight?.[1]===c?'#ffe6a6':'#e3f0ff',stroke:'#7b9ab8'});
        el('text',{x:x+23,y:cy+24,'text-anchor':'middle','font-size':13,fill:'#163a5f'},value===null?'×':String(value));
      }));y+=grid.cells.length*40+38;
    }box.append(svg);
  }
  function run(){stop();try{state=model.trace($('mode').value,JSON.parse($('input').value));index=0;$('input-error').textContent='';render();}
    catch(error){$('input-error').textContent=error.message;}}
  function select(){
    stop();const selected=model.modes.find(x=>x.id===$('mode').value);
    $('title').textContent=selected.title;$('doc-link').href=`../../docs/${selected.doc}`;
    $('input').value=JSON.stringify(selected.sample,null,2);run();
  }
  $('mode').addEventListener('change',()=>{history.replaceState(null,'',`#${$('mode').value}`);select();});
  $('run').addEventListener('click',run);$('sample').addEventListener('click',select);
  for(const [id,move] of [['first',()=>0],['previous',()=>Math.max(0,index-1)],['next',()=>Math.min(state.steps.length-1,index+1)],['finish',()=>state.steps.length-1]]){
    $(id).addEventListener('click',()=>{stop();if(state){index=move();render();}});
  }
  $('play').addEventListener('click',()=>{
    if(timer!==null){stop();return;}if(!state)return;
    if(index===state.steps.length-1)index=0;render();$('play').textContent='停止';
    timer=setInterval(()=>{index=Math.min(index+1,state.steps.length-1);render();if(index===state.steps.length-1)stop();},650);
  });
  function fromHash(){const id=location.hash.slice(1);$('mode').value=model.modes.some(x=>x.id===id)?id:model.modes[0].id;select();}
  window.addEventListener('hashchange',fromHash);fromHash();
})();
