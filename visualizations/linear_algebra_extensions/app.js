(function () {
  'use strict';
  const model=window.LinearAlgebraExtensionsModel,$=id=>document.getElementById(id);
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
    drawState(step);
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
  function drawState(step){
    const box=$('forest');box.replaceChildren();
    const ns='http://www.w3.org/2000/svg',svg=document.createElementNS(ns,'svg');
    const rows=step.matrix||step.rows.map(row=>[row[1]]),cols=Math.max(1,...rows.map(row=>row.length));
    const width=Math.max(360,cols*112+52),height=Math.max(100,rows.length*64+40);
    svg.setAttribute('viewBox',`0 0 ${width} ${height}`);svg.setAttribute('role','img');svg.setAttribute('aria-label','行列・係数表の途中状態');
    const el=(tag,attrs,text)=>{const e=document.createElementNS(ns,tag);for(const[k,v]of Object.entries(attrs))e.setAttribute(k,v);if(text!==undefined)e.textContent=text;svg.append(e);};
    for(let j=0;j<cols;j++)el('text',{x:106+j*112,y:19,'text-anchor':'middle','font-size':12,fill:'#527070'},j);
    rows.forEach((row,i)=>{el('text',{x:20,y:66+i*64,'text-anchor':'middle','font-size':12,fill:'#527070'},i);row.forEach((value,j)=>{
      const x=52+j*112,y=30+i*64,active=step.pivot&&step.pivot[0]===i&&step.pivot[1]===j;
      el('rect',{x,y,width:106,height:54,rx:7,fill:active?'#ffe6a6':i===j?'#e0f2eb':'#e3f0ff',stroke:active?'#bc8315':'#8baab0'});
      const text=String(value);el('text',{x:x+53,y:y+33,'text-anchor':'middle','font-size':text.length>10?10:15,fill:'#163a5f'},text.length>15?text.slice(0,12)+'…':text);
    });});box.append(svg);
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
