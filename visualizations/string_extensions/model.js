(function(root){
'use strict';
const modes=typeof module!=='undefined'?require('./modes.js'):root.StringExtensionModes;
const check=(ok,message)=>{if(!ok)throw Error(message);};
const integer=(x,l,r,name)=>{check(Number.isSafeInteger(x)&&x>=l&&x<=r,`${name}は${l}〜${r}の整数です`);return x;};
function bytes(value,max=48){const a=typeof value==='string'?[...new TextEncoder().encode(value)]:value;check(Array.isArray(a)&&a.length<=max&&a.every(x=>Number.isInteger(x)&&x>=0&&x<=255),`文字列はUTF-8、または0〜255のbyte配列です（上限${max}byte）`);return a.slice();}
const equal=(a,b)=>a.length===b.length&&a.every((x,i)=>x===b[i]);
const hex=x=>x.toString(16).padStart(2,'0'),label=x=>x>=32&&x<127?`${String.fromCharCode(x)}·${hex(x)}`:hex(x);
const key=a=>a.join(','),display=a=>a.every(x=>x>=33&&x<127)?String.fromCharCode(...a):'hex:'+a.map(hex).join('');
const compare=(a,b)=>{for(let i=0;i<Math.min(a.length,b.length);i++)if(a[i]!==b[i])return a[i]-b[i];return a.length-b.length;};
const occurrences=(s,p)=>Array.from({length:s.length+1},(_,i)=>i).filter(i=>equal(s.slice(i,i+p.length),p));
const zValues=s=>s.map((_,i)=>{let k=0;while(i+k<s.length&&s[k]===s[i+k])k++;return k;});
const prefix=s=>s.map((_,i)=>{for(let k=i;k>0;k--)if(equal(s.slice(0,k),s.slice(i+1-k,i+1)))return k;return 0;});
function allSubstrings(s){const values=new Map();for(let i=0;i<s.length;i++)for(let j=i+1;j<=s.length;j++){const p=s.slice(i,j);values.set(key(p),p);}return [...values.values()].sort(compare);}
function bwt(s){const text=s.map(x=>x+1).concat(0),rows=text.map((_,i)=>({p:i,a:text.slice(i).concat(text.slice(0,i))})).sort((x,y)=>compare(x.a,y.a));return {rows,last:rows.map(x=>x.a.at(-1)),primary:rows.findIndex(x=>x.p===0)};}
function sam(text,emit=()=>{}){const states=[{length:0,link:-1,next:new Map()}];let last=0;for(const c of text){const current=states.length;states.push({length:states[last].length+1,link:0,next:new Map()});let p=last;while(p>=0&&!states[p].next.has(c)){states[p].next.set(c,current);p=states[p].link;}if(p>=0){const q=states[p].next.get(c);if(states[q].length===states[p].length+1)states[current].link=q;else{const clone=states.length;states.push({length:states[p].length+1,link:states[q].link,next:new Map(states[q].next)});while(p>=0&&states[p].next.get(c)===q){states[p].next.set(c,clone);p=states[p].link;}states[q].link=states[current].link=clone;}}last=current;emit(states,c);}return states;}
function trace(id,input){
 check(input&&typeof input==='object'&&!Array.isArray(input),'入力はJSON objectです');check(modes.some(m=>m.id===id),'不明なモードです');
 const steps=[];let result=[],meta={};
 const step=(title,detail,columns,rows,extra={})=>{if(steps.length<600)steps.push({title,detail,columns,rows:rows.map(r=>r.map(String)),values:[{label:'行数',value:String(rows.length)}],...structuredClone(extra)});};
 const sequence=(title,detail,s,extra={})=>step(title,detail,['位置','byte (hex)'],s.map((c,i)=>[i,hex(c)]),{items:s.map(label),...extra});
 const automatonStep=(states,c)=>step('SAMへ記号を追加',c===256?'256は文書を区切る私有記号です。byte検索では通りません。':`追加byte: ${label(c)}`,['node','最大長','suffix link','遷移'],states.map((v,i)=>[i,v.length,v.link,[...v.next].map(([c,to])=>`${c===256?'#':hex(c)}→${to}`).join(' ')]),{items:states.map((v,i)=>`${i} / ${v.length}`),edges:states.slice(1).map((v,i)=>[i+1,v.link,'link'])});
 step('入力を確認','文字列はUTF-8のbyte列として扱います。0/FFも数値配列で入力できます。',['項目','値'],Object.entries(input).map(([k,v])=>[k,JSON.stringify(v)]));
 if(id==='subsequence'){
  const s=bytes(input.text),p=bytes(input.pattern);check(typeof input.modulus==='string'&&/^\d{1,20}$/.test(input.modulus),'法は10進文字列です');const m=BigInt(input.modulus);check(m>0n&&m<(1n<<64n),'法は正のuint64です');
  let total=1n%m;const previous=Array(256).fill(0n);for(let i=0;i<s.length;i++){const before=total;total=(2n*total-previous[s[i]]+m)%m;previous[s[i]]=before;sequence('重複する部分列を引く',`位置${i}までの相異なる部分列数（空を含む）: ${total}`,s.slice(0,i+1),{values:[{label:'個数 mod m',value:String(total)}]});}
  let at=0,positions=[];for(const c of p){const old=at;while(at<s.length&&s[at]!==c)at++;if(at===s.length){positions=null;break;}positions.push(at);sequence('次の出現へ進む',`探すbyte ${label(c)}、採用位置 ${at}`,s,{active:[at,at+1],edges:[[old,at,'next']]});at++;}
  result=[total,...(positions||['none'])];meta={positions};
 }else if(id==='lz77'){
  const s=bytes(input.text),order=Array.from({length:s.length},(_,i)=>i).sort((a,b)=>compare(s.slice(a),s.slice(b))),rank=Array(s.length);order.forEach((p,i)=>rank[p]=i);let i=0;const factors=[];
  while(i<s.length){const old=order.filter(p=>p<i),less=old.filter(p=>rank[p]<rank[i]).at(-1),greater=old.find(p=>rank[p]>rank[i]);let source=-1,length=0;
   for(const p of [less,greater])if(p!==undefined){let k=0;while(i+k<s.length&&s[p+k]===s[i+k])k++;if(k>length||(k===length&&k&&p<source)){source=p;length=k;}}
   const f=length?[source,length,0]:[-1,1,s[i]];factors.push(f);sequence('最長の既出因子を取る',source<0?'1byteをliteralとして保存します。':`source=${source}, length=${length}。重なる参照も許します。`,s,{active:[i,i+f[1]],edges:source<0?[]:[[source,i,'copy']]});i+=f[1];
  }
  const restored=[];for(const [p,n,c]of factors){if(p<0)restored.push(c);else for(let k=0;k<n;k++)restored.push(restored[p+k]);}result=[...factors.flat(),Number(equal(s,restored))];meta={factors};
 }else if(id==='bwt'||id==='fm_index'){
  const s=bytes(input.text,24),b=bwt(s);step('終端付きsuffixの辞書順','0は仮想終端。元byteは1〜256に写しています。',['row','開始位置','last',...s.map((_,i)=>i)],b.rows.map((row,i)=>[i,row.p,b.last[i],...row.a]),{matrix:b.rows.map(x=>x.a.map(c=>c?label(c-1):'$')),pivot:[b.primary,0]});
  const counts=Array(257).fill(0),C=Array(257).fill(0),ranks=b.last.map(c=>counts[c]++);for(let c=1;c<=256;c++)C[c]=C[c-1]+counts[c-1];
  if(id==='bwt'){let row=b.primary;const recovered=Array(s.length);for(let i=s.length;i-->0;){row=C[b.last[row]]+ranks[row];recovered[i]=b.last[row]-1;step('LF対応で1byte戻る',`row ${row} から出力位置 ${i} のbyteを復元します。`,['row','last','同文字の前の個数','LF'],b.last.map((c,j)=>[j,c,ranks[j],C[c]+ranks[j]]),{items:b.last.map(c=>c?label(c-1):'$'),active:[row,row+1]});}result=[b.primary,...b.last,Number(equal(s,recovered))];meta={bwt:b,recovered};}
  else{const p=bytes(input.pattern);integer(input.sample_rate??32,1,1000000,'sample_rate');let l=0,r=s.length+1;for(let i=p.length;i-->0&&l<r;){const c=p[i]+1;const rank=x=>b.last.slice(0,x).filter(y=>y===c).length;l=C[c]+rank(l);r=C[c]+rank(r);step('後ろから候補区間を狭める',`byte ${label(p[i])}、SA区間 [${l},${r})。図解のrankは直接数えます。`,['row','last','SA'],b.rows.map((row,j)=>[j,b.last[j],row.p]),{items:b.last.map(c=>c?label(c-1):'$'),active:[l,r]});}const found=b.rows.slice(l,r).map(x=>x.p).sort((a,b)=>a-b);result=[r-l,...found];meta={interval:[l,r],positions:found};}
 }else if(id==='semi_local'){
  const a=bytes(input.a,16),b=bytes(input.b,16),m=a.length,n=b.length,p=integer(input.prefix,0,m,'prefix'),l=integer(input.left,0,n,'left'),r=integer(input.right,l,n,'right');check(l<=m&&r<=m&&p<=n,'サンプル共通引数はA/B両方の範囲内にしてください');
  const bottom=Array.from({length:n},(_,j)=>j),out=Array(m+n),history=[bottom.slice()];let answer=r-l-bottom.slice(0,r).filter(x=>x>=l).length;
  for(let i=0;i<m;i++){let left=-i-1;for(let j=0;j<n;j++)if(a[i]===b[j]||left>bottom[j])[left,bottom[j]]=[bottom[j],left];out[n+m-i-1]=left;history.push(bottom.slice());if(i+1===p)answer=r-l-bottom.slice(0,r).filter(x=>x>=l).length;step('seaweedの1行を処理',`A[${i}]=${label(a[i])}。一致、または左label > 上labelなら交換します。`,['prefix',...b.map(label)],history.map((row,k)=>[k,...row]),{matrix:history,pivot:[i+1,0]});}bottom.forEach((x,i)=>out[i]=x);
  const count=(end,min)=>out.slice(0,end).filter(x=>x>=min).length;
  result=[answer,r-l-count(r,l),n-count(n+m-r,-l),n-l-count(n+m-p,l),p-count(p,-l)];meta={seaweed:out,prefix_answer:answer};
 }else if(id==='borders'||id==='online_z'){
  let s=bytes(input.text);const pi=prefix(s);
  for(let i=0;i<s.length;i++)sequence('prefixの失敗リンクを追加',`頂点${i+1}の親は${pi[i]}です。`,s.slice(0,i+1),{items:['0',...pi.slice(0,i+1).map((_,j)=>String(j+1))],edges:pi.slice(0,i+1).map((parent,j)=>[j+1,parent,'border'])});
  if(id==='borders'){const borders=[],periods=[];for(let k=1;k<s.length;k++)if(equal(s.slice(0,k),s.slice(s.length-k)))borders.push(k);for(let p=1;p<=s.length;p++)if(equal(s.slice(0,s.length-p),s.slice(p)))periods.push(p);const samplePrefix=Math.min(2,s.length);result=[...borders,...periods,occurrences(s,s.slice(0,samplePrefix)).length];meta={borders,periods};}
  else{const position=integer(input.position,0,s.length,'position'),before=position===s.length?0:zValues(s)[position],more=bytes(input.append);check(s.length+more.length<=48,'合計48byte以下です');for(const c of more){s.push(c);const z=zValues(s);step('追記後のZ値',`byte ${label(c)} を追加。以前の位置のZ値も伸びることがあります。`,['位置','byte','Z'],s.map((c,i)=>[i,hex(c),z[i]]),{items:s.map((c,i)=>`${label(c)} / ${z[i]}`),active:[position,position+(z[position]||0)]});}const z=zValues(s);result=[before,z[position]||0,...z];meta={z};}
 }else if(id==='sam_utilities'||id==='generalized_sam'){
  const documents=id==='sam_utilities'?[bytes(input.text,24)]:(check(Array.isArray(input.documents)&&input.documents.length<=8,'文書は8個以下です'),input.documents.map(x=>bytes(x,24)));check(documents.reduce((n,s)=>n+s.length,0)<=48,'文書合計48byte以下です');
  const text=id==='sam_utilities'?documents[0]:documents.flatMap(x=>x.concat(256)),states=sam(text,automatonStep),p=bytes(input.pattern),all=new Map();for(const s of documents)for(const w of allSubstrings(s))all.set(key(w),w);const distinct=[...all.values()].sort(compare),found=documents.flatMap((s,d)=>occurrences(s,p).map(i=>[d,i]));
  if(id==='sam_utilities'){const k=integer(input.k,0,1000000,'k');result=[k<distinct.length?display(distinct[k]):'none',...found.map(x=>x[1])];step('経路を辞書順に数える','空部分文字列を除き、0始まりのk番目を選びます。',['k','byte列'],distinct.map((s,i)=>[i,display(s)]),{items:distinct.slice(0,24).map(display),active:[k,k+1]});}
  else{const ids=[...new Set(found.map(x=>x[0]))];result=[found.length,distinct.length,...ids];step('文書境界を越えない出現だけを返す','separatorを通る経路はbyte検索や相異なる部分文字列数に含めません。',['文書ID','開始位置'],found,{items:found.map(([d,p])=>`${d}:${p}`)});}
  meta={distinct,positions:found,state_count:states.length};
 }else if(id==='deque_eertree'){
  check(Array.isArray(input.operations)&&input.operations.length<=80,'操作は80個以下です');let s=[],stats=[0,0,0,0];
  for(const op of input.operations){check(Array.isArray(op)&&['push_front','push_back','pop_front','pop_back'].includes(op[0]),'操作はpush/popのfront/backです');if(op[0].startsWith('push')){const a=typeof op[1]==='number'?[op[1]]:bytes(op[1],1);check(a.length===1&&Number.isInteger(a[0])&&a[0]>=0&&a[0]<=255,'追加は1byteです');op[0]==='push_front'?s.unshift(a[0]):s.push(a[0]);}else{check(s.length,'空dequeから削除できません');op[0]==='pop_front'?s.shift():s.pop();}check(s.length<=48,'dequeは48byte以下です');
   const pal=new Set(),intervals=[];stats=[0,0,0,0];for(let i=0;i<s.length;i++)for(let j=i+1;j<=s.length;j++){const p=s.slice(i,j);if(equal(p,p.slice().reverse())){pal.add(key(p));intervals.push([i,j,display(p)]);stats[1]++;if(i===0)stats[2]=Math.max(stats[2],j);if(j===s.length)stats[3]=Math.max(stats[3],j-i);}}stats[0]=pal.size;
   sequence(op[0]+' 後の回文',`異なる回文${stats[0]}、出現総数${stats[1]}。図解は全区間比較、C++はsurfaceを更新します。`,s,{active:[0,stats[2]],edges:intervals.filter(([l,r])=>r-l>1).map(([l,r])=>[l,r-1,'pal']),values:stats.map((v,i)=>({label:['種類数','総出現','最長prefix','最長suffix'][i],value:String(v)}))});
  }result=stats;meta={text:s};
 }else if(id==='period_queries'){
  const s=bytes(input.text),l=integer(input.left,0,s.length,'left'),r=integer(input.right,l,s.length,'right'),p=integer(input.period,1,r-l,'period'),isPeriod=k=>equal(s.slice(l,r-k),s.slice(l+k,r));let root=r-l;
  sequence('ずらした区間を比較',`period ${p}: ${isPeriod(p)}。長さを割り切らないperiodも判定できます。`,s,{active:[l,r],edges:[[l,l+p,'period']]});
  let remain=root;for(let f=2;f<=remain;f++)if(remain%f===0){while(remain%f===0)remain/=f;while(root%f===0&&isPeriod(root/f)){root/=f;sequence('反復単位を短くする',`素因数 ${f} で割った周期 ${root} でも一致します。`,s,{active:[l,l+root],edges:[[l,l+root,'repeat']]});}}
  result=[Number(isPeriod(p)),root,root?(r-l)/root:0];meta={root};
 }
 result=result.map(String);step('結果','API・使用例の出力と対応しています。',['順','値'],result.map((x,i)=>[i,x]),{items:result});return {steps,result,meta};
}
const api={modes,trace};if(typeof module!=='undefined')module.exports=api;else root.StringExtensionsModel=api;
})(typeof window!=='undefined'?window:globalThis);
