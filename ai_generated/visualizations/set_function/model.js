(function(root){
'use strict';
const MODES=['submask','supermask','sos','subset-zeta','subset-mobius','superset-zeta','superset-mobius'];
const norm=(x,p)=>p?(x%p+p)%p:x;
function popcount(x){let n=0;for(;x;x&=x-1n)n++;return n;}
function defaults(mode){return {mode,bits:'3',mask:mode==='supermask'?'2':'5',includeEmpty:true,values:mode==='subset-mobius'?'1 3 4 10 6 14 16 36':mode==='superset-mobius'?'36 20 22 12 26 14 15 8':'1 2 3 4 5 6 7 8',modulus:'integer',operation:'sum',focus:'5'};}
function scenario(mode,type){const c=defaults(mode);if(type==='zero'){c.mask='0';c.values='0 0 0 0 0 0 0 0';}if(type==='one'){c.bits='0';c.mask='0';c.values='7';c.focus='0';}if(type==='full')c.mask='7';if(type==='negative')c.values='-1 2 -3 4 -5 6 -7 8';if(type==='mod12')c.modulus='12';if(type==='mod1')c.modulus='1';if(type==='highest'){c.bits='64';c.mask=mode==='supermask'?'18446744073709551612':'9223372036854775809';}if(type==='no-empty'){c.includeEmpty=false;c.mask='5';}if(type==='overflow')c.values='1 9223372036854775807 0 0 0 0 0 0';if(type==='max')c.operation='max';if(type==='or')c.operation='or';return c;}
function parse(raw){
 const mode=MODES.includes(raw.mode)?raw.mode:'submask',enumeration=mode==='submask'||mode==='supermask';
 function integer(s,name,low,high){if(!/^-?\d+$/.test(String(s)))throw Error(`${name}は整数で指定します。`);const n=BigInt(s);if(n<low||n>high)throw Error(`${name}が範囲外です。`);return n;}
 const bits=Number(integer(raw.bits,'ビット数',0n,64n)),operation=['sum','min','max','or'].includes(raw.operation)?raw.operation:'sum';
 let mask=0n,values=[],p=0n;
 if(enumeration){
  if(!/^(?:\d+|0[bB][01]+)$/.test(String(raw.mask)))throw Error('maskは非負の整数または0b形式で指定します。');mask=BigInt(raw.mask);if(mask>=(1n<<64n))throw Error('maskがuint64の範囲外です。');if(mask>=(1n<<BigInt(bits)))throw Error('maskが指定したビット数に収まりません。');const free=mode==='submask'?popcount(mask):bits-popcount(mask);if(free>8)throw Error('図の列挙は自由なビット8個までです。');
 }else{
  if(bits>4)throw Error('図の集合変換は4ビットまでです。');const s=String(raw.values||'').trim();values=s?s.split(/[\s,]+/).map(x=>integer(x,'係数',-(1n<<63n),(1n<<63n)-1n)):[];if(values.length!==2**bits)throw Error(`係数列は2^${bits}=${2**bits}項が必要です。`);
  if(raw.modulus!=='integer'&&!(mode==='sos'&&operation!=='sum')){if(!['1','12','998244353'].includes(String(raw.modulus)))throw Error('法を選んでください。');p=BigInt(raw.modulus);values=values.map(x=>norm(x,p));}
 }
 const focus=enumeration?0:Number(integer(raw.focus??0,'注目する集合',0n,BigInt(values.length-1)));
 return {mode,enumeration,bits,mask,includeEmpty:raw.includeEmpty!==false,values,p,operation,focus};
}
function run(raw){
 const config=parse(raw),{mode,enumeration,bits,mask,p,operation}=config,frames=[];
 const emit=(title,description,extra={})=>frames.push(structuredClone({title,description,result:null,...extra}));
 if(enumeration){
  const full=(1n<<BigInt(bits))-1n,visited=[];let current=mask;
  emit('列挙する集合と全体集合','ビットiの1は要素iを含むことを表します。',{current,visited,full,stage:'input'});
  while(true){
   if(mode==='supermask'||current||config.includeEmpty)visited.push(current);
   emit(mode==='submask'&&!current&&!config.includeEmpty?'空集合を除外':'現在の集合を列挙',mode==='submask'?'maskの外のビットを立てず、数値の降順にたどります。':'maskの1を保ち、全体集合内の上位集合を数値の昇順にたどります。',{current,visited,full,stage:'visit'});
   if(mode==='submask'?!current:current===full)break;
   const intermediate=mode==='submask'?current-1n:current+1n,next=mode==='submask'?intermediate&mask:intermediate|mask;
   emit('次の集合へ進む',mode==='submask'?'1を引いてから元のmaskとANDを取り、余分な1を落とします。':'1を足してから元のmaskとORを取り、必須の1を戻します。',{current,intermediate,next,visited,full,stage:'next',formula:mode==='submask'?'next = (current-1) & mask':'next = (current+1) | mask'});current=next;
  }
  emit('列挙の完了','終端を確認してから終了するため、0やuint64の上限から折り返しません。',{current,visited,full,result:visited,stage:'done'});return {config,frames,result:[...visited]};
 }
 const input=[...config.values],f=[...input],subset=!mode.startsWith('superset'),inverse=mode.endsWith('mobius'),sos=mode==='sos',columns=[{label:'入力',values:[...f]}];let completed=0;
 emit('係数列と集合の対応','添字maskに、その集合の値を置きます。長さ1は0ビットの集合関数です。',{values:f,columns,completed,updated:[],stage:'input'});
 const order=Array.from({length:bits},(_,i)=>i);if(inverse)order.reverse();
 function merge(a,b){
  if(sos&&operation!=='sum')return operation==='min'?(a<b?a:b):operation==='max'?(a>b?a:b):a|b;
  const x=norm(inverse?a-b:a+b,p);if(!p&&(x<-(1n<<63n)||x>=(1n<<63n)))throw Error('途中の加減算がlong longの範囲を超えます。');return x;
 }
 for(const bit of order){
  const flag=1<<bit,updated=[];columns.push({label:`bit ${bit}`,values:[...f]});
  emit(`ビット${bit}の段階を開始`,inverse?'逆変換はビットを大きい順に処理して、集めた寄与を取り除きます。':'このビットを含む／含まない集合の値を合わせます。',{values:f,columns,completed,updated,bit,stage:'start'});
  for(let first=0;first<f.length;first+=2*flag)for(let offset=0;offset<flag;++offset){const low=first+offset,high=low+flag,target=subset?high:low,source=subset?low:high,before=f[target],read=f[source];f[target]=merge(before,read);updated.push(target);columns.at(-1).values=[...f];
   emit(`f[${target}]を更新`,subset?'このビットを持たない集合から、持つ集合へ寄与を伝えます。':'このビットを持つ上位集合から、持たない集合へ寄与を伝えます。',{values:f,columns,completed,updated,bit,target,source,before,read,after:f[target],stage:'update',formula:sos&&operation!=='sum'?`f[${target}] = ${operation}(f[${target}], f[${source}])`:`f[${target}] ${inverse?'-=':'+='} f[${source}]`});
  }
  completed|=flag;emit(`ビット${bit}の段階が完了`,'処理済みのビットだけを変えられる集合の寄与が集まっています。',{values:f,columns,completed,updated:[],bit,stage:'complete'});
 }
 emit('変換の完了',inverse?'交互和により、重複していた寄与を取り除きました。':'全ての対象集合を集約しました。',{values:f,columns,completed,updated:[],result:f,stage:'done'});return {config,frames,result:[...f]};
}
const api={MODES,defaults,scenario,parse,run};if(typeof module!=='undefined'&&module.exports)module.exports=api;else root.SetFunctionModel=api;
})(typeof globalThis!=='undefined'?globalThis:this);
