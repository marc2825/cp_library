(function (root) {
  'use strict';
  const P = (1n << 61n) - 1n, R = 1n << 64n;
  const modes = [
    {id:'static_modint',title:'固定法modint',sample:{mod:'12',a:'-5',b:'5'}},
    {id:'dynamic_modint',title:'実行時法と世代',sample:{mod:'12',value:'5',nextMod:'11'}},
    {id:'barrett_reduction',title:'Barrett Reduction',sample:{mod:'7',value:'18446744073709551615'}},
    {id:'montgomery_modint',title:'Montgomery Reduction',sample:{mod:'101',a:'12',b:'13'}},
    {id:'modint61',title:'2^61−1の積',sample:{a:'2305843009213693950',b:'2305843009213693950'}},
    {id:'fast_io',title:'バッファと整数入力',sample:{text:' -128 127 128 +7',bits:8,signed:true,buffer:3}},
    {id:'safe_integer',title:'検査・飽和・丸め',sample:{a:'127',b:'3',bits:8,signed:true}}
  ];
  function integer(x) {
    if (typeof x !== 'string' || !/^[+-]?\d{1,50}$/.test(x)) throw Error('整数は50桁以内の10進文字列で指定してください。');
    const value=BigInt(x);
    if(value<-(1n<<127n)||value>=(1n<<128n))throw Error('入力整数は符号付き／なし128bitの範囲です。');
    return value;
  }
  function modulus(x,limit) { const m=integer(x); if(m<1n||m>=limit)throw Error('法が許容範囲外です。');return m; }
  function normalized(x,m) { return (x%m+m)%m; }
  function inverse(a,m) {
    let r=m,s=normalized(a,m),x=0n,y=1n;
    while(s){const q=r/s;[r,s]=[s,r-q*s];[x,y]=[y,x-q*y];}
    return r===1n?normalized(x,m):null;
  }
  function bounds(input) {
    const bits=input.bits;
    if(![8,16,32,64,128].includes(bits)||typeof input.signed!=='boolean')throw Error('bitsは8/16/32/64/128、signedは真偽値です。');
    return input.signed?[-(1n<<BigInt(bits-1)),(1n<<BigInt(bits-1))-1n]:[0n,(1n<<BigInt(bits))-1n];
  }
  function trace(mode,input) {
    if(!input||typeof input!=='object'||Array.isArray(input))throw Error('入力はJSONオブジェクトで指定してください。');
    const steps=[];
    const step=(title,detail,values)=>steps.push({title,detail,values:Object.entries(values).map(([label,value])=>({label,value:String(value)}))});
    let result;
    if(mode==='static_modint'){
      const m=modulus(input.mod,1n<<32n),a=integer(input.a),b=integer(input.b),x=normalized(a,m),y=normalized(b,m);
      step('入力と法','負数も同じ剰余類に正規化します。',{a,b,m});
      step('代表元をそろえる','代表元は0以上m未満です。',{x,y});
      step('乗算する','積を広い整数で作り、法で割った余りを取ります。',{product:x*y,remainder:x*y%m});
      const inv=inverse(y,m);
      result={a:String(x),b:String(y),sum:String((x+y)%m),product:String(x*y%m),inverse:inv===null?'none':String(inv)};
      step('逆元の存在','gcd(b,m)=1のときだけ除算できます。法1では逆元も0です。',{inverse:result.inverse});
    }else if(mode==='dynamic_modint'){
      const m=modulus(input.mod,1n<<32n),next=modulus(input.nextMod,1n<<32n),v=normalized(integer(input.value),m);
      step('法を設定','IDごとに法と世代を共有します。',{mod:m,generation:1});
      step('値を構築','値にも現在の世代を記録します。',{value:v,valueGeneration:1});
      step('法を再設定','同じ法への再設定も世代を進めます。',{mod:next,generation:2,valueGeneration:1});
      step('古い値の使用を検出','世代が一致しないためlogic_error。新しい法で構築し直します。',{currentGeneration:2,valueGeneration:1});
      result={oldValue:String(v),status:'stale',newValue:String(normalized(integer(input.value),next))};
    }else if(mode==='barrett_reduction'){
      const m=modulus(input.mod,1n<<32n),x=integer(input.value);
      if(x<0n||x>=R)throw Error('valueはuint64の範囲です。');
      if(m===1n){step('法1','すべての剰余は0です。',{value:x,remainder:0});result='0';}
      else{
        const reciprocal=(R+m-1n)/m,q=x*reciprocal/R,product=q*m;
        step('逆数を前計算','ceil(2^64/m)を保持します。',{m,reciprocal});
        step('積の上位64bit','真の商か、それより1大きい商が得られます。',{x,q,product});
        step('1回だけ補正','q*mがxより大きいときだけ法を足します。',{difference:x-product,correction:product>x?m:0n,remainder:product>x?x+m-product:x-product});
        result=String(product>x?x+m-product:x-product);
      }
    }else if(mode==='montgomery_modint'){
      const m=modulus(input.mod,1n<<63n);
      if(!(m&1n))throw Error('Montgomery法は奇数です。');
      const a=normalized(integer(input.a),m),b=normalized(integer(input.b),m),x=a*R%m,y=b*R%m;
      const neg=normalized(-inverse(m,R),R),t=x*y,q=t%R*neg%R,u=(t+q*m)/R,z=u>=m?u-m:u;
      step('Montgomery表現へ','各値にR=2^64を掛けた剰余を保持します。',{a,b,encodedA:x,encodedB:y});
      step('表現同士の積','積はRを2つ含むので、REDCで1つ取り除きます。',{product:t,negativeInverse:neg});
      step('下位64bitを打ち消す','q*mを足すと下位64bitが0になり、右シフトできます。',{q,sum:t+q*m,divided:u});
      step('法で補正','内部値は常に0以上m未満です。',{encodedProduct:z});
      result=String(z*inverse(R,m)%m);
      step('通常表現に戻す','もう一度Rを取り除き、a*b mod mを得ます。',{result});
    }else if(mode==='modint61'){
      const a=normalized(integer(input.a),P),b=normalized(integer(input.b),P),product=a*b,low=product&P,high=product>>61n,folded=low+high;
      step('積を計算','中間積は128bitに収まります。',{a,b,product});
      step('61bitで分割','2^61 ≡ 1なので高位と低位を足せます。',{low,high});
      step('折り畳んで補正','和は2p未満。p以上ならpを1回引きます。',{folded,mod:P,correction:folded>=P?P:0n});
      result=String(folded>=P?folded-P:folded);
    }else if(mode==='fast_io'){
      const [lo,hi]=bounds(input),size=input.buffer,text=input.text;
      if(!Number.isInteger(size)||size<1||size>16||typeof text!=='string'||text.length>120||/[^\x00-\x7f]/.test(text))throw Error('textは120文字以内のASCII、bufferは1〜16です。');
      result=[];
      for(let start=0;start<text.length;){
        if(text.charCodeAt(start)<=32){start++;continue;}
        let end=start;while(end<text.length&&text.charCodeAt(end)>32)end++;
        const token=text.slice(start,end),negative=token.startsWith('-');
        const valid=/^[+-]?\d+$/.test(token)&&(!negative||input.signed);
        let mag=0n;
        for(let at=start;at<end;at++){
          const c=text[at];if(c>='0'&&c<='9')mag=mag*10n+BigInt(c);
          step('バイトを読む','バッファ境界をまたいでも整数の状態を保持します。',{buffer:text.slice(Math.floor(at/size)*size,Math.floor(at/size)*size+size),offset:at,byte:JSON.stringify(c),magnitude:mag});
        }
        const value=negative?-mag:mag,status=!valid?'invalid':value<lo||value>hi?'overflow':String(value);
        result.push(status);step('トークンを確定','不正なトークンを消費しても対象変数は更新しません。',{token,result:status});start=end;
      }
      step('EOF','末尾改行がなくても読み取りを終了します。',{read:'false'});
    }else if(mode==='safe_integer'){
      const [lo,hi]=bounds(input),a=integer(input.a),b=integer(input.b);
      if(a<lo||a>hi||b<lo||b>hi)throw Error('a,bは指定した型の範囲に収めてください。');
      const sum=a+b,product=a*b,clamp=x=>x<lo?lo:x>hi?hi:x;
      step('型の範囲','演算結果を型の上下限と照合します。',{a,b,minimum:lo,maximum:hi});
      step('数学的な結果','C++は検査付きbuiltinで、溢れる演算を安全に検出します。',{sum,product});
      result={sum:sum<lo||sum>hi?'none':String(sum),saturatedSum:String(clamp(sum)),product:product<lo||product>hi?'none':String(product),saturatedProduct:String(clamp(product))};
      step('検査と飽和','checkedは失敗、saturatingは端点を返します。',result);
      if(b===0n){result.floor=result.ceil='invalid';}
      else if(input.signed&&a===lo&&b===-1n){result.floor=result.ceil='overflow';}
      else {const q=a/b,r=a%b,negative=(a<0n)!==(b<0n);result.floor=String(q-(r!==0n&&negative?1n:0n));result.ceil=String(q+(r!==0n&&!negative?1n:0n));}
      step('床・天井へ丸める','0方向への商を、余りと符号を使って補正します。',{floor:result.floor,ceil:result.ceil});
    }else throw Error('未知のモードです。');
    step('最終結果','入力を変えて、境界値での動作を確認できます。',{result:JSON.stringify(result)});
    return {steps,result};
  }
  const api={modes,trace};
  if(typeof module!=='undefined'&&module.exports)module.exports=api;
  else root.FoundationModel=api;
})(typeof globalThis!=='undefined'?globalThis:this);
