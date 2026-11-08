(function(root){
  'use strict';
  const modes=[
    {id:'parse',title:'10進・16進の読み取り',doc:'basic_utility/big_integer.md',sample:{text:'-0x123456789ABCDEF',base:16}},
    {id:'arithmetic',title:'桁の加算・減算・乗算',doc:'basic_utility/big_integer.md',sample:{a:'999999999',b:'123456',op:'*'}},
    {id:'division',title:'商と余りの符号',doc:'basic_utility/big_integer.md',sample:{a:'-12345678901234567890',b:'97'}},
    {id:'gcd',title:'最大公約数を求める',doc:'basic_utility/big_integer.md',sample:{a:'12345678901234567890',b:'9876543210'}}
  ];
  const abs=x=>x<0n?-x:x,sign=x=>x<0n?-1n:1n;
  function decimal(text){if(typeof text!=='string'||!/^[-+]?\d{1,36}$/.test(text))throw Error('整数は36桁以内の10進文字列です。');return BigInt(text);}
  function trace(mode,input){
    if(!input||typeof input!=='object'||Array.isArray(input))throw Error('JSONオブジェクトを指定してください。');
    const steps=[];let result;
    const step=(title,detail,values,limbs)=>steps.push({title,detail,values:Object.entries(values).map(([label,value])=>({label,value:String(value)})),...(limbs?{limbs:[...limbs]}:{})});
    if(mode==='parse'){
      const {text,base}=input;
      if((base!==10&&base!==16)||typeof text!=='string'||text.length>39)throw Error('baseは10または16、入力は36桁までです。');
      let digits=text,negative=false;
      if(digits[0]==='-'||digits[0]==='+'){negative=digits[0]==='-';digits=digits.slice(1);}
      if(base===16&&/^0[xX]/.test(digits))digits=digits.slice(2);
      if(!(base===10?/^\d{1,36}$/:/^[0-9a-fA-F]{1,36}$/).test(digits))throw Error('指定した基数では読めない文字、または空の数です。');
      const width=base===10?9:8;let value=0n;
      step('符号と基数を分ける','10進では先頭の0も10進の桁として読みます。',{negative,base,digits});
      for(let i=0;i<digits.length;i+=width){
        const chunk=digits.slice(i,i+width),factor=BigInt(base)**BigInt(chunk.length),part=BigInt(base===16?'0x'+chunk:chunk),before=value;
        value=value*factor+part;step('桁のまとまりを取り込む','value = before × factor + chunk。C++と同じ最大9桁／8桁の区切りです。',{before,factor,chunk,part,value});
      }
      if(negative)value=-value;
      result={decimal:String(value),hex:(value<0n?'-':'')+abs(value).toString(16)};
    }else if(mode==='arithmetic'){
      const a=decimal(input.a),b=decimal(input.b),op=input.op;
      if(!['+','-','*'].includes(op))throw Error('opは+、-、*です。');
      const split=x=>{const digits=abs(x).toString(),out=[];for(let i=digits.length;i>0;i-=3)out.push(Number(digits.slice(Math.max(0,i-3),i)));return out;};
      let left=split(a),right=split(b),out=[],polarity=1n;
      step('基数1000に分ける','図では3桁を1セルとし、配列は下位桁から保持します。',{a,b,op,left:left.join(','),right:right.join(',')});
      if(op==='*'){
        out=Array(left.length+right.length).fill(0);polarity=sign(a)*sign(b);
        for(let i=0;i<left.length;i++){
          let carry=0;
          for(let j=0;j<right.length;j++){
            const position=i+j,total=out[position]+left[i]*right[j]+carry;
            out[position]=total%1000;carry=Math.floor(total/1000);
            step('部分積と繰り上がり','現在の桁にa[i]×b[j]とcarryを加えます。',{i,j,position,total,digit:out[position],carry},out);
          }
          out[i+right.length]=carry;step('繰り上がりを置く','次の上位桁へcarryを置きます。',{position:i+right.length,carry},out);
        }
      }else{
        const effective=op==='-'?-b:b;
        if(sign(a)===sign(effective)){
          let carry=0;polarity=sign(a);
          for(let i=0;i<Math.max(left.length,right.length);i++){
            const total=(left[i]||0)+(right[i]||0)+carry;out.push(total%1000);carry=Math.floor(total/1000);
            step('各桁を加える','1000以上なら次の桁へ1を繰り上げます。',{i,total,digit:out[i],carry},out);
          }
          if(carry)out.push(carry);
        }else{
          polarity=abs(a)>=abs(effective)?sign(a):sign(effective);
          if(abs(a)<abs(effective))[left,right]=[right,left];
          let borrow=0;
          for(let i=0;i<left.length;i++){
            const difference=left[i]-(right[i]||0)-borrow;borrow=difference<0?1:0;out.push(difference+1000*borrow);
            step('各桁を引く','負になった桁は上位から1000を借ります。',{i,difference,digit:out[i],borrow},out);
          }
        }
      }
      while(out.length>1&&out.at(-1)===0)out.pop();
      let magnitude=0n;for(let i=out.length;i-->0;)magnitude=magnitude*1000n+BigInt(out[i]);
      result={value:String(polarity*magnitude)};
      step('符号を戻す','不要な上位の0を除き、ゼロの符号を正規化します。',result,out);
    }else if(mode==='division'){
      const a=decimal(input.a),b=decimal(input.b);if(b===0n)throw Error('0では割れません。');
      const divisor=abs(b);let remainder=0n,quotient=0n;
      for(const digit of abs(a).toString()){
        const partial=remainder*10n+BigInt(digit),q=partial/divisor;
        remainder=partial-q*divisor;quotient=quotient*10n+q;
        step('次の桁を下ろす','部分被除数から除数×商の1桁を引き、余りを残します。',{digit,partial,divisor,quotientDigit:q,quotient,remainder});
      }
      quotient*=sign(a)*sign(b);remainder*=sign(a);
      result={quotient:String(quotient),remainder:String(remainder)};
      step('商と余りに符号を戻す','商は0方向へ切り捨て、余りは被除数と同符号です。',{a,b,...result,identity:quotient*b+remainder});
    }else if(mode==='gcd'){
      let a=abs(decimal(input.a)),b=abs(decimal(input.b));
      step('絶対値から始める','GCDは非負です。gcd(0,0)=0とします。',{a,b});
      while(b!==0n){const quotient=a/b,remainder=a%b;step('余りへ置き換える','gcd(a,b) = gcd(b,a mod b)。これは説明用のEuclid法です。',{a,b,quotient,remainder});[a,b]=[b,remainder];}
      result={gcd:String(a)};
    }else throw Error('不明なモードです。');
    step('最終結果','C++ APIと同じ値・符号の規約で結果を表示します。',result);
    return {steps,result};
  }
  const api={modes,trace};if(typeof module!=='undefined'&&module.exports)module.exports=api;else root.BigIntegerModel=api;
})(typeof window==='undefined'?globalThis:window);
