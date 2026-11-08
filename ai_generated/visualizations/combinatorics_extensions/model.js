(function(root){
  'use strict';
  const MOD=998244353n;
  const modes=[{"id": "q_binomial", "title": "C-01 q二項係数・q階乗", "sample": {"prime": 7, "q": 2, "n": 8, "k": 3}, "doc": "combinatorics/q_binomial.md"}, {"id": "bounded_monotone", "title": "C-02 上下境界付き単調列・格子路", "sample": {"lower": [0, 1, 1, 2], "upper": [2, 3, 4, 4], "strict": false}, "doc": "combinatorics/bounded_monotone.md"}, {"id": "stirling_small_prime", "title": "C-03 小素数法・巨大引数のStirling数", "sample": {"prime": 5, "n": "1000000000003", "k": "3"}, "doc": "combinatorics/stirling_small_prime.md"}, {"id": "unlabeled_trees", "title": "C-04 ラベルなし木の同型類数", "sample": {"n": 8}, "doc": "combinatorics/unlabeled_trees.md"}, {"id": "rsk", "title": "C-20 RSK対応と逆対応", "sample": {"word": [3, 1, 4, 1, 5]}, "doc": "combinatorics/rsk.md"}, {"id": "young_tableaux", "title": "C-21 Young図形と標準盤の列挙", "sample": {"shape": [3, 2]}, "doc": "combinatorics/young_tableaux.md"}, {"id": "hook_length", "title": "C-22 Hook-Length / Hook-Content", "sample": {"shape": [3, 2], "alphabet": 3, "modulus": 4}, "doc": "combinatorics/hook_length.md"}, {"id": "combinatorial_transforms", "title": "C-23 Lah・Euler変換と逆変換", "sample": {"values": [0, 1, 1, 1, 1, 1]}, "doc": "combinatorics/combinatorial_transforms.md"}];
  const clone=x=>JSON.parse(JSON.stringify(x,(_,v)=>typeof v==='bigint'?v.toString():v));
  const integer=(x,name,lo,hi)=>{if(!Number.isSafeInteger(x)||x<lo||x>hi)throw Error(`${name}: ${lo}..${hi}の整数を指定してください`);return x;};
  const uint=(x,name)=>{if(typeof x==='number'&&!Number.isSafeInteger(x))throw Error(`${name}: 大きい整数は文字列で指定してください`);if(!/^[0-9]+$/.test(String(x)))throw Error(`${name}: 非負整数が必要です`);const b=BigInt(x);if(b>18446744073709551615n)throw Error(`${name}: uint64を超えています`);return b;};
  const vector=(a,name,limit,lo,hi)=>{if(!Array.isArray(a)||a.length>limit)throw Error(`${name}: 長さ${limit}以下の配列が必要です`);return a.map(x=>integer(x,name,lo,hi));};
  const prime=x=>{integer(x,'prime',2,43);for(let d=2;d*d<=x;d++)if(x%d===0)throw Error('prime: 素数が必要です');return x;};
  const norm=(a,p=MOD)=>(a%p+p)%p;
  const pow=(a,e,p=MOD)=>{let r=1n;a=norm(a,p);for(;e;e>>=1n,a=a*a%p)if(e&1n)r=r*a%p;return r;};
  const choose=(n,k)=>{if(k<0||k>n)return 0n;k=Math.min(k,n-k);let b=1n;for(let i=1;i<=k;i++)b=b*BigInt(n-k+i)/BigInt(i);return b;};
  const lucas=(n,k,p)=>{let b=1n,P=BigInt(p);while(n||k){b=b*choose(Number(n%P),Number(k%P))%P;n/=P;k/=P;}return Number(b);};
  const shapeOf=a=>vector(a,'shape',40,1,40).map((x,i)=>{if(i&&x>a[i-1])throw Error('shape: 行長は非増加にしてください');return x;});
  const conjugate=s=>Array.from({length:s[0]||0},(_,c)=>s.filter(x=>x>c).length);
  function trace(id,input){
    if(!input||typeof input!=='object'||Array.isArray(input))throw Error('入力はJSONオブジェクトにしてください');
    const steps=[];
    const push=(title,detail,columns=[],rows=[],values=[],state=null,plot=null)=>steps.push(clone({title,detail,columns,rows,values,state,plot}));
    const grid=(title,cells,highlight)=>({grids:[{title,cells,highlight}]});
    let result=[];
    push('入力を確認','入力を編集し、次へ進むと計算状態を追えます。');
    if(id==='q_binomial'){
      const p=prime(input.prime),q=((integer(input.q,'q',-1000,1000)%p)+p)%p,n=integer(input.n,'n',0,25),k=integer(input.k,'k',0,40);
      let row=[1],fac=1,qi=1,sum=0,period=0;
      for(let i=1;i<=p;i++){sum=(sum+qi)%p;qi=qi*q%p;if(sum===0){period=i;break;}}
      qi=1;sum=0;
      for(let i=1;i<=n;i++){
        const next=Array(i+1).fill(0);let power=1;
        for(let j=0;j<=i;j++){next[j]=((row[j-1]||0)+power*(row[j]||0))%p;power=power*q%p;}
        sum=(sum+qi)%p;qi=qi*q%p;fac=fac*sum%p;row=next;
        push(`n = ${i}`,'図解はq-Pascalで1行ずつ更新。C++は位数とq-Lucasで巨大nを処理します。',['k','q二項係数'],row.map((x,j)=>[j,x]),[{label:'q階乗',value:fac},{label:'最初の消失',value:period||'なし'}]);
      }
      result=[row[k]||0,fac,period];
    }else if(id==='bounded_monotone'){
      let lo=vector(input.lower,'lower',12,-30,30),hi=vector(input.upper,'upper',12,-30,30);
      if(lo.length!==hi.length)throw Error('lowerとupperの長さを揃えてください');
      if(typeof input.strict!=='boolean')throw Error('strictはtrue/falseです');
      const n=lo.length;
      lo=lo.map((x,i)=>x-(input.strict?i:0));hi=hi.map((x,i)=>x-(input.strict?i:0));
      for(let i=1;i<n;i++)lo[i]=Math.max(lo[i],lo[i-1]);for(let i=n-2;i>=0;i--)hi[i]=Math.min(hi[i],hi[i+1]);
      if(!n)result=[1];else if(lo.some((x,i)=>x>hi[i]))result=[0];else{
        const base=lo[0],height=hi[n-1]-base+1;if(height>32)throw Error('図解の高さは32以下にしてください');
        const cells=Array.from({length:n+1},()=>Array(height).fill(null));
        for(let x=0;x<=n;x++){
          for(let j=0;j<height;j++)if(lo[Math.max(0,x-1)]<=base+j&&base+j<=hi[Math.min(x,n-1)])cells[x][j]=((x?cells[x-1][j]||0:0)+(j?cells[x][j-1]||0:0)+(x===0&&j===0?1:0))%998244353;
          push(`列 ${x} を更新`,'図の行が格子路のx、列が高さです。×は境界外。C++は許可長方形を畳み込みでまとめて転送します。',['i','下限','上限'],lo.map((v,i)=>[i,v,hi[i]]),[],grid(`高さ ${base}..${hi[n-1]}`,cells,[x,height-1]));
        }
        result=[cells[n][height-1]||0];
      }
    }else if(id==='stirling_small_prime'){
      const p=prime(input.prime),n=uint(input.n,'n'),k=uint(input.k,'k'),P=BigInt(p),d=P-1n;
      const c=Array.from({length:p},()=>Array(p).fill(0)),h=Array.from({length:p},()=>Array(p-1).fill(0));c[0][0]=h[0][0]=1;
      for(let r=1;r<p;r++){for(let j=1;j<=r;j++)c[r][j]=(c[r-1][j-1]+(r-1)*c[r-1][j])%p;h[r][0]=1;for(let t=1;t<p-1;t++)h[r][t]=(h[r-1][t]+r*h[r][t-1])%p;}
      const q=n/P,r=Number(n%P);
      push('上昇階乗をp個ずつ分解','(x^p−x)^q と、小さいr行目の積へ分けます。',['次数','係数'],c[r].map((x,i)=>[i,x]),[{label:'q',value:q},{label:'r',value:r}]);
      let first=0,second=0;
      if(k<=n&&k>=q){const i=(k-q)%d,j=(k-q)/d;for(let offset=0;offset<=1;offset++){const a=Number(i+BigInt(offset)*d),b=j-BigInt(offset);if(a<=r&&b>=0n&&b<=q){let term=c[r][a]*lucas(q,b,p)%p;if((q-b)&1n)term=(p-term)%p;first=(first+term)%p;}}}
      if(k<=n){if(k===0n)second=n===0n?1:0;else{const a=k/P,b=Number(k%P),t=n-k,u=t/d,v=Number(t%d);second=b?h[b][v]*lucas(a+u,u,p)%p:v?0:lucas(a+u-1n,u,p);}}
      push('周期表とLucas二項係数','第2種はk=qp+rへ分け、周期p−1の完全対称式を使います。',['r',...Array.from({length:p-1},(_,i)=>i)],h.map((row,i)=>[i,...row]),[{label:'第1種',value:first},{label:'第2種',value:second}]);result=[first,second];
    }else if(id==='unlabeled_trees'){
      const n=integer(input.n,'n',0,24),t=Array(n+1).fill(0n),u=[...t],b=[...t];
      for(let v=1;v<=n;v++){
        if(v===1)t[v]=1n;else{for(let j=1;j<v;j++)t[v]+=b[j]*t[v-j];t[v]/=BigInt(v-1);}
        for(let j=v;j<=n;j+=v)b[j]+=BigInt(v)*t[v];let pairs=0n;for(let j=1;j<v;j++)pairs+=t[j]*t[v-j];if(v%2===0)pairs-=t[v/2];u[v]=t[v]-pairs/2n;
        push(`${v}頂点の同型類`,'根付き木の子は根付き木の多重集合。辺を根とする寄与を引いて無根木を得ます。',['頂点','根付き','無根'],t.slice(1,v+1).map((x,i)=>[i+1,x,u[i+1]]),[],null,t.slice(1,v+1).map((x,i)=>[i+1,Number(x)]));
      }result=[...t.slice(1),...u.slice(1)];
    }else if(id==='rsk'){
      const word=vector(input.word,'word',16,-99,99),p=[],q=[];
      for(let time=0;time<word.length;time++){
        let value=word[time],r=0;
        while(true){if(!p[r]){p[r]=[];q[r]=[];}let c=p[r].findIndex(x=>x>value);
          if(c<0){c=p[r].length;p[r].push(value);q[r].push(time+1);push(`挿入 ${word[time]}：行末へ追加`,'追加位置に時刻を記録します。',[],[],[],{grids:[{title:'P：挿入盤',cells:p,highlight:[r,c]},{title:'Q：記録盤',cells:q,highlight:[r,c]}]});break;}
          [p[r][c],value]=[value,p[r][c]];push(`挿入 ${word[time]}：${value}を次の行へ`,'最初の「より大きい」値を押し出します。',[],[],[],{grids:[{title:'P：挿入盤',cells:p,highlight:[r,c]},{title:'Q：記録盤',cells:q}]});r++;
        }
      }
      const shape=p.map(r=>r.length),restored=[];
      for(let label=word.length;label;label--){const r=q.findIndex(row=>row.at(-1)===label);q[r].pop();let value=p[r].pop();if(!p[r].length){p.pop();q.pop();}
        for(let j=r-1;j>=0;j--){let c=p[j].length-1;while(p[j][c]>=value)c--;[value,p[j][c]]=[p[j][c],value];}restored.push(value);
        push(`逆対応：時刻${label}`,'最大時刻の角を外し、上の行の右端の小さい値と交換します。',[],[],[{label:'取り出した値',value}],{grids:[{title:'P',cells:p},{title:'Q',cells:q}]});
      }result=[...shape,...restored.reverse()];
    }else if(id==='young_tableaux'){
      const shape=shapeOf(input.shape),n=shape.reduce((a,b)=>a+b,0);if(n>8)throw Error('全列挙の図解は8セルまでです');
      const t=shape.map(()=>[]);let count=0;
      function visit(label){if(label>n){count++;push(`標準盤 ${count}`,'各段階でYoung図形を保つ角に、1から順に置いた盤です。',[],[],[{label:'列挙数',value:count}],grid('標準Young盤',t));return;}
        for(let r=0;r<shape.length;r++)if(t[r].length<shape[r]&&(!r||t[r].length<t[r-1].length)){t[r].push(label);visit(label+1);t[r].pop();}}
      visit(1);result=[count,...conjugate(shape)];
    }else if(id==='hook_length'){
      const shape=shapeOf(input.shape),n=shape.reduce((a,b)=>a+b,0),alphabet=uint(input.alphabet,'alphabet'),mod=uint(input.modulus,'modulus');if(n>40)throw Error('hook図解は40セルまでです');if(!mod)throw Error('modulusは正にしてください');
      const columns=conjugate(shape),hooks=shape.map((len,r)=>Array.from({length:len},(_,c)=>len-c+columns[c]-r-1));let numerator=1n,denominator=1n,content=1n;
      for(let i=1;i<=n;i++)numerator*=BigInt(i);
      for(let r=0;r<shape.length;r++)for(let c=0;c<shape[r];c++){denominator*=BigInt(hooks[r][c]);content*=alphabet+BigInt(c-r);push(`セル (${r}, ${c})`,'hook長は右と下と自身。標準盤はN!をhook積で割ります。',[],[],[{label:'N!',value:numerator},{label:'hook積',value:denominator}],grid('hook長',hooks,[r,c]));}
      const count=numerator/denominator,semi=alphabet<BigInt(shape.length)?0n:content/denominator;result=[count,count%mod,semi];
    }else if(id==='combinatorial_transforms'){
      const a=vector(input.values,'values',20,-100,100).map(x=>norm(BigInt(x))),n=a.length;if(n&&a[0])throw Error('Euler正変換にはvalues[0]=0が必要です');
      const lah=Array.from({length:n},()=>Array(n).fill(0n)),b=Array(n).fill(0n),c=[...b],s=[...b],back=[...b];if(n){lah[0][0]=1n;b[0]=a[0];c[0]=1n;}
      for(let i=1;i<n;i++){
        for(let k=1;k<=i;k++)lah[i][k]=norm(lah[i-1][k-1]+BigInt(i+k-1)*lah[i-1][k]);
        for(let k=1;k<=i;k++){b[i]=norm(b[i]+lah[i][k]*a[k]);if(i%k===0)s[i]=norm(s[i]+BigInt(k)*a[k]);}
        for(let k=1;k<=i;k++)c[i]=norm(c[i]+s[k]*c[i-k]);c[i]=c[i]*pow(BigInt(i),MOD-2n)%MOD;
        push(`係数 ${i}`,'図解はLah行列積とEuler積の対数微分を直接評価。C++は畳み込みとFPS log/expを使います。',['次数','Lah','Euler'],b.slice(0,i+1).map((x,j)=>[j,x,c[j]]));
      }
      // B'/B の係数を独立に解き、約数成分を除去する。
      const logarithmic=Array(n).fill(0n);for(let i=1;i<n;i++){let value=BigInt(i)*c[i];for(let j=1;j<i;j++)value-=logarithmic[j]*c[i-j];logarithmic[i]=norm(value);let residual=logarithmic[i];for(let d=1;d<i;d++)if(i%d===0)residual-=BigInt(d)*back[d];back[i]=norm(residual)*pow(BigInt(i),MOD-2n)%MOD;}
      push('Euler逆変換','対数微分から真の約数の寄与を除き、元の指数列に戻します。',['次数','復元した指数'],back.map((x,i)=>[i,x]));result=[...b,...c,...back];
    }else throw Error('不明なアルゴリズムです');
    result=result.map(String);push('計算完了','下の結果欄で係数・盤の形・個数を確認できます。',[],[],[{label:'出力数',value:result.length}]);return {steps,result};
  }
  const api={modes,trace};if(typeof module!=='undefined'&&module.exports)module.exports=api;else root.CombinatoricsExtensionsModel=api;
})(typeof window==='undefined'?globalThis:window);
