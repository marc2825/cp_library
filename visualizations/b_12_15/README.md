# 3.12〜3.15 B：操作できる図解

[index.html](index.html)をブラウザで直接開きます。[30項目の実装と検証](../../docs/b_12_15/README.md) / [一次資料](../../docs/b_12_15/references.md)。JSON入力、最初・前後・再生/停止・最後・サンプル復帰で状態を比較できます。

## 入力と簡略化

モデルは小入力の教材で、C++の性能・数値範囲の代用にはしません。一般の配列は20要素程度、Mongeは18頂点、行列は12×12、ナップサックは20品物、組合せの次数は20程度に制限します。基本の法は101で編集可能。モデル固有の上限違反はエラーへ表示します。

| モード | 表示する状態とC++との差異 |
|---|---|
| Min-Plus | 形状を検査し、反対角線の全候補と最小値を表示。C++は凸merge・Monge探索・凹の部分包絡。 |
| Monge最短路／LARSCH | 各行の候補dp[j]+cost(j,i)と確定値。モデルは全走査、C++はオンラインLARSCHの縮約。入力xは非減少、辺費用は(x[j]−x[i])²+vertex_cost[j]。 |
| 辺数制約Monge | 段ごとのDPと復元パス。モデルは全候補、C++は段ごとのSMAWK。 |
| CDQ | 左の確定、区間から右への寄与、葉の確定。モデルの一括転送は直接和、C++具体版はNTT。 |
| 分枝限定ナップサック | 密度順、分数上界、採用集合、枝刈り。値を小さく制限し、整数上界の判断を表示。C++は128bitで密度と上界を計算。 |
| Four Russians | Bの行OR表とAのmaskからの選択、現在のBoolean積。モデルは0/1配列、C++は64bit列。LCS用の版ではない。 |
| Discrete Log | gcd縮約、babyの最小指数、giantの照合。モデルの法は10^6以下、C++はsigned64とbaby予算。 |
| mod平方根 | Tonelli–Shanksの位数・補正元・根候補・誤差。モデルは小さい素数法、C++は64bit素数検査と安全な乗算。 |
| 整数根 | BigIntの二分探索。nはuint64の10進文字列も可。64bit極値を正確に保持。 |
| modテトレーション | φ連鎖と実指数の飽和判定。a/heightはuint64の10進文字列も可、法は10^6以下に制限しtrial divisionでφを計算。 |
| 商列挙／約数個数和 | 閉区間[l,r]と商、区間長×商の累積。モデルはN<=10^9、C++はuint64入力と128bit総和。 |
| 線形合同式／Lucas | gcdと全解の等差構造、p進桁ごとの二項係数。Lucasのn/kはBigInt、p<=101でPascal表を使用。 |
| 素数カウント2入口 | 同じLucy DPの商状態。モデルはN<=10^5、C++では明示的なsqrt(N)メモリ予算。 |
| φ総和／Mertens | 小prefixと、商区間から計算したmemoの状態。モデルは小さいNとNumber、C++はsigned128。 |
| 乗法的関数総和／Square-Free | 素因数冪ごとの合成とprefix、μ(i)floor(N/i²)の包除。モデルは直接分解、C++はSPF／μ篩。乗法的関数はtau/sigma/phi/muを切替。 |
| Eulerian／Bernoulli／Narayana | 三角漸化式、二項係数からBernoulli、Narayanaの二項係数積。逆元が存在しない入力は拒否。 |
| Lagrange Inversion | φの繰り返し乗算とF'の寄与。C++は既存FPS冪を使用。 |
| Stirling／二項変換／階乗基底 | 三角行列と寄与、転置方向、符号、上昇／下降基底。小入力でC++と同じ定義を使用。 |
| 標本点シフト | 直接Lagrange補間の寄与を各点で表示。C++は補間とProductTree多点評価を共有。 |
| 二項Prefix | Pascal行とprefixの構築、kの境界clamp。C++は前計算してO(1)query。 |

250状態を超えると途中表示を打ち切って最終結果へ進めます。結果は最後まで計算します。各状態の表と配列はスナップショットで、後の更新に影響されません。行列や長い表は表示枠内で横スクロールできます。

## 検証

```sh
node verify/visualizations/b_12_15_model_test.js
node verify/visualizations/b_12_15_browser_test.mjs --browser /path/to/chromium
```

モデルは全モードのサンプル・入力不変性と、パス全列挙、有限剰余軌道、全根、任意精度二項係数、全品物subset、直接行列積、gcd・約数・平方因子、全置換・Dyckパス・集合分割、べき和・多項式評価を比較します。ブラウザは入力編集、前後、再生、エラー回復、切替、リンク、390px表示を確認します。
