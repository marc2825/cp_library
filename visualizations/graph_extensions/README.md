# グラフの追加15項目の図解

[index.html](index.html)を直接開けます。外部通信・依存なし。JSONの入力を編集し、先頭・戻る・再生・次へ・最後で内部状態を追います。青い頂点と太線は現在の対象、辺彩色では線の色が割当色です。表で辺ID・距離・親・残次数・fanを確認できます。

図解は小規模整数入力の説明用モデルです。通常n=1..10、18辺以下、費用−100..100。二部は各側0..6、次数列とXORは空列も許可。XORの値は0..65535。区間辺のopsは`[l,r,a,b,cost]`という区間→区間形式で、長さ1の区間が点を表します。最小費用素パスは12辺・k≤3で辺素/内部頂点素を両方計算します。C++の入力範囲は各APIの説明に従います。

| モード | 表示する手順・C++との違い |
|---|---|
| 区間辺 | 標準区間ノードを表示後、元頂点への展開で距離を計算。C++の補助頂点配置そのものは描かない。 |
| 二部被覆・反鎖 | DFS増加路と交互到達。C++はHopcroft–Karp。反鎖の推移閉包は図解ではFloyd、C++ではbitset。 |
| 辺被覆 | 全部分集合から最大matchingを求め、未対応頂点を補う。C++はBlossom。 |
| Gomory–Hu | 全カットを列挙して親を更新。C++はDinic。 |
| st順序・次数列 | lowpointと符号付きリスト、Havel–Hakimiの残次数を表示。 |
| 最小平均閉路 | Karp DPと最適分数。図解の復元は閉路列挙、C++はtight辺のDFS。 |
| Borůvka・増分MST | 成分ごとの候補と最大辺交換。図解は明示全辺・BFS、C++はtrie・LCT。 |
| なもり | 葉剥がしと閉路根からの探索。C++はCactus分解を共有。 |
| Vizing | KACTLのfan・2色交互路・回転をCC0で移植。各辺追加後の彩色を表示。 |
| 制約・負閉路 | 緩和ラウンド、親と辺ID、閉路の証拠。図解の数値は小さい整数に制限。 |
| 最小費用素パス | 全単純パスの組合せで目的関数を説明。C++の費用流・potentialの内部手順は再現しない。 |

```sh
node verify/visualizations/graph_extensions_model_test.js
node verify/visualizations/graph_extensions_browser_test.mjs --browser /path/to/chromium
```

[公開API・検証](../../docs/graph_extensions/README.md) / [出典](../../docs/graph_extensions/references.md) / [全図解](../index.html)。同じ最適値でもC++と図解で選ばれる証拠が異なる場合があります。
