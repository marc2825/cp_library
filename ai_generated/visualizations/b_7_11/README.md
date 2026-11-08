# 3.7〜3.11 B：操作できる図解

[index.html](index.html)をブラウザで直接開きます。[36項目のAPIと検証](../../docs/b_7_11/README.md) / [一次資料](../../docs/b_7_11/references.md)。入力JSONを変更し、入力を反映・最初・前後・再生/停止・最後・サンプル復帰を操作できます。

グラフは原則7頂点・12辺以下、整数の重み−20〜20。二部版は左右各6頂点以下、b-flow/循環流は6頂点・8辺以下、流量範囲−2〜3、費用−10〜10。森の操作列は40操作以下、素パスのモデルは単純パス候補20個以下に制限します。C++の64/128bit精度・大入力性能はこの図からは検証できません。C++の比較テストは別に実施します。

## 図に表示する状態と簡略化

| モード | 追える状態・C++との差異 |
|---|---|
| Directed MST | 最小入辺と実行可能有向木。最適値は全列挙で確認。縮約の全階層は表示せず、C++はChu–Liu/Edmonds。 |
| Manhattan MST | 完全グラフのL1辺とKruskal採用。C++の4方向sweepによる候補絞り込みは省略。 |
| 最短有向／無向閉路 | 改善する閉路と辺ID。モデルは単純閉路のDFS、C++はDijkstra。 |
| MST感度／第二最小 | 基準木、各辺の強制／禁止費用、等費用とstrictの区別。モデルは全木、C++はHLDの最大・chmin。 |
| Dominator | 一頂点の除去前後で到達できなくなる頂点。C++はLengauer–Tarjan。 |
| 三重辺連結 | 各頂点対の最小cutと併合。モデルは全cut、C++はpath absorption。 |
| Cactus | 各辺の閉路所属数。モデルは閉路の全列挙、C++はLowlink block。 |
| Chordal／幅2 | MCSの重みと完全消去順序、次数2消去・fill・bag。これらはC++と同じ基本手順。 |
| Incremental SCC | 各辺追加後のSCC。モデルは到達閉包で再計算、C++は縮約DAGの交差到達集合を併合。 |
| DAG到達 | 逆topological順のbit集合伝播。モデルは1整数のbitset、C++は64bit列とバッチ版。 |
| b-flow／循環流 | balanceと、改善する実行可能流。モデルは流量列挙、C++は飽和・SSP・双対証明。 |
| Stoer–Wagner／Push–Relabel | phaseの隣接重み・縮約、残余容量・高さ・超過流。Push–Relabelのactive選択順は教材では最小番号。 |
| DM／最大重み二部 | 交互辺・余剰領域・SCC、改善するマッチング。モデルは全マッチング、C++はHopcroft–Karp／Assignment。 |
| 二部辺彩色 | Δ色の辺割当。モデルはbacktracking、C++は2色交互路の交換。 |
| 彩色多項式 | 独立集合による集合分割DPと下降階乗基底の係数変換。C++と同じ漸化式。 |
| C4／極大clique | 重複を除いた閉路／部分集合。C++は次数順2辺walk／Bron–Kerbosch。 |
| Steiner／DAG cover／素パス | 改善する辺集合、マッチングとパス連結、共有しないパス。モデルは全列挙、C++はsubset DP／matching／flow。 |
| 簡易同型／木同型 | 同時1-WLの色、子の括弧列。木モデルは根を全て試し、C++は重心と辞書。同型簡易版の未確定を維持。 |
| Static Top Tree | heavy chainとlight枝のrake/compress式、頂点更新後の式の値。例のSpecは成分和。モデルは全式を再評価、C++は祖先だけ更新。 |
| Link-Cut／Euler Tour Tree | パス露出後の値／自己tokenと両向き辺tokenのEuler列。モデルはBFS／列を再構築、C++はsplay／treap。 |
| 動的reroot／動的直径 | 更新後の各根距離和・距離表と直径。モデルは直接走査、C++はtop treeのContext／端点情報。topology固定。 |
| 距離分布／Contour | 頂点対の距離頻度、距離帯の対象・点値。モデルは全始点BFS、C++は重心と畳み込み／all−sub BIT。 |
| マージ過程 | 葉と履歴内部節点、成功した併合の親子関係。C++と同じ履歴構造。 |

表・グラフは各ステップのスナップショットです。全列挙で250状態を超える場合は表示を打ち切って最終結果へ進めます。グラフの辺はID順、無向辺に矢印は付けません。Static Top Treeと履歴木は親子を階層配置します。表示中の表は横スクロール可能です。

## 検証

```sh
node verify/visualizations/b_7_11_model_test.js
node verify/visualizations/b_7_11_browser_test.mjs --browser /path/to/chromium
```

モデルでは全モードの入力不変性・スナップショット・最終結果と、小入力の独立した定義を比較します。ブラウザは36モードの入力編集、前後、再生、エラー復帰、切替、ローカルリンク、390px表示を検査します。
