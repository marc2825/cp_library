# インタラクティブ解説

[すべての解説を開く](index.html)

各カテゴリのフォルダに実装を置きます。実装言語は自由ですが、このカテゴリは外部通信や追加パッケージなしで開ける HTML / CSS / JavaScript / SVG で作っています。

## 3.2 Union-Find

- [通常版・成分サイズ／個数](union_find/index.html#ordinary)
- [重み付き版・差分制約・矛盾判定](union_find/index.html#potential)
- [仕様・使用例・計算量・問題例](../docs/union_find/README.md)

モード切替、任意の併合／差分問い合わせ、経路圧縮の追跡、戻る／次へ／再生、サンプルシナリオに対応します。図の矢印は DSU 内部の親ポインタであり、入力グラフの辺そのものではありません。

以降のカテゴリを追加するときは、`index.html` と該当する `docs/` の両方からリンクします。

## 3.3 セグメント木・Fenwick Tree

- [Fenwick Tree・lower_bound](range_query/index.html#fenwick)
- [Segtree](range_query/index.html#segtree)
- [Lazy Segtree](range_query/index.html#lazy)
- [Dual Segtree](range_query/index.html#dual)
- [区間加算・区間和 BIT](range_query/index.html#range-bit)
- [仕様・使用例・計算量・問題例](../docs/range_query/README.md)

配列の初期化、任意の更新／問い合わせ、戻る／次へ／再生、おすすめの操作列に対応します。更新途中で新しい操作を実行した場合は、現在の操作を完了してから続けます。操作の開始・完了位置まで戻ると、その時点から履歴を作り直せます。

図は原理を見やすくするため再帰的な区間分解を用いる部分があり、C++ の反復実装の各命令との対応は意図していません。全モードの結果を素朴配列と比較します。

## 3.4 静的クエリ・単調構造・SWAG

- [Sparse Table](static_query/index.html#sparse) — 前計算と 2 区間の重なり。
- [Monotonic Stack](static_query/index.html#stack) — 左右の最近傍、厳密／同値を含む比較。
- [Monotonic Queue](static_query/index.html#queue) — 論理要素と候補列の違い。
- [Sliding Window Minimum](static_query/index.html#window) — 期限切れと最左の argmin。
- [SWAG](static_query/index.html#swag) — 連結／和、2 スタックの移し替えと集約方向。
- [仕様・usage・計算量・問題例](../docs/static_query/README.md)。

配列・窓幅・優先する大小・厳密性・走査方向・集約を変更できます。ステップ処理、キューの任意 push/pop、問い合わせ、再生と履歴の分岐に対応します。キューの入力全体は説明用に表示し、C++ の単調キューが保持するのは候補のみです。移し替え途中の SWAG は、残る back の前半と front へ移した後半を並べて元の列を表示します。

## 3.5 標準コンテナ・座標圧縮・順位管理

- [set](ordered_structure/index.html#set) — 重複しない集合と前後検索。
- [multiset](ordered_structure/index.html#multiset) — 1 個の削除と全削除。
- [map](ordered_structure/index.html#map) — try_emplace、insert_or_assign、find、operator[]。
- [座標圧縮](ordered_structure/index.html#compression) — ソート・同値の集約・境界位置。
- [圧縮＋BIT](ordered_structure/index.html#statistics) — 頻度、累積順位、k 番目と前後検索。
- [API・usage・計算量・問題例](../docs/ordered_structure/README.md)。

初期要素・登録候補・昇降順を変更し、更新・削除・検索・再生・履歴の分岐を試せます。初期 map の値はキー×10 にしています。標準コンテナの図は論理的な並びと比較の概念であり、実装依存の木の形状を再現していません。数値の図解では安全な JavaScript 整数を使い、C++ の個数上限とは別に入力を制限します。

## 3.6 オフラインクエリ・二次元データ構造

- [Mo](offline_query/index.html#mo) — 種類数、区間の伸縮、処理順の移動経路。
- [並列二分探索](offline_query/index.html#pbs) — 辺追加・初回連結、low/mid/high、毎ラウンドの reset。
- [Sweep Line](offline_query/index.html#sweep) — 点と走査線、y の圧縮・BIT、半開長方形の境界。
- [CDQ](offline_query/index.html#cdq) — 時刻分割、左更新 → 右問い合わせ、4 つの prefix 和、巻き戻し。
- [API・usage・計算量・問題例](../docs/offline_query/README.md)。

配列・ブロック幅・辺列・点・問い合わせ・操作列を編集できます。「条件を反映して実行」でバッチ全体から履歴を作り直し、戻る／次へ／再生で途中を追います。CDQ の時刻列は展開後のイベント順です。薄い点には未来の更新も含まれ、BIT に入るのは現在の分割で有効な左側の更新だけです。数値は図解用に ±100 万以内へ制限しています。

## 3.7 グラフ：基本探索・最短路・MST

- [BFS](graph_basic/index.html#bfs) / [DFS](graph_basic/index.html#dfs) — FIFO とスタック、行きがけ順・帰りがけ順。
- [0-1 BFS](graph_basic/index.html#zero-one) / [Dijkstra](graph_basic/index.html#dijkstra) — deque と優先候補、緩和・古い候補。
- [Bellman–Ford](graph_basic/index.html#bellman) / [Warshall–Floyd](graph_basic/index.html#floyd) — 負辺・負閉路の影響、全点対の行列。
- [トポロジカルソート](graph_basic/index.html#topological) / [閉路検出](graph_basic/index.html#cycle) — 入次数、3 色 DFS、自己ループ・多重辺。
- [Kruskal](graph_basic/index.html#kruskal) / [Prim](graph_basic/index.html#prim) — 採用辺、成分、非連結な最小全域森。
- [経路復元](graph_basic/index.html#path) — 親の頂点と辺 ID の逆走査。
- [API・usage・計算量・問題例](../docs/graph_basic/README.md)。

頂点数・向き・辺・重み・始点・終点を変更できます。基本／非連結／負閉路／多重辺等の例を選び、入力から履歴を再生成します。全 11 モードで戻る／次へ／再生が可能です。最短路モードは「経路復元へ」で親の逆走査へ移動できます。DFS は全成分の dfs_forest、優先度付きキューは取り出し順の候補列で示します。C++ の各命令や内部ヒープ形状との一致は意図していません。

## 3.8 グラフ：連結成分・構造分解

- [SCC](graph_connectivity/index.html#scc) / [縮約 DAG](graph_connectivity/index.html#condensation) — 正逆 2 回の DFS、成分順、内部辺と重複辺の除外。
- [Lowlink](graph_connectivity/index.html#lowlink) — ord/low と部分木サイズの伝播。
- [橋](graph_connectivity/index.html#bridges) / [関節点](graph_connectivity/index.html#articulation) — < / ≤、根の特例、辺／頂点の仮削除。
- [二部グラフ](graph_connectivity/index.html#bipartite) / [無向連結成分](graph_connectivity/index.html#components) — 全成分の探索・彩色・衝突。
- [API・usage・計算量・問題例](../docs/graph_connectivity/README.md)。

7 モードで頂点数・辺・例を変更し、逆グラフへの移動、縮約辺の重複集約、再生・前後のステップを試せます。Lowlink／橋／関節点では辺 ID または頂点を仮に削除して成分数の増減を確認できます。そのとき ord/low は元のグラフの値、色分けと成分一覧は削除後です。二部グラフの衝突辺は説明用で、奇閉路全体の復元ではありません。

## 3.9 グラフ：フロー・マッチング・最適化

- [Dinic 最大流](graph_flow/index.html#dinic) — 残余容量・BFS の層・current arc・増加路と逆辺。
- [最小カット復元](graph_flow/index.html#cut) — 最大流後の到達範囲・頂点側・元辺 ID・容量の一致。
- [二部最大マッチング](graph_flow/index.html#matching) — Hopcroft–Karp、左右の相手、最短の交互路と組み替え。
- [API・usage・計算量・問題例](../docs/graph_flow/README.md)。

頂点数・左右の大きさ・有向辺・容量・始点／終点を編集し、入力から履歴を再生成します。前後のステップ・再生・次の増加路／交互路への移動、逆辺表示の切替に対応します。Dinic は流量上限を指定でき、残余辺で終点に届く間は最小カットを未確定と表示します。最小カットモードは常に流し切ります。マッチングの経路の橙／青は、反転前の追加／削除を表します。全てのスナップショットで流量保存とマッチングの妥当性を確認します。

## 3.10 グラフ：組合せ構造・数え上げ

- [三角形列挙](graph_combinatorics/index.html#triangles) — 次数による向き・マーク・一意な列挙。
- [クリーク列挙](graph_combinatorics/index.html#cliques) — 全て／極大のみ、P/X/pivot、空集合・打切り。
- [最大クリーク](graph_combinatorics/index.html#maximum) / [最大独立集合](graph_combinatorics/index.html#independent) — 候補の交差・貪欲彩色の上界・枝刈り・補グラフ。
- [彩色数](graph_combinatorics/index.html#coloring) — 独立集合による厳密 DP・最小彩色の復元。
- [行列木定理](graph_combinatorics/index.html#trees) — ラプラシアン・余因子・素数体上の行交換と消去。
- [API・usage・計算量・問題例](../docs/graph_combinatorics/README.md)。

6 モードで頂点数0〜8、辺を最大16本まで編集できます。列挙件数の上限、極大のみ、空集合を含むか、元グラフ／補グラフ、辺重み・素数mod・除く行列の頂点を変更できます。前後のステップ・再生・注目する状態への移動に対応します。DP は改善と確定を表示し、失敗した全ての候補は表示しません。消去後の行は元の頂点と一対一には対応せず、余因子内の行番号を使います。元のラプラシアンも別に表示します。剰余演算の乗算には BigInt を用います。

## 検証

```sh
node verify/visualizations/union_find_model_test.js
node verify/visualizations/browser_test.mjs --browser /path/to/chromium
node verify/visualizations/range_query_model_test.js
node verify/visualizations/range_query_browser_test.mjs --browser /path/to/chromium
node verify/visualizations/static_query_model_test.js
node verify/visualizations/static_query_browser_test.mjs --browser /path/to/chromium
node verify/visualizations/ordered_structure_model_test.js
node verify/visualizations/ordered_structure_browser_test.mjs --browser /path/to/chromium
node verify/visualizations/offline_query_model_test.js
node verify/visualizations/offline_query_browser_test.mjs --browser /path/to/chromium
node verify/visualizations/graph_basic_model_test.js
node verify/visualizations/graph_basic_browser_test.mjs --browser /path/to/chromium
node verify/visualizations/graph_connectivity_model_test.js
node verify/visualizations/graph_connectivity_browser_test.mjs --browser /path/to/chromium
node verify/visualizations/graph_flow_model_test.js
node verify/visualizations/graph_flow_browser_test.mjs --browser /path/to/chromium
node verify/visualizations/graph_combinatorics_model_test.js
node verify/visualizations/graph_combinatorics_browser_test.mjs --browser /path/to/chromium
```

モデルの状態は独立した BFS オラクルと比較します。ブラウザ検証は Node.js 22 以降と、必要な OS ライブラリが揃った Chromium で実行できます。追加の npm パッケージは不要です。

3.3 のモデルは 500 個の系列・50,000 操作を独立した素朴配列と比較します。各ブラウザ検証に `--screenshot /tmp/visualization.png` を付けると、デスクトップとモバイルの画像を保存できます。

3.4 のモデルは 160 個の入力配列と 189,940 操作について、全区間・最近傍・窓・FIFO・集約を素朴解と比較します。SWAG は移し替え途中の論理順序と集約も確認します。

3.5 のモデルは 200 個の入力と 133,200 操作を素朴配列・辞書と比較します。重複・map の挿入規則・圧縮境界・全 BIT ブロック・順位・失敗時の状態維持を確認します。

3.6 のモデルは 647 バッチ・53,519 スナップショットを配列・BFS・時刻付き点列の素朴解と比較します。頻度、探索範囲、BIT 全ブロック、過去／未来の関係、巻き戻し後の 0 を確認します。

3.7 のモデルは 1,042 バッチ・11,140 スナップショットを独立した歩道 DP・DFS・到達探索・辺部分集合と比較します。負閉路の影響する組、全成分の訪問順、閉路の辺番号と向き、多重辺、復元経路を確認します。

3.8 のモデルは 1,559 ケース・37,101 スナップショットを相互到達性・辺／頂点削除・全彩色の素朴解と比較します。縮約辺の重複と元辺対応、根の特例、仮削除後の成分数も確認します。

3.9 のモデルは 1,479 バッチ・20,355 スナップショットを全カット列挙・右部分集合 DP と比較します。各フレームの容量制約・流量保存・左右の一意性、残余到達性、上限停止、カット元辺 ID、逆向きの増加路と長い交互路も検証します。

3.10 のモデルは 1,809 バッチ・34,537 スナップショットを全頂点部分集合・色割当の直接探索・全辺部分集合と比較します。枝刈りの上界、極大性、全ての DP 状態、行消去の各操作、多重辺・負の重み・行交換・連結でも0 mod pになるケースも確認します。
