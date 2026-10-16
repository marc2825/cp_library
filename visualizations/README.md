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

## 3.11 木アルゴリズム

- [部分木・深さ](tree/index.html#rooted) — 行きがけ／帰りがけ、親・サイズ・半開区間。
- [LCA](tree/index.html#lca) — 祖先表、深さ調整、同時移動とパス上のk番目。
- [Euler Tour](tree/index.html#euler) — preorder・往復の頂点列・辺ID列の違い。
- [木の直径](tree/index.html#diameter) — 非負重みの2回走査と復元。
- [全方位木DP](tree/index.html#rerooting) — 距離和、親側の寄与、prefix/suffixによる除外。
- [Binary Lifting](tree/index.html#lifting) — 任意の遷移・閉路・-1終端・uint64の回数。
- [Tree DP](tree/index.html#dp) — 独立集合の白／黒の状態、帰りがけの集約。
- [API・usage・計算量・問題例](../docs/tree/README.md)。

7モードで根・頂点数0〜12・重み・始点／終点・k・遷移先を編集できます。形や重みの例、前後のステップ・再生・次の段階への移動に対応します。全方位木DPでは各隣接先の寄与と累積表を並べ、どの子を除いたかを表示します。図解のDPは距離和／独立集合の例で、C++の任意のコールバックの実行画面ではありません。Liftingの表は入力kに必要な段数のみを表示します。木のモードは森にも対応し、直径だけ連結・非負重みを要求します。

## 3.12 DP・最適化

- [LIS](dp_optimization/index.html#lis) / [LCS](dp_optimization/index.html#lcs) / [編集距離](dp_optimization/index.html#edit) — 二分探索、接頭辞の表、解の復元。
- [0-1](dp_optimization/index.html#zero-one) / [完全](dp_optimization/index.html#unbounded) / [個数制限](dp_optimization/index.html#bounded) — 更新の向き、剰余類の単調キュー。
- [Bitset Knapsack](dp_optimization/index.html#bitset) — シフトとOR、64ビット語、最終語の余白。
- [区間DP](dp_optimization/index.html#interval) / [桁DP](dp_optimization/index.html#digit) / [部分集合DP](dp_optimization/index.html#subset) — 分割、tight/started、巡回路・部分集合列挙・集合変換。
- [Largest Rectangle](dp_optimization/index.html#rectangle) / [二値グリッド](dp_optimization/index.html#grid) — 左端を引き継ぐ単調スタックと行ごとの高さ。
- [API・usage・計算量・問題例](../docs/dp_optimization/README.md)。

12モードで入力・狭義／広義・昇降順、容量・価値・個数、桁和の法・0を含むか、欠辺と始点、障害物を編集できます。部分集合では4種類の操作を選べます。空・同値・負値・重さ0・64ビット境界等の例、前後のステップ・再生・次の段階への移動に対応します。図解の区間DPは隣接結合、桁DPは桁和の例です。C++の任意のコールバックは実行しません。BitsetはBigIntで同じ集合演算を行い、C++の各機械語命令は再現しません。

## 3.13 数論：基本整数演算

- [GCD・LCM](number_theory_basic/index.html#gcd) — 余りを小さくする互除法と、LCMの整数範囲を確かめます。
- [拡張Euclid](number_theory_basic/index.html#extended) — 各余りの係数を更新し、Bézoutの恒等式を保ちます。
- [mod累乗](number_theory_basic/index.html#pow) — 指数をビットに分け、積と二乗を追います。
- [mod逆元](number_theory_basic/index.html#inverse) — GCDが1のときだけ、係数を逆元へ正規化します。
- [素数篩](number_theory_basic/index.html#sieve) — 素数の平方から倍数を消し、候補を絞ります。
- [SPF・最小素因数](number_theory_basic/index.html#spf) — 合成数を一度ずつ生成し、表をたどって分解します。
- [素因数分解](number_theory_basic/index.html#factor) — 試し割りで素因数を取り出し、残る素数を見つけます。
- [約数列挙](number_theory_basic/index.html#divisors) — 小さい約数と大きい約数を対で集め、昇順につなぎます。
- [API・usage・計算量・問題例](../docs/number_theory_basic/README.md)。

8モードで負数・整数極値・指数・法・篩の上限・分解する数を変更できます。0・1・逆元なし・LCM超過・平方等の例、前後のステップ・再生・次の段階への移動に対応します。計算はBigIntで行い、Euclidの係数・累乗のビット・篩の除去・SPFの生成と分解経路・約数の2列を表示します。図解の上限は篩80・試し割り10,000です。余り0の係数もBigIntで表示しますが、C++はその不要な計算を省きます。mod乗算の各加算は再現しません。

## 3.14 数論：乗法的関数・高度な整数問題

- [約数ゼータ変換](number_theory_convolution/index.html#divisor-zeta) — 素数の冪に沿って、約数の値を小さい方から集めます。
- [約数メビウス変換](number_theory_convolution/index.html#divisor-mobius) — 加えた順序を逆にたどり、約数和から元の値へ戻します。
- [倍数ゼータ変換](number_theory_convolution/index.html#multiple-zeta) — 大きい添字から、倍数の値を小さい方へ伝えます。
- [倍数メビウス変換](number_theory_convolution/index.html#multiple-mobius) — 倍数の集計から、ちょうどの値へ逆変換します。
- [Dirichlet Convolution](number_theory_convolution/index.html#dirichlet) — 添字の積i*jへ係数の積を加え、約数ごとに集計します。
- [GCD Convolution](number_theory_convolution/index.html#gcd) — 倍数ゼータ・要素積・逆変換で、GCDごとの和を作ります。
- [LCM Convolution](number_theory_convolution/index.html#lcm) — 約数ゼータ・要素積・逆変換で、LCMごとの和を作ります。
- [API・usage・計算量・問題例](../docs/number_theory_convolution/README.md)。

7モードで2列の係数・整数／mod12／998244353／1・添字0・注目する添字を変更できます。素数の冪、負の値、出力の切り捨てと0拡張、64ビット境界、正の範囲が空の例を用意しています。各更新、素数の区切り、GCD/LCMの3段階を戻る／次へ／再生で追えます。元の入力から定義通りに集計した項と、作業中の値・履歴を併記します。BigIntで計算し、C++標準整数の超過例外は再現しません。配列は正の添字12個までです。

## 3.15 組合せ論・数え上げ

- [階乗・逆階乗](combinatorics/index.html#factorial) — 階乗を順に作り、最後の逆元から逆向きに表を埋めます。
- [二項係数](combinatorics/index.html#binomial) — 高速な階乗方式と、任意法のPascalの三角形を比べます。
- [順列](combinatorics/index.html#permutation) — 選んだ要素を並べる順序を区別して数えます。
- [組合せ](combinatorics/index.html#combination) — 同じ集合のk!種類の並べ方をまとめて数えます。
- [Catalan数](combinatorics/index.html#catalan) — 括弧列が最初に違反する部分を反転して対応を見ます。
- [重複組合せ](combinatorics/index.html#multiset) — 星と仕切りで、種類ごとの個数を表します。
- [包除原理](combinatorics/index.html#inclusion) — 共通部分を交互に加減算し、各要素の係数を追います。
- [API・usage・計算量・問題例](../docs/combinatorics/README.md)。

7モードでn・k・前計算の上限・法・計算法、全体集合・各集合・和集合／補集合を変更できます。0・合成数法・法1・大きい法・逆元なし・前計算不足・全て重なる集合の例を用意しています。階乗と逆階乗、Pascalの各行、式へ代入する項、括弧列の反射、星と仕切り、包除の各項と要素ごとの係数を、前後・再生・次の段階で追います。上限24、n/kは12まで、包除は全体12要素・6集合です。図の選択や括弧列は一例であり、C++の高速式が実際に全列挙するものではありません。

## 3.16 畳み込み・高速変換

- [NTT](convolution/index.html#ntt) — 有限体の単位根、ビット反転、バタフライを追います。
- [通常のmod畳み込み](convolution/index.html#convolution) — 0埋め・変換・各点積・逆変換で多項式を掛けます。
- [整数畳み込み・CRT](convolution/index.html#integer) — 3法の剰余から、負の値も正確に復元します。
- [実数FFT](convolution/index.html#fft-real) — 実数列の全スペクトルと共役対称性を見ます。
- [複素FFT](convolution/index.html#fft-complex) — 複素平面の単位根と、逆変換の正規化を追います。
- [FFT畳み込み](convolution/index.html#convolution-fft) — 実数・複素数の積を求め、定義式との差を見ます。
- [API・usage・計算量・問題例](../docs/convolution/README.md)。

6モードで係数列・法／原始根・順／逆変換・実数／複素数・注目する係数を変更できます。ビット反転の交換、長さごとのバタフライ、複素平面の単位根、0埋め、各点積、1/n、3法CRTの混合基数と符号を追います。入力は変換16要素、畳み込み各8要素まで。NTT/CRTはBigInt、FFTはNumberで計算し、表示は小数5桁へ丸めます。LLONG_MIN・保守的上限の拒否・負数・打ち消し・空・1要素を用意しています。

## 3.17 多項式・形式的冪級数（FPS）

- [多項式の加減乗](fps/index.html#polynomial)
- [微積分・係数切り詰め](fps/index.html#calculus)
- [FPS逆数 inv](fps/index.html#inv)
- [FPS対数 log](fps/index.html#log)
- [FPS指数 exp](fps/index.html#exp)
- [FPS累乗 pow](fps/index.html#pow)
- [FPS平方根 sqrt](fps/index.html#sqrt)
- [多項式除算・剰余](fps/index.html#division)
- [多点評価](fps/index.html#multipoint)
- [多項式補間](fps/index.html#interpolation)
- [Taylor Shift](fps/index.html#taylor)
- [Bostan–Mori](fps/index.html#bostan)
- [仕様・使用例・計算量・問題例](../docs/fps/README.md)

12モードで係数、出力精度、指数、移動量、評価点を変更できます。Newton法の補正・確定精度、積木の各多項式と剰余、補間の重み、Bostan–Moriの偶奇抽出を追います。図の積は定義式、除算は筆算を使い、C++のNTTによる高速化は説明へ分けています。BigIntで計算し、先頭0・平方根なし・法で重複する点・uint64の最大値を試せます。

## 3.18 線形漸化式・母関数

- [行列累乗による漸化式](linear_recurrence/index.html#matrix)
- [Kitamasa](linear_recurrence/index.html#kitamasa)
- [Berlekamp–Massey](linear_recurrence/index.html#berlekamp)
- [線形漸化式と有理母関数](linear_recurrence/index.html#bostan)
- [仕様・使用例・計算量・問題例](../docs/linear_recurrence/README.md)

4モードで初期値・係数・法・項番号を変更できます。列ベクトルの状態と行列の二乗、Kitamasaの各還元と重み、BMの不一致と次数変更、母関数P/Qへの変換と偶奇抽出を表示します。小さい任意行列、合成数法・法1、有限観測の推定、先頭0・末尾0、uint64最大値に対応します。

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
node verify/visualizations/tree_model_test.js
node verify/visualizations/tree_browser_test.mjs --browser /path/to/chromium
node verify/visualizations/dp_optimization_model_test.js
node verify/visualizations/dp_optimization_browser_test.mjs --browser /path/to/chromium
node verify/visualizations/number_theory_basic_model_test.js
node verify/visualizations/number_theory_basic_browser_test.mjs --browser /path/to/chromium
node verify/visualizations/number_theory_convolution_model_test.js
node verify/visualizations/number_theory_convolution_browser_test.mjs --browser /path/to/chromium
node verify/visualizations/combinatorics_model_test.js
node verify/visualizations/combinatorics_browser_test.mjs --browser /path/to/chromium
node verify/visualizations/convolution_model_test.js
node verify/visualizations/convolution_browser_test.mjs --browser /path/to/chromium
node verify/visualizations/fps_model_test.js
node verify/visualizations/fps_browser_test.mjs --browser /path/to/chromium
node verify/visualizations/linear_recurrence_model_test.js
node verify/visualizations/linear_recurrence_browser_test.mjs --browser /path/to/chromium
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

3.11のモデルは1720バッチ・34238スナップショットをBFS距離・再帰DFS・全頂点部分集合・独立した閉路走査と比較します。全方位の各メッセージとprefix/suffix、根変更、森・空、負辺と0重み、uint64の境界、スナップショットの独立性を確認します。

3.12のモデルは1,784入力・19,679スナップショットを独立した部分列、編集する文字列のBFS、品物個数、十進接頭辞、隣接結合順、巡回路順列、全長方形と比較します。途中のdp・候補キュー・tight/started・全集合状態・語境界・スナップショットの独立性も確認します。

3.13のモデルは8,529入力・58,563スナップショットを約数・素数・分解の全走査、反復乗算と剰余の周期、Bézoutの恒等式と照合します。全係数・消された候補・SPFの一意な生成・約数の重複なし・64ビット境界・スナップショットの独立性を確認します。

3.14のモデルは4,082入力・70,761スナップショットについて、素因数の集合を制限した定義式から各更新途中の行を求めます。μの定義・全添字対の独立走査、素数／kの方向、添字0、異なる長さ、上限での切り捨て、合成数法、整数境界、スナップショットの独立性も検証します。

3.15のモデルは3,011入力・32,543スナップショット、96個の構築／範囲の拒否条件を確認します。部分集合・順序付き選択・括弧列の列挙、弱合成のDP、要素ごとの部分集合係数から独立に比較します。階乗・逆階乗の全エントリ、Pascalの各行、空集合の呼び出し、負の途中係数、合成数法・法1、スナップショットの独立性も確認します。

3.16のモデルは1,017入力・73,452スナップショットと17個の拒否条件を検証します。有限体／複素数の直接DFT、ビット反転で対応する各部分列のDFT、素朴な多項式積、CRTの剰余と符号を独立に比較します。全バタフライ・各段階・正規化・0埋め・空・極値・スナップショットの独立性も確認します。

3.17のモデルは4,116入力・26,757スナップショット、15拒否条件を検証します。log/expの級数定義・平方根の全候補・多項式の恒等式・全積木・各Newton精度・Bostan–Moriの分母と縮約後の係数を独立に比較し、uint64の極値とスナップショットの独立性も確認します。

3.18のモデルは3,446入力・55,104スナップショット、15拒否条件を検証します。行列の各段階を逐次遷移、Kitamasaの各還元を単位初期値の数列、BMの各処理済み接頭辞の最短次数を独立な連立方程式と照合します。母関数の高次が消える条件・Bostan–Moriの分母と縮約係数、uint64最大値とスナップショットの独立性も確認します。
