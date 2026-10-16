# cp_library 開発の引き継ぎ書

更新日: 2026-10-16。新しいセッションでは、このファイルと対象カテゴリの README を読んでから作業してください。新カテゴリの追加後には、下の「現状」と検証コマンドも更新します。

## ユーザーの方針

競技中に必要な実装を取り出しやすく、詳細を読みたいときには説明と図解へ辿れるライブラリを作っています。AI にカテゴリを順番に依頼し、数回おきに新しいセッションへ切り替える予定です。

- **`3.xx` ごとに独立したカテゴリフォルダを作る。** 新しい実装をすべて `data_structure/` に集めない。
- ヘッダはコピーしやすい短い説明と実装を中心にする。長い `/* ★ Name ... */` 形式の説明を新たに置かない。
- 長い説明は `docs/`、操作できるビジュアル解説は **`docs/` とは別の `visualizations/`** に置く。
- 各ライブラリに説明・usage・API／使用条件・計算量・使用する問題・検証範囲・参考資料を用意する。
- 静止画だけでなく、入力や操作を変えて内部状態を追えるインタラクティブ解説を用意する。実装言語は自由。
- 説明・カテゴリ索引・共通の図解一覧から相互に辿れるようにする。
- 有名ライブラリを適宜調べ、実用的な API と検証を揃える。オンライン AC をしていないものを AC 済みと書かない。

## 作業場所・Git

```text
/home/marc2825/procon/             # IDE のワークスペース
  main.cpp, test.cpp, input.txt …  # ユーザーの問題を解く作業ファイル
  ac-library/                    # 外部 ACL。新実装の実行依存にはしない
  cp_library/                    # このライブラリの Git リポジトリ
```

Git コマンドは `cp_library/` を作業ディレクトリにして実行します。外側のワークスペースの `.git` を操作しません。開始時に必ず `git status --short` と `git branch --show-current` を確認し、既存の未コミット変更を保持します。`main.cpp`、`test.cpp`、入出力ファイルを検証用に上書きしません。

この文書を書いた時点のブランチは **`ai/generated-libraries`** です。

- `7fc6034`: グラフ・木の機能別分離（元ブランチ `refactor/graph-tree-library`）。
- `3bd6a08`: AI 生成ライブラリ用の基準点、既存実装と説明フォルダの準備。
- **3.2〜3.10 の実装・説明・図解・検証と、この引き継ぎ書は、2026-10-16 のスナップショットコミットに収録。** 以降の未コミット変更があれば、次のセッションでも保持する。

この Git 状態は今後変わるため、実際の状態を優先します。commit・push・ブランチ変更は、そのときの依頼の範囲で行います。

## 追加するファイルの構成

```text
<category>/algorithm.hpp
<category>/all.hpp                 # カテゴリのヘッダをまとめる入口
docs/<category>/README.md          # 選び方・API への索引・検証方法
docs/<category>/algorithm.md       # 同名の個別説明
docs/<category>/references.md      # 調査した一次資料、出典、設計判断
examples/<category>/*.cpp          # コンパイルして動く最小例
verify/<category>/unit_test.cpp    # 独立した素朴解との比較
verify/<category>/*.test.cpp       # PROBLEM を指定したジャッジ形式
verify/run_<category>.py           # カテゴリのローカル検証入口
visualizations/<category>/         # インタラクティブ解説
verify/visualizations/             # 図解モデルとブラウザの検証
```

各説明の先頭:

```yaml
---
title: アルゴリズム名
documentation_of: //<category>/algorithm.hpp
---
```

`//` はこの Git リポジトリのルートです。将来の verification-helper 系ドキュメント生成を想定したメタデータです。**公開サイトの自動生成・デプロイはまだ構築していません。** GitHub 上で .hpp を開くだけで .md が自動表示される仕組みは未実装です。

追加時には次の索引も更新します。

1. ルート `README.md` のカテゴリ表・検証案内。
2. `docs/README.md` のカテゴリ一覧。
3. `visualizations/index.html` のカードと `visualizations/README.md`。
4. `Makefile` のカテゴリ検証ターゲット・`verify-visualizations`。
5. このファイルの現状・検証方法。

## C++ の設計方針

新カテゴリは C++17、`namespace cp`、0 始まり・半開区間 `[l,r)` を基本にします。各ヘッダは単独 include 可能、標準ライブラリ以外はリポジトリ内の明示的な include だけにします。`ll`、マクロ、先行 include、`using namespace std` に依存しません。

型・関数名は既存の新カテゴリに揃え、汎用化で使い方を過度に複雑にしないでください。必要な前提、戻り値、空入力や境界の扱い、整数の中間値の範囲を明記します。テンプレート演算の計算量も説明します。

旧 API は移行用ヘッダ／別名を残す方針です。公開フィールドまで使える旧実装は、単純な型別名への置換で壊れることがあります。既存の使用箇所を検索して判断し、変更理由と移行方法を説明します。古いカテゴリすべてを一度に整備する必要はありません。

## 現状

| 分野 | 実装・説明 | 主な注意点 |
|---|---|---|
| グラフ・木 | `graph/`, `tree/`, `docs/graph_tree.md` | 機能別に分離済み。旧 `graph_theory/` は互換入口。新 `Graph` は既定で無向、論理辺 ID を両向きで共有。HLD は方向付き半開区間を返す。基本グラフの図解は 3.7、連結性・Lowlink は 3.8 に整備。木・二重連結分解の図解は未整備。 |
| 3.2 Union-Find | `union_find/`, `docs/union_find/`, `visualizations/union_find/` | 通常版・サイズ／個数管理・重み付き版。重み付き制約は `potential[v]-potential[u]=w`。矛盾は拒否し、merge は enum を返す。通常版 merge は bool。 |
| 3.3 セグメント木・Fenwick | `range_query/`, `docs/range_query/`, `visualizations/range_query/` | Fenwick＋lower_bound、Segtree、Lazy、Dual、区間加算・区間和 BIT。composition(f,g) は g の後に f。 |
| 3.4 静的クエリ・順序統計・集合管理 | `static_query/`, `docs/static_query/`, `visualizations/static_query/` | 今回の S 項目: Sparse Table、Monotonic Stack/Queue、Sliding Window Minimum、SWAG。 |
| 3.5 平衡二分探索木・永続データ構造 | `ordered_structure/`, `docs/ordered_structure/`, `visualizations/ordered_structure/` | 今回の S 項目: std::set/map/multiset の典型操作、座標圧縮＋BIT の Order Statistics。 |
| 3.6 オフラインクエリ・二次元データ構造 | `offline_query/`, `docs/offline_query/`, `visualizations/offline_query/` | Mo、並列二分探索、静的長方形和の Sweep Line、点追加・長方形和の CDQ。 |
| 3.7 グラフ：基本探索・最短路・MST | `graph_basic/`, `docs/graph_basic/`, `visualizations/graph_basic/` | BFS、DFS、0-1、Dijkstra、Bellman–Ford、Warshall–Floyd、トポロジカル順、閉路、Kruskal、Prim、経路復元。 |
| 3.8 グラフ：連結成分・構造分解 | `graph_connectivity/`, `docs/graph_connectivity/`, `visualizations/graph_connectivity/` | SCC・縮約 DAG・Lowlink・橋・関節点・二部グラフ・無向連結成分。 |
| 3.9 グラフ：フロー・マッチング・最適化 | `graph_flow/`, `docs/graph_flow/`, `visualizations/graph_flow/` | Dinic・最小カット復元・Hopcroft–Karp 二部最大マッチング。 |
| 3.10 グラフ：組合せ構造・数え上げ | `graph_combinatorics/`, `docs/graph_combinatorics/`, `visualizations/graph_combinatorics/` | 三角形・全／極大クリーク、最大クリーク／独立集合、厳密彩色 DP、行列木定理。 |

3.3 の補足:

- Fenwick `lower_bound(w)` は最初に累積和が w 以上になる**要素添字**、該当なしは n、w<=0 は 0。全要素非負が探索条件。
- Lazy の長さ付き S では、`LazySegtree(n)` の初期値は e()。ゼロ配列が必要なら `vector<S>(n,{0,1})` 等を渡す。
- Dual は値 S と更新 F を分け、一点 set/get と区間 apply を提供。非可換な更新に対応。
- Segtree/Lazy は ACL の CC0 実装を基に依存を除去・調整。出典とライセンスは `docs/range_query/references.md` に記載。
- 旧 `segtree` は新実装への別名。旧 `BIT` は公開 `n/bit/val` の互換性のため保持。

3.4 の補足:

- Sparse Table は冪等なモノイド専用。空区間は e()、和や一般の関数合成には使わない。build で全体を再構築する。
- MonotonicStack.push は追加前の最近傍の適格な添字を返す。strict=true は < / >、false は ≤ / ≥。見つからない添字は -1。size は候補数。
- 一括の `previous_better_indices` / `next_better_indices` を用意。less/greater で小さい／大きい側を選ぶ。
- MonotonicQueue.pop は論理先頭を削除する。size は論理長、best_index は clear 以降の通算添字。同値は最も左の候補を返す。
- Sliding Window Minimum は値と argmin を提供。k>0 が前提、k>n は空、添字は元配列の位置。
- SWAG は `S,op,e`、prod は左から右の順序。空の積は e()、pop/front はならし O(1)・最悪 O(n)。
- 図解では SWAG の移し替え途中の状態も表示する。その途中では back の残り→front の逆順が元の列になる。新操作は安定した開始／完了位置から行う。

3.5 の補足:

- 標準コンテナを覆う独自クラスは作らず、前後検索・erase_one・pop_first/pop_last の補助関数を提供。検索は該当なしで end() を返す。
- predecessor/successor は比較器順の前／後。既定は厳密、inclusive=true で同値を含む。const・透明な比較器の異種キー検索に対応。
- CoordinateCompression は元の値を保持し、同値を比較器で判定。index(x) は optional、lower/upper_bound は未登録の閾値にも位置を返す。
- CompressedOrderStatistics は事前登録された候補に対する多重集合。insert/erase は bool、erase_all は削除個数、kth は 0 始まりの optional<T>。
- 未登録の更新・負の個数・個数不足・総数の LLONG_MAX 超過は論理状態を変えず拒否。c=0 は登録済みなら成功。問い合わせ閾値は登録不要。
- 3.3 の FenwickTree を再利用。clear は個数だけを消して登録候補を保持する。set の問題では count==0 のときだけ insert する。
- 図解は set/multiset/map/圧縮/順位管理の 5 モード。標準コンテナの内部木は描かず、論理的な並びを示す。

3.6 の補足:

- Mo は `Mo(n)` に `[l,r)` を登録して ID を受け取り、run の answer(id) で元の順へ保存。左右 4 コールバック／共通 2 コールバック、空区間、処理順取得・幅指定に対応。run 前に利用者が空の集計へ初期化する。終了時の状態を空へ戻さない。
- parallel_binary_search は `M,Q,reset,apply,check` を受け取る汎用関数。時刻 t は先頭 t 回の更新適用後。答え 0=初期成立、M+1=該当なし。判定は false→true に単調。毎ラウンド reset、apply(i) は 0 始まり順。Q=0 ならコールバックなし。スレッドは使わない。
- Sweep Line は `static_rectangle_sum(points,rectangles)`。点は `{x,y,weight}`、長方形は `{left,down,right,up}`。x の二境界と圧縮 y の BIT で静的な半開長方形和を解く。
- CDQRectangleSum は add_point/add_query で時刻順に登録し、solve() const で ID 順の答え。過去の点だけを参照し、繰り返し solve・追記・clear に対応。展開イベントを時間で二分し、左右を x 順にマージする。今回入れた BIT 更新だけを巻き戻す。
- 空・逆向きの長方形は 0、同一座標・負の重み可。座標は < による順序のみ、境界 −1 不要。重みは逆元を含む全中間値が収まる加法可換群。
- 座標圧縮（3.5）と Fenwick（3.3）を再利用。`misc/mo_algorithm.hpp` は空の既存ファイルなのでそのまま保持し、新 API は offline_query/mo.hpp に置いた。
- 図解は 4 モード。バッチ入力を変更して履歴を再生成する方式。CDQ の時刻列は 4 prefix に展開した後のイベント列で、途中の答えは部分寄与。入力変更を反映すると常に最初から計算する。

3.7 の補足:

- 基本アルゴリズム本体を graph_basic へ移動、旧 graph/*.hpp は互換入口。共有 Graph / 入力 / 距離の算術補助は graph/ に保持。tree や LowLink の Graph と辺 ID を共用する。
- MST は kruskal.hpp / prim.hpp / spanning_forest.hpp に分離。minimum_spanning_forest.hpp は共通入口。非連結も解き、cost / 採用 Edge / component_count を返す。空の connected() は true。
- 既存 depth_first_search は単一始点の ShortestPathResult<int> を維持。追加 dfs_forest は全成分の parent / parent_edge / depth / component / roots / preorder / postorder / tin / tout を返す。first_root=-1 なら頂点順、それ以外はその根から優先。部分木は preorder[tin[v]..tout[v])。有向 DFS の component は SCC ではない。
- Bellman–Ford を辺列の直接走査へ変更して O(V+VE)。無向辺は両向きを緩和。孤立点が多く負閉路がある場合の全頂点走査を避ける。同距離で選ぶ経路の順は保証しない。
- 距離の結果型は共通。reachable は負閉路の影響下でも true、has_finite_distance は影響下で false。vertex_path / edge_path は復元不可で空、始点自身の頂点列は {s}（有限時のみ）。FW は始点別の結果列。
- 距離最大値は INF 専用。負閉路検出中の値を含め全加算が型に収まることが条件で、範囲外は overflow_error。負閉路影響フラグと数値 dist を区別する。
- 図解は 11 モード。全成分 DFS、キュー/deque/優先候補、FW の全点対行列、負閉路、始点限定の閉路、多重辺、MST、親の復元を表示。入力全体から履歴を再生成する。

3.8 の補足:

- SCC は反復 Kosaraju 法、scc / strongly_connected_components は同じ結果型を返す。component / groups / size / same。成分 ID はトポロジカル順、異なる成分への辺は from<to。各 groups の頂点は昇順、無関係な成分同士の ID 順は保証しない。
- condensation_graph(g,deduplicate=true) は scc / dag / original_edge を返す。内部辺を除外、既定は平行な成分間の辺をまとめ、false なら全て残す。縮約辺の重みは 1。代表元辺 ID は最小 ID 保証なし（成分→頂点順→隣接順で最初の辺）。線形時間の行先マークを使う。
- SCC／縮約は有向用。それ以外は無向用。全て重みを無視し、空・非連結・自己ループ・多重辺に対応。
- Lowlink / bipartite / connected_components 本体を graph_connectivity に移動し、旧 graph パスは互換入口。graph/all.hpp に新カテゴリ全体を含めた。既存の二重辺／頂点連結成分は graph/ に残し、新 Lowlink を経由する。
- bridges / articulation_points は ID／頂点の昇順一覧の補助関数で、その都度 low_link を呼ぶ。両方必要なら一度の low_link 結果を使う。
- 橋は ord[p]<low[v]、根以外の関節点は ord[p]≤low[v]、DFS の根は子が 2 個以上。親頂点でなく親辺 ID の逆向きだけ除外する。孤立点の削除で成分数が減っても関節点ではない。
- bipartite_coloring の成功は optional の 0/1 色列、失敗 nullopt、空グラフは成功した空列。奇閉路の復元 API は含まない。無向 connected_components にも size/same を追加。
- 図解は 7 モード。仮削除では元の ord/low を保持し、色分け・一覧は削除後の連結成分へ切替。C++ の Lowlink を再計算する操作ではない。SCC の逆パスと DAG 重複切替、二部グラフの衝突辺も表示する。

3.9 の補足:

- `Dinic<Cap>` は Graph と別の残余ネットワーク。Cap は bool 以外の整数型、非負容量。元辺ごとに正逆の残余辺を対で持ち、自己ループ・平行辺・逆向き元辺に対応。add_edge は元辺 ID、get_edge / edges は from/to/capacity/flow の値。
- max_flow は今回の追加量を返し、limit の省略値は型の最大値。全体の最大流が型に収まることが条件。最初の呼出し（limit=0 を含む）から reset_flow まで s,t を固定。辺追記は可、容量変更は未提供。reset_flow は容量・ID を保持し、流量と s,t の固定をリセット。
- Dinic は BFS + current arc + 明示的な経路／最小容量スタック。再帰なし。一般容量 O(V²E)、E=0 は O(V)。逆辺を通ると元辺の flow が減る。
- min_cut(s) は現在の残余到達性のみ。`minimum_cut(network,s,t)` は残りも流してから capacity/source_side/edge_ids を返す。network は更新される。容量 0 を含む元辺 S→T のみを ID 昇順で復元。型超過で overflow_error の場合も、それまでに流した分は戻さない。
- `BipartiteMatching(L,R)` は左右別の 0 始まりで辺を登録。maximum_matching() const は毎回空から計算し、cardinality/left_match/right_match/pairs を値で返す。未マッチ -1、pairs は左昇順、相手の辞書順最小は保証しない。辺重複・片側 0・追記可。
- Hopcroft–Karp も反復実装。BFS は全自由左頂点が根、自由右頂点までの最短長を明示し、同じ長さの交互路だけを使う。O((V+E)√(V+1))。一般グラフのマッチングや二部判定は含めない。
- 図解は 3 モード。入力変更から履歴を再生成、逆辺表示・次の増加路／交互路・上限停止と未確定カットを表示。経路の青破線は、フローでは残余逆辺、マッチングでは反転前に外す採用辺。level/distance の -1 は未発見またはその段階で使い切った頂点。

3.10 の補足:

- 共有 Graph を使う無向アルゴリズム。三角形・クリーク・独立集合・彩色は単純グラフが条件、重みを無視。行列木定理だけは多重辺・自己ループを許容（ループは除外）。単純性・向きは assert。
- enumerate_triangles は次数・頂点番号の順に辺を向け、O(N+M√M) 時間、コールバックへ昇順の三頂点を返す。全体の列挙順は未保証。
- enumerate_cliques は番号の昇順で候補を交差させる全列挙で、64頂点制限なし。enumerate_maximal_cliques は64ビットの pivot 付き Bron–Kerbosch。両方 bool(const vector<int>&) のコールバック、空集合は既定で除外、false で打切り。参照を保持せず保存時はコピー。
- 極大は追加できない集合、最大は頂点数が最大の集合。空集合が極大になるのは空グラフのみ。列挙関数は全探索完了で true、コールバック停止で false。
- maximum_clique は貪欲彩色の色数を上界に分枝限定。maximum_independent_set は自己ループを除いた補グラフで同じ探索。64頂点まで、結果は昇順の頂点列。同点解の辞書順保証なし。再帰深さ64以下、最悪指数時間。
- minimum_coloring は厳密 O(3^N) DP、22頂点まで。最小頂点を含む独立集合を一色として除く。非独立集合で2色が見つかったら打切り可。chromatic_number/color/groups を返し、色は0始まり。chromatic_number(g) は数だけの入口。空なら0色。
- 64／22頂点の制限超過は release でも length_error。上限は表現・メモリの制限であり、全ての入力の実行時間保証ではない。指数時間・出力数の制約を説明に明記。
- count_spanning_trees<Mod> は辺 cost を無視して数える。count_weighted_spanning_trees<Mod> は整数 cost の積の和。Mod は int の素数、コンパイル時判定。負・符号なし64ビット重みも正規化。乗算は long long に収まる。removed_vertex=-1 は末尾の行・列。
- 空グラフの全域木数は0、一頂点は1。多重辺は別々、ループは無視。答え0 mod pは非連結を意味しない。determinant_mod<Mod> は long long 正方行列を値で受け、空行列式1、行交換と非零 pivot を使う。
- 図解は6モード。列挙上限・極大のみ・空集合・補グラフ表示、DPと彩色復元、重み・素数mod・除く行列の頂点を編集。行列の乗算は BigInt。消去後は余因子内の行番号を表示し、元ラプラシアンと分ける。

## 検証方法

ライブラリのルートで実行します。Python 検証に追加 pip パッケージは不要です。

```sh
python3 verify/run_graph_tree.py
python3 verify/run_graph_tree.py --sanitize --stress
python3 verify/run_union_find.py
python3 verify/run_union_find.py --sanitize
python3 verify/run_range_query.py
python3 verify/run_range_query.py --sanitize
python3 verify/run_static_query.py
python3 verify/run_static_query.py --sanitize
python3 verify/run_ordered_structure.py
python3 verify/run_ordered_structure.py --sanitize
python3 verify/run_offline_query.py
python3 verify/run_offline_query.py --sanitize
python3 verify/run_graph_basic.py
python3 verify/run_graph_basic.py --sanitize
python3 verify/run_graph_connectivity.py
python3 verify/run_graph_connectivity.py --sanitize
python3 verify/run_graph_flow.py
python3 verify/run_graph_flow.py --sanitize
python3 verify/run_graph_combinatorics.py
python3 verify/run_graph_combinatorics.py --sanitize
make verify-visualizations
```

`--compiler clang++` 等で切り替えられます。C++17 と警告エラーを使い、単独ヘッダ・複数翻訳単位・使用例・素朴解比較・ジャッジ入出力を確認します。ASan/UBSan も実行します。

3.3 追加時には通常検証・ASan/UBSan・両カテゴリの図解モデル検証とブラウザ検証が成功しました。3.3 の説明・図解・索引のローカルリンク 115 件も確認しました。

3.4 追加時にも通常検証・ASan/UBSan・全カテゴリの図解モデル・5 モードのブラウザ検証が成功しました。3.4 の説明・図解・索引のローカルリンク 132 件と、5 つの documentation_of の参照先、Markdown の usage 5 本のコンパイル・実行を確認しています。

3.5 追加時も通常検証・ASan/UBSan・全カテゴリの図解モデル・5 モードのブラウザ検証が成功しました。説明・図解・索引のローカルリンク 130 件、3 つの documentation_of の参照先、Markdown の usage 3 本のコンパイル・実行を確認しています。

3.6 追加時も通常検証・ASan/UBSan・全カテゴリの図解モデル・4 モードのブラウザ検証が成功しました。説明・図解・索引のローカルリンク 164 件、6 つの documentation_of の参照先、Markdown の usage 5 本のコンパイル・実行を確認しています。デスクトップ全モードと 390px 表示の画像も確認しました。

3.7 追加時も通常検証・ASan/UBSan・全カテゴリの図解モデル・11 モードのブラウザ検証が成功しました。Bellman–Ford の改善後に既存グラフ／木／旧 API の ASan/UBSan と 20 万頂点の回帰検証も実行済みです。説明・図解・索引のローカルリンク 265 件、16 の documentation_of、Markdown の usage 15 本をコンパイル・実行し、デスクトップと 390px 表示の画像も確認しました。

3.8 追加時も通常検証・ASan/UBSan・全カテゴリの図解モデル・7 モードのブラウザ検証が成功しました。既存グラフ／木／旧 API／二重連結分解の通常・ASan/UBSan・20 万頂点の回帰検証も実行済みです。説明・図解・索引のローカルリンク 242 件、9 の documentation_of、Markdown の usage 8 本をコンパイル・実行しました。デスクトップ各モード・仮削除・不等式の比較・390px 表示の画像も確認しています。

3.9 追加時も通常検証・ASan/UBSan（debug/release）・全カテゴリの図解モデル・3 モードのブラウザ検証が成功しました。20 万頂点のフロー経路と 20 万左頂点の交互路も検証済みです。説明・図解・索引のローカルリンク 207 件、4 の documentation_of、Markdown の usage 3 本をコンパイル・実行しました。逆辺・交互路・最小カットの画像と 390px の表示を確認し、交差辺のラベルを調整後にもブラウザ検証を実行しています。

3.10 追加時も通常検証・ASan/UBSan（debug/release）・全カテゴリの図解モデル・6 モードのブラウザ検証が成功しました。コールバックの const 参照の契約を反映後も再検証しています。説明・図解・索引のローカルリンク 248 件、8 の documentation_of、Markdown の usage 7 本をコンパイル・実行しました。枝刈り・最小彩色・全域木・390px 表示に加え、DP 表と行交換直後の余因子／元ラプラシアンの画像も確認しています。既存カテゴリとの混在 include と複数翻訳単位でのリンクも通っています。

- グラフ・木: 1,920 ランダムケース、旧 API、深いパス。`--stress` は 20 万頂点。
- 3.2: 併合／制約履歴の全探索、1,050 ランダム系列、20 万要素、3 ジャッジ形式。
- 3.3: 720 ランダム系列・154,400 操作、20 万要素、5 ジャッジ形式を各 1,200 クエリで比較。debug/release 両方で検査する。
- 3.4: 三値配列 9,841 個の全探索、240 FIFO 履歴・120,000 操作、20 万要素、比較／集約回数の線形上限、5 ジャッジ用プログラム。debug/release 両方。
- 3.5: 486 頻度状態の全探索、400 履歴・152,000 更新操作、昇降順・異種キー・独自比較器・LLONG_MAX、20 万座標、4 ジャッジ用プログラム。debug/release 両方。
- 3.6: 1,093 三値配列の全区間、M=0..80 の全成立境界、200 グラフ履歴、300 点更新履歴・48,000 操作、625 格子長方形、極値座標、独自型、20 万要素／状態／操作・50 万 CDQ イベント。6 単独ヘッダ・4 使用例・4 ジャッジ形式を各 600 クエリで比較。debug/release 両方。
- 3.7: 16 ヘッダ・4 使用例・10 ジャッジ形式／250 ケース。600 グラフを歩道 DP・再帰 DFS・順列 DAG・辺部分集合 MSF と比較し、全復元経路の始終点・向き・重みも確認。20 万頂点と 1 辺の負閉路。既存 graph/tree・旧 API の回帰検証も実行。
- 3.8: 9 ヘッダ・4 使用例・5 ジャッジ形式／240 ケース。531 有向グラフ・760 無向多重グラフの全探索、600 ランダムグラフ、20 万頂点。相互到達性、縮約の重複と元辺、辺／頂点削除、low と部分木、全彩色を検証。既存木・旧 API・二重連結分解も回帰検証。
- 3.9: 4 ヘッダ・3 使用例・3 ジャッジ形式／192 ケース。738 容量割当・689 二部グラフの全探索、700 組のランダム入力、全カット列挙・部分集合 DP。流量保存・残余到達・各辺の上限・リセット・容量型の極値／超過、20 万頂点のフロー経路と 20 万左頂点の交互路。debug/release 両方。
- 3.10: 9 ヘッダ（内部補助含む）・4 使用例・6 ジャッジ形式／288 ケース。1,100 単純グラフの全探索、240 ランダム構造、240 多重グラフ／行列。全頂点／辺部分集合・色割当・行列式の全順列と比較。全余因子、負の重み・整数極値・素数2と最大int素数、64ビット・22頂点の境界、Cayley、20万頂点の疎グラフ。debug/release 両方。
- 図解: 3.2 は独立 BFS、3.3 / 3.4 は素朴配列・列の集約によりモデルを検証。3.4 は 189,940 操作、移し替え途中も確認。
- 3.5 の図解: 200 入力・133,200 操作を素朴配列・辞書と照合。失敗時の状態維持と BIT ブロックの値も確認。
- 3.6 の図解: 647 バッチ・53,519 スナップショットを素朴配列・BFS・時刻付き点列と比較。全 BIT ブロック、探索範囲、時刻順、巻き戻し後の 0 を確認。

- 3.7 の図解: 1,042 バッチ・11,140 スナップショットを独立オラクルと比較。距離と負閉路の影響、全成分 DFS、閉路の向き、復元経路、MST の最小性を確認。

- 3.8 の図解: 1,559 ケース・37,101 スナップショットを独立オラクルと照合。SCC、縮約 DAG、根の特例、平行辺、仮削除・彩色と衝突を確認。
- 3.9 の図解: 1,479 バッチ・20,355 スナップショットを全カット列挙・右部分集合 DP と照合。全フレームの流量保存・容量上限・左右の対応、上限停止・逆辺・交互路・カット元辺 ID を確認。
- 3.10 の図解: 1,809 バッチ・34,537 スナップショット。全列挙・極大性・最適性・枝刈り上界・全DP状態・重み付き木・各行消去・行交換・0 mod p・スナップショット独立性を確認。

ブラウザ操作の検証には Node.js 22 以降と Chromium が必要です。npm パッケージは不要です。

```sh
node verify/visualizations/browser_test.mjs --browser /path/to/chromium
node verify/visualizations/range_query_browser_test.mjs --browser /path/to/chromium
node verify/visualizations/static_query_browser_test.mjs --browser /path/to/chromium
node verify/visualizations/ordered_structure_browser_test.mjs --browser /path/to/chromium
node verify/visualizations/offline_query_browser_test.mjs --browser /path/to/chromium
node verify/visualizations/graph_basic_browser_test.mjs --browser /path/to/chromium
node verify/visualizations/graph_connectivity_browser_test.mjs --browser /path/to/chromium
node verify/visualizations/graph_flow_browser_test.mjs --browser /path/to/chromium
node verify/visualizations/graph_combinatorics_browser_test.mjs --browser /path/to/chromium
```

`--screenshot /tmp/example.png` でデスクトップ・モバイルの画像を保存できます。全モードの操作・再生・入力エラー・ローカルリンク・390px の横幅を確認します。

この環境では、サンドボックス内の LeakSanitizer が ptrace 制限、Chromium がソケット制限で失敗する場合がありました。その場合はエラーを確認し、環境の承認手順で同じチェックをサンドボックス外で実行します。検証機能を無効にして成功扱いにしません。以前のセッションで作った `/tmp/` のブラウザ補助ファイルは永続依存にしません。

## 図解の追加・確認

現在はローカルで直接開ける HTML/CSS/JavaScript/SVG を使っています。外部通信・CDN・npm 導入なしで動作します。共有 CSS は `visualizations/assets/style.css` です。モデルと画面コードを分け、モデルは Node.js でも実行できます。

入力変更、操作の実行、前後のステップ、再生、サンプル操作列を基本とします。どのノードが参照／変更されたか、何を保持するか、各ステップの理由が分かるようにします。内部状態を簡略化する場合は、その範囲を明記してください。

モデルの期待値は内部状態から再計算するだけで済ませず、独立した素朴解と比較します。更新途中の一時的に不整合な状態から別操作を始めないようにし、操作完了まで進める等の扱いを設けます。スマホではページ全体の横はみ出しを避け、必要なら図の領域だけをスクロールさせます。

## 参考にするライブラリ

必要な分野の一次資料を選んで実際に読み、`references.md` に参考箇所と設計の違いを残します。すべてのライブラリを毎回調べる必要はありません。

[ACL](https://github.com/atcoder/ac-library)、[Luzhiled](https://ei1333.github.io/library/)、[Nyaan](https://nyaannyaan.github.io/library/)、[maspypy](https://maspypy.github.io/library/)、[suisen](https://suisen-cp.github.io/cp-library-cpp/)、[hitonanode](https://hitonanode.github.io/cplib-cpp/)、[sotanishy](https://sotanishy.github.io/cp-library-cpp/)、[noshi91](https://noshi91.github.io/Library/)、[kmyk](https://kmyk.github.io/competitive-programming-library/)、[KACTL](https://github.com/kth-competitive-programming/kactl)、[CP-Algorithms](https://cp-algorithms.com/)、[Library Checker](https://judge.yosupo.jp/)。

コードを取り込む場合はライセンスと出典を確認して保持します。説明文はこの API に合わせて書きます。

## 新しいセッションへの依頼テンプレート

以下のカテゴリ部分を差し替えて使えます。

```text
cp_library/HANDOFF.md を読んで、現在の変更とブランチを確認してから続けてください。
既存の変更を保持し、次のカテゴリを実装してください。

### 3.x カテゴリ名
- S: 実装してほしい項目...

3.xx ごとに独立した実装フォルダにしてください。
Markdown の説明・usage・計算量・問題例に加え、docs/ とは別の
visualizations/ に操作できる図解も用意し、各索引からリンクしてください。
適切な検証を実行し、完了後に HANDOFF.md の現状も更新してください。
```
