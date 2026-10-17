# 実装予定リスト（3.1〜3.25）

更新日: 2026-10-17。ユーザー提供の全分野リストをS/A/B/Cの分類ごと保存し、A項目だけ現在の実装と照合しました。[引継書](../HANDOFF.md) / [カテゴリ索引](README.md) / [先に追加した高度な20項目](advanced/README.md)。

## 今後の進め方

**次の開発は、未実装のA項目と一部対応の残りを優先します。** 3.1から3.25へカテゴリ順に進め、同じカテゴリでは下の掲載順を基本にします。依存する部品や重複項目はまとめて処理し、実装済みのものは再実装しません。S/B/Cは全体計画として残しますが、ユーザーから別の指定がない限り、Aの補完を先に進めます。

3.1〜3.6のAは47掲載箇所すべて実装済みです（今回の補完32箇所）。現在の最初の不足Aは **3.7「Radix Heap Dijkstra」の統合**、続いて「Dial's Algorithm」「DAG最短路」です。3.1の新しい共通部品は `basic_utility/` に置きました。ダブリング・座標圧縮・Cartesian Tree・XOR基底など他カテゴリに実体がある機能は参照し、移動や複製を前提にしません。

実装前にはこの文書の状態と実ファイルを再確認し、空ファイル・雛形・WIP・内部専用部品を完成済みの公開APIと取り違えないでください。「一部対応」は注記した不足分だけを補います。コードが同じ役割を既に満たす場合は、薄い入口・説明・使用例の整備で対応できます。

1. 対象の既存実装・説明・検証を読み、必要な機能と配置先を決める。
2. C++17・`namespace cp`・0始まり／半開区間の規約に合わせ、旧APIを保持して不足分を実装する。
3. 説明・usage・API条件・計算量・問題例・出典、動く例、独立した素朴解との比較検証、操作できる図解を揃える。
4. 適切な通常／サニタイザ検証を実行し、各カテゴリのall.hpp・索引・Makefile・このリスト・引継書を更新する。オンラインACとローカル検証を区別する。

## 状態の読み方

- `[x] 実装あり`：該当する公開機能を確認。今回、全項目を再実行して検証したという意味ではありません。
- `[x] 旧実装あり`：既存APIで機能を確認。新APIへの移行や説明の不足と、アルゴリズムの未実装を区別します。
- `[ ] 一部対応`：関連する核はあるものの、注記した機能・公開入口・組合せの検証などを補う必要があります。
- `[ ] 未実装`：現状のコードから対応する完成機能を確認できません。再開時にもう一度検索します。
- S/B/Cの列挙には状態を付けていません。未実装という意味ではなく、今回はAを追跡対象にしています。

同じ項目が複数カテゴリに載っているため、件数は**掲載箇所単位**です。現在のAは169箇所（実装あり113、旧実装あり3、一部対応10、未実装43）。チェックは実装あり／旧実装ありだけに付けています。

分類は提供リストを維持します。実体の配置は既存構成を優先し、Miller–Rabin／Pollard's Rhoは3.14、永続セグメント木は3.5、Li Chaoは3.12、集合変換は3.19にある実装を参照します。原稿にある検索結果のサイト名・「+2」等の表示断片だけを除き、項目と補足は保持しました。

## カテゴリ別の進捗

| 分野 | 主な配置先 | A掲載数 | 実装あり（旧含む） | 一部対応 | 未実装 |
|---|---|---:|---:|---:|---:|
| [3.1 基本ユーティリティ・探索](#3-1) | `basic_utility/` | 11 | 11 | 0 | 0 |
| [3.2 Union-Find・連結性管理](#3-2) | `union_find/` | 7 | 7 | 0 | 0 |
| [3.3 セグメント木・Fenwick Tree](#3-3) | `range_query/` | 7 | 7 | 0 | 0 |
| [3.4 静的クエリ・順序統計・集合管理](#3-4) | `static_query/` | 7 | 7 | 0 | 0 |
| [3.5 平衡二分探索木・永続データ構造](#3-5) | `ordered_structure/` | 7 | 7 | 0 | 0 |
| [3.6 オフラインクエリ・二次元データ構造](#3-6) | `offline_query/` | 8 | 8 | 0 | 0 |
| [3.7 グラフ：基本探索・最短路・MST](#3-7) | `graph_basic/` | 8 | 0 | 1 | 7 |
| [3.8 グラフ：連結成分・構造分解](#3-8) | `graph_connectivity/` | 8 | 4 | 0 | 4 |
| [3.9 グラフ：フロー・マッチング・最適化](#3-9) | `graph_flow/` | 6 | 4 | 0 | 2 |
| [3.10 グラフ：組合せ構造・数え上げ](#3-10) | `graph_combinatorics/` | 6 | 6 | 0 | 0 |
| [3.11 木アルゴリズム](#3-11) | `tree/` | 8 | 7 | 0 | 1 |
| [3.12 DP・最適化](#3-12) | `dp_optimization/` | 9 | 3 | 3 | 3 |
| [3.13 数論：基本整数演算](#3-13) | `number_theory_basic/` | 10 | 3 | 3 | 4 |
| [3.14 数論：乗法的関数・高度な整数問題](#3-14) | `number_theory_convolution/` | 5 | 5 | 0 | 0 |
| [3.15 組合せ論・数え上げ](#3-15) | `combinatorics/` | 8 | 2 | 0 | 6 |
| [3.16 畳み込み・高速変換](#3-16) | `convolution/` | 6 | 3 | 0 | 3 |
| [3.17 多項式・形式的冪級数（FPS）](#3-17) | `fps/` | 10 | 10 | 0 | 0 |
| [3.18 線形漸化式・母関数](#3-18) | `linear_recurrence/` | 4 | 4 | 0 | 0 |
| [3.19 集合関数・Bitmask変換](#3-19) | `set_function/` | 3 | 1 | 1 | 1 |
| [3.20 線形代数](#3-20) | `linear_algebra/` | 6 | 2 | 1 | 3 |
| [3.21 文字列アルゴリズム](#3-21) | `string_algorithm/` | 6 | 2 | 0 | 4 |
| [3.22 計算幾何](#3-22) | `computational_geometry/` | 7 | 7 | 0 | 0 |
| [3.23 組合せ最適化・マトロイド](#3-23) | `combinatorial_optimization/` | 4 | 4 | 0 | 0 |
| [3.24 ゲーム理論](#3-24) | `game_theory/` | 3 | 1 | 0 | 2 |
| [3.25 探索・ヒューリスティック](#3-25) | `heuristic_search/` | 5 | 1 | 1 | 3 |

## 重複項目・取り違えやすい箇所

- Mo on Tree（3.6／3.11）、Functional Graph（3.7／3.8）、Cartesian Tree（3.1／3.11）、集合変換・畳み込み（3.16／3.19）、フロー・割当（3.9／3.23）は共通実装を使い、関係するチェックをまとめて更新します。
- Persistent Segtreeのpoint get/setは永続配列として利用できます。一方、永続Stack／QueueやRollback DSUは永続Union-Findの代用ではありません。
- 任意mod畳み込みの内部CRTは、一般の合同式を解くCRT／Garnerの公開APIとは別です。Subset Convolution内部のrank別変換も、公開Ranked Zeta APIとは別です。
- Implicit Treapと汎用作用付きLazy Reversible Treapは別APIとして整備済みです。既存Beam Searchは重複を除去する契約ではありません。
- 新二次元Segtreeはrange_query/segtree_2d.hpp。旧 `data_structure/2D_segtree.hpp` はWIPのまま保持。`graph_theory/funtional_graph.hpp` は説明雛形です。存在するだけで実装済みにしません。
- 行列式・階数・Lagrange補間・線形篩・ゲームの直和など、他カテゴリや既存APIの返り値で既に満たせるものは再実装しません。追加の高速化や対象型の拡張は別の要件として扱います。

## 全実装予定リスト

<a id="3-1"></a>

### 3.1 基本ユーティリティ・探索

主な配置先: `basic_utility/`。

- **S**：二分探索、三分探索、座標圧縮、累積和、二次元累積和、いもす法、二次元いもす法、差分配列、ランレングス圧縮、転倒数、床・天井除算、整数平方根、整数累乗、chmin/chmax

**A（優先実装）**

- [x] ダブリング — **実装あり**。 [tree/binary_lifting.hpp](../tree/binary_lifting.hpp)。 終端・循環を含む遷移の反復。
- [x] 循環列の操作 — **実装あり**。 [basic_utility/cyclic_sequence.hpp](../basic_utility/cyclic_sequence.hpp)。 [説明](basic_utility/cyclic_sequence.md)。
- [x] 区間のマージ・分割 — **実装あり**。 [basic_utility/interval_operations.hpp](../basic_utility/interval_operations.hpp)。 [説明](basic_utility/interval_operations.md)。
- [x] 商列挙 — **実装あり**。 [basic_utility/quotient_enumeration.hpp](../basic_utility/quotient_enumeration.hpp)。 [説明](basic_utility/quotient_enumeration.md)。
- [x] 倍数・約数列挙 — **実装あり**。 [basic_utility/multiple_enumeration.hpp](../basic_utility/multiple_enumeration.hpp)。 [説明](basic_utility/multiple_enumeration.md)。約数は既存のnumber_theory_basic/divisors.hppを共有。
- [x] Gray Code — **実装あり**。 [basic_utility/gray_code.hpp](../basic_utility/gray_code.hpp)。 [説明](basic_utility/gray_code.md)。
- [x] 部分集合列挙 — **実装あり**。 [set_function/submask_enumeration.hpp](../set_function/submask_enumeration.hpp)。 全体maskのsubmask列挙で部分集合を列挙できる。
- [x] 部分集合の部分集合列挙 — **実装あり**。 [basic_utility/subset_pairs.hpp](../basic_utility/subset_pairs.hpp)。 [説明](basic_utility/subset_pairs.md)。既存for_each_submaskを共有し全(S,T⊆S)を列挙。
- [x] 順列のランク・アンランク — **実装あり**。 [basic_utility/permutation_rank.hpp](../basic_utility/permutation_rank.hpp)。 [説明](basic_utility/permutation_rank.md)。
- [x] Cartesian Tree — **旧実装あり**。 [data_structure/cartesian_tree.hpp](../data_structure/cartesian_tree.hpp)。 旧API。再実装せず必要なら移行入口・説明・検証だけを補う。
- [x] XOR基底 — **実装あり**。 [linear_algebra/xor_basis.hpp](../linear_algebra/xor_basis.hpp)。

- **B**：Fractional Cascading、整数 k 乗根、Stern–Brocot Tree、分数二分探索、平方分割テンプレート、非単調関数の探索、連続区間集合の管理

- **C**：部分永続の探索補助、Retroactiveな操作管理、特殊な整数列挙

補足：日常的に書き直す処理を最優先で共通化します。

<a id="3-2"></a>

### 3.2 Union-Find・連結性管理

主な配置先: `union_find/`。

- **S**：通常のUnion-Find、重み付きUnion-Find（Potentialized DSU）、連結成分サイズ・個数管理

**A（優先実装）**

- [x] Undo/Rollback Union-Find — **実装あり**。 [union_find/rollback_disjoint_set_union.hpp](../union_find/rollback_disjoint_set_union.hpp)。
- [x] 部分永続Union-Find — **実装あり**。 [union_find/partially_persistent_disjoint_set_union.hpp](../union_find/partially_persistent_disjoint_set_union.hpp)。 [説明](union_find/partially_persistent_disjoint_set_union.md)。
- [x] 完全永続Union-Find — **実装あり**。 [union_find/persistent_disjoint_set_union.hpp](../union_find/persistent_disjoint_set_union.hpp)。 [説明](union_find/persistent_disjoint_set_union.md)。
- [x] 連結成分のモノイド集約 — **実装あり**。 [union_find/aggregate_disjoint_set_union.hpp](../union_find/aggregate_disjoint_set_union.hpp)。 [説明](union_find/aggregate_disjoint_set_union.md)。
- [x] 二部グラフ判定付きUnion-Find — **実装あり**。 [union_find/bipartite_disjoint_set_union.hpp](../union_find/bipartite_disjoint_set_union.hpp)。 [説明](union_find/bipartite_disjoint_set_union.md)。
- [x] 区間Union-Find — **実装あり**。 [union_find/interval_disjoint_set_union.hpp](../union_find/interval_disjoint_set_union.hpp)。 [説明](union_find/interval_disjoint_set_union.md)。
- [x] マージ履歴木（Merge Tree） — **実装あり**。 [union_find/merge_tree.hpp](../union_find/merge_tree.hpp)。 [説明](union_find/merge_tree.md)。

- **B**：Offline Dynamic Connectivity、Parallel Union-Find、動的Union-Find、加算などの操作付きDSU、Incremental Bridge Connectivity

- **C**：非可換群上のPotentialized DSU、一般的なFully Dynamic Connectivity

補足：特に Rollback DSU + 時間軸セグ木 は、削除を含む連結性クエリのためにセットで用意したい構成です。

<a id="3-3"></a>

### 3.3 セグメント木・Fenwick Tree

主な配置先: `range_query/`。

- **S**：Fenwick Tree、Segtree、Lazy Segtree、Dual Segtree、Fenwick Treeの lower_bound、区間加算・区間和BIT

**A（優先実装）**

- [x] Dynamic Segtree — **実装あり**。 [range_query/dynamic_segtree.hpp](../range_query/dynamic_segtree.hpp)。 [説明](range_query/dynamic_segtree.md)。
- [x] Dynamic Lazy Segtree — **実装あり**。 [range_query/dynamic_lazy_segtree.hpp](../range_query/dynamic_lazy_segtree.hpp)。 [説明](range_query/dynamic_lazy_segtree.md)。
- [x] Persistent Segtree — **実装あり**。 [ordered_structure/persistent_segment_tree.hpp](../ordered_structure/persistent_segment_tree.hpp)。 区間k番目は ordered_structure/persistent_order_statistics.hpp。
- [x] Segment Tree Beats — **実装あり**。 [range_query/segment_tree_beats.hpp](../range_query/segment_tree_beats.hpp)。 chmin/chmax/addとmin/max/sum。原稿推奨のassignはchmin→chmaxで合成可能、専用range_assign APIはない。
- [x] 二次元Fenwick Tree — **実装あり**。 [range_query/fenwick_tree_2d.hpp](../range_query/fenwick_tree_2d.hpp)。 [説明](range_query/fenwick_tree_2d.md)。
- [x] 二次元Segtree — **実装あり**。 [range_query/segtree_2d.hpp](../range_query/segtree_2d.hpp)。 [説明](range_query/segtree_2d.md)。旧WIPは保持し、新しい完成APIを追加。
- [x] Range Add/Assign/Affine のLazy Segtree用モノイド — **実装あり**。 [range_query/range_actions.hpp](../range_query/range_actions.hpp)。 [説明](range_query/range_actions.md)。

- **B**：Sparse Dynamic Segtree、Rollback Segtree、Persistent Lazy Segtree、Sortable Segtree、Range Tree、Segtree with Fractional Cascading、2D Dual Segtree

- **C**：Kinetic Segment Tree、RBST-based Segment Tree、複雑な範囲変換をサポートするBeats

補足：最も実用性の高い分野の一つです。

Segment Tree Beats は range chmin/chmax/add/assign と range min/max/sum を扱うものを一つ用意すると、多くの問題に対応できます。

参考：ei1333、maspypy、sotanishy。

<a id="3-4"></a>

### 3.4 静的クエリ・順序統計・集合管理

主な配置先: `static_query/`。

- **S**：Sparse Table、Monotonic Stack、Monotonic Queue、Sliding Window Minimum、SWAG（Sliding Window Aggregation）

**A（優先実装）**

- [x] Disjoint Sparse Table — **実装あり**。 [static_query/disjoint_sparse_table.hpp](../static_query/disjoint_sparse_table.hpp)。 [説明](static_query/disjoint_sparse_table.md)。
- [x] Wavelet Matrix — **実装あり**。 [static_query/wavelet_matrix.hpp](../static_query/wavelet_matrix.hpp)。
- [x] Binary Trie — **実装あり**。 [static_query/binary_trie.hpp](../static_query/binary_trie.hpp)。 [説明](static_query/binary_trie.md)。
- [x] Li Chao Tree — **実装あり**。 [dp_optimization/li_chao_tree.hpp](../dp_optimization/li_chao_tree.hpp)。
- [x] Convex Hull Trick — **旧実装あり**。 [data_structure/ConvexHullTrick.hpp](../data_structure/ConvexHullTrick.hpp)。 旧API。再実装せず必要なら移行入口・説明・検証だけを補う。
- [x] Fast Set / Predecessor Set — **実装あり**。 [static_query/fast_set.hpp](../static_query/fast_set.hpp)。 [説明](static_query/fast_set.md)。
- [x] Ordered Set — **実装あり**。 [ordered_structure/ordered_set.hpp](../ordered_structure/ordered_set.hpp)。 [説明](ordered_structure/ordered_set.md)。3.4のstatic_query/ordered_set.hppは共通入口。

- **B**：Wavelet Matrixの区間和拡張、Wavelet Tree、Persistent Binary Trie、Priority Sum（上位・下位k個の和）、Dynamic Bitset、Succinct Bit Vector、Linear RMQ、Sqrt Tree

- **C**：Static Range Mode、Static Range LIS、静的転倒数クエリ、動的Wavelet Matrix、特殊なRank/Select構造

補足：Wavelet Matrixは、rank、kth_smallest、range_freq、predecessor/successor を一つのAPIにまとめるのがよいです。

<a id="3-5"></a>

### 3.5 平衡二分探索木・永続データ構造

主な配置先: `ordered_structure/`。

- **S**：std::set/map と multiset を使った典型操作、座標圧縮とBITによるOrder Statistics

**A（優先実装）**

- [x] Treap — **実装あり**。 [ordered_structure/treap.hpp](../ordered_structure/treap.hpp)。 [説明](ordered_structure/treap.md)。
- [x] RBST — **実装あり**。 [ordered_structure/rbst.hpp](../ordered_structure/rbst.hpp)。 [説明](ordered_structure/rbst.md)。
- [x] Implicit Treap — **実装あり**。 [ordered_structure/implicit_treap.hpp](../ordered_structure/implicit_treap.hpp)。
- [x] Lazy Reversible Treap — **実装あり**。 [ordered_structure/lazy_reversible_treap.hpp](../ordered_structure/lazy_reversible_treap.hpp)。 [説明](ordered_structure/lazy_reversible_treap.md)。
- [x] 永続配列 — **実装あり**。 [ordered_structure/persistent_segment_tree.hpp](../ordered_structure/persistent_segment_tree.hpp)。 get/setと版の分岐で永続配列として利用可能。専用ラッパを重複実装する必要はない。
- [x] 永続Stack — **実装あり**。 [ordered_structure/persistent_stack.hpp](../ordered_structure/persistent_stack.hpp)。 [説明](ordered_structure/persistent_stack.md)。
- [x] 永続Queue — **実装あり**。 [ordered_structure/persistent_queue.hpp](../ordered_structure/persistent_queue.hpp)。 [説明](ordered_structure/persistent_queue.md)。

- **B**：Splay Tree、Red-Black Tree、Weight-Balanced Tree、Lazy Balanced BST、永続平衡BST、Skew Heap、Leftist Heap、Radix Heap、Erasable Priority Queue

- **C**：Finger Tree、Partially Retroactive Priority Queue、完全永続Lazy Balanced BST

補足：TreapやRBSTは「値の集合」と「暗黙キーによる列操作」を分けて実装すると扱いやすいです。

<a id="3-6"></a>

### 3.6 オフラインクエリ・二次元データ構造

主な配置先: `offline_query/`。

- **S**：Mo's Algorithm、Parallel Binary Search（並列二分探索）、Sweep Line、CDQ Divide and Conquerの基本形

**A（優先実装）**

- [x] Mo with Updates（時空間Mo） — **実装あり**。 [offline_query/mo_with_updates.hpp](../offline_query/mo_with_updates.hpp)。
- [x] Mo on Tree — **実装あり**。 [offline_query/mo_on_tree.hpp](../offline_query/mo_on_tree.hpp)。 [説明](offline_query/mo_on_tree.md)。
- [x] Rollback Mo — **実装あり**。 [offline_query/rollback_mo.hpp](../offline_query/rollback_mo.hpp)。 [説明](offline_query/rollback_mo.md)。
- [x] 二次元BIT（座圧版） — **実装あり**。 [offline_query/compressed_fenwick_tree_2d.hpp](../offline_query/compressed_fenwick_tree_2d.hpp)。 [説明](offline_query/compressed_fenwick_tree_2d.md)。
- [x] Static Rectangle Sum — **実装あり**。 [offline_query/sweep_line.hpp](../offline_query/sweep_line.hpp)。
- [x] Dynamic Point Add Rectangle Sum — **実装あり**。 [offline_query/cdq_rectangle_sum.hpp](../offline_query/cdq_rectangle_sum.hpp)。 登録操作列に対するオフライン処理。オンライン2D構造は別項目。
- [x] Range Set — **実装あり**。 [offline_query/range_set.hpp](../offline_query/range_set.hpp)。 [説明](offline_query/range_set.md)。
- [x] Offline RMQ — **実装あり**。 [offline_query/offline_rmq.hpp](../offline_query/offline_rmq.hpp)。 [説明](offline_query/offline_rmq.md)。

- **B**：長方形加算・長方形和、長方形加算・点取得、Kd-Tree、Range Tree、Hilbert Order Mo、Static Range Count Distinct、Dynamic Rectangle Affine Sum

- **C**：三次元以上の直交範囲クエリ、特殊な動的最近傍探索、汎用多次元Fenwick Tree

補足：これらはmaspypy、ei1333のライブラリで特に豊富です。

<a id="3-7"></a>

### 3.7 グラフ：基本探索・最短路・MST

主な配置先: `graph_basic/`。

- **S**：BFS、DFS、0-1 BFS、Dijkstra、Bellman–Ford、Warshall–Floyd、トポロジカルソート、閉路検出、Kruskal、Prim、経路復元

**A（優先実装）**

- [ ] Radix Heap Dijkstra — **一部対応**。 [data_structure/radix_heap_iwiwi.hpp](../data_structure/radix_heap_iwiwi.hpp)。 Radix Heap本体のみ存在。経路復元対応のDijkstraとの統合は未実装。
- [ ] Dial's Algorithm — **未実装**。
- [ ] DAG最短路 — **未実装**。
- [ ] Johnson's Algorithm — **未実装**。
- [ ] K-th Shortest Path — **未実装**。
- [ ] K-th Shortest Walk — **未実装**。
- [ ] 補グラフBFS — **未実装**。
- [ ] Functional Graph — **未実装**。 [graph_theory/funtional_graph.hpp](../graph_theory/funtional_graph.hpp)。 graph_theory/funtional_graph.hppは説明雛形のみ。BinaryLiftingだけでは成分・閉路分解を満たさない。

- **B**：Directed MST（Chu–Liu/Edmonds）、Manhattan MST、最短有向閉路、最短無向閉路、最小全域木の感度解析、全域木の第二最小コスト

- **C**：高速な特殊重み最短路、グラフ制約付きShortest Nonzero Path、Minimum Diameter Spanning Tree

補足：最短路は経路復元対応版を最初から作成するのがおすすめです。

<a id="3-8"></a>

### 3.8 グラフ：連結成分・構造分解

主な配置先: `graph_connectivity/`。

- **S**：SCC（強連結成分）、Lowlink、橋・関節点、二部グラフ判定

**A（優先実装）**

- [x] 二重辺連結成分 — **実装あり**。 [graph/two_edge_connected_components.hpp](../graph/two_edge_connected_components.hpp)。
- [x] 二重頂点連結成分 — **実装あり**。 [graph/biconnected_components.hpp](../graph/biconnected_components.hpp)。
- [ ] Block-Cut Tree — **未実装**。
- [x] Bridge Tree — **実装あり**。 [graph/two_edge_connected_components.hpp](../graph/two_edge_connected_components.hpp)。 返り値forestが橋の縮約木。
- [ ] 2-SAT — **未実装**。
- [ ] Eulerian Trail — **未実装**。
- [x] サイクル復元 — **実装あり**。 [graph_basic/cycle.hpp](../graph_basic/cycle.hpp)。 頂点列と辺ID列を復元。
- [ ] Functional Graph分解 — **未実装**。 3.7のFunctional Graphと共通実装する。

- **B**：Dominator Tree、三重辺連結成分、Cactus Graph分解、Chordal Graph判定、Tree Decomposition（幅2）、Incremental SCC、DAG Reachability

- **C**：SPQR Tree、一般グラフの木幅分解、特殊なグラフの標準形

<a id="3-9"></a>

### 3.9 グラフ：フロー・マッチング・最適化

主な配置先: `graph_flow/`。

- **S**：Dinic最大流、最小カット復元、二部最大マッチング

**A（優先実装）**

- [x] 最小費用流（Primal-Dual） — **実装あり**。 [combinatorial_optimization/min_cost_flow.hpp](../combinatorial_optimization/min_cost_flow.hpp)。
- [x] Hopcroft–Karp — **実装あり**。 [graph_flow/bipartite_matching.hpp](../graph_flow/bipartite_matching.hpp)。
- [x] Hungarian Algorithm（割当問題） — **実装あり**。 [combinatorial_optimization/hungarian.hpp](../combinatorial_optimization/hungarian.hpp)。
- [ ] 下限制約付きフロー — **未実装**。
- [x] 一般グラフの最大マッチング（Blossom） — **実装あり**。 [combinatorial_optimization/general_matching.hpp](../combinatorial_optimization/general_matching.hpp)。
- [ ] Project Selection Problem（燃やす埋める） — **未実装**。

- **B**：Min-Cost b-Flow、一般グラフの最小カット（Stoer–Wagner）、Push-Relabel、最小費用循環流、DM分解、二部グラフ辺彩色、最大重みマッチング

- **C**：一般グラフの最大重みマッチング（Blossomの重み付き版）、Rank-Maximal Matching、特殊な動的フロー

<a id="3-10"></a>

### 3.10 グラフ：組合せ構造・数え上げ

主な配置先: `graph_combinatorics/`。

**A（優先実装）**

- [x] 三角形列挙 — **実装あり**。 [graph_combinatorics/triangle_enumeration.hpp](../graph_combinatorics/triangle_enumeration.hpp)。
- [x] クリーク列挙 — **実装あり**。 [graph_combinatorics/clique_enumeration.hpp](../graph_combinatorics/clique_enumeration.hpp)。
- [x] 最大クリーク — **実装あり**。 [graph_combinatorics/maximum_clique.hpp](../graph_combinatorics/maximum_clique.hpp)。
- [x] 最大独立集合 — **実装あり**。 [graph_combinatorics/maximum_independent_set.hpp](../graph_combinatorics/maximum_independent_set.hpp)。
- [x] 彩色数 — **実装あり**。 [graph_combinatorics/chromatic_number.hpp](../graph_combinatorics/chromatic_number.hpp)。
- [x] 全域木の個数（行列木定理） — **実装あり**。 [graph_combinatorics/matrix_tree.hpp](../graph_combinatorics/matrix_tree.hpp)。

- **B**：彩色多項式、C4の個数、極大クリーク列挙、Steiner Tree DP、DAGの最小Path Cover、辺素・頂点素パス、グラフ同型の簡易判定

- **C**：ラベル付き連結グラフ数、強連結有向グラフ数、Euler閉路数（BEST定理）、一般的なグラフ同型、特殊なグラフ分解

補足：ei1333、hitonanode、maspypyには、一般マッチングや有向MST、彩色数などの実装があります。Library Checkerにもこれらの検証問題が用意されています。

<a id="3-11"></a>

### 3.11 木アルゴリズム

主な配置先: `tree/`。

- **S**：LCA、木の直径、Euler Tour、部分木サイズ・深さ、全方位木DP（Rerooting）、Binary Lifting、Tree DPの基本テンプレート

**A（優先実装）**

- [x] Heavy-Light Decomposition — **実装あり**。 [tree/heavy_light_decomposition.hpp](../tree/heavy_light_decomposition.hpp)。
- [x] Virtual Tree（Auxiliary Tree） — **実装あり**。 [tree/virtual_tree.hpp](../tree/virtual_tree.hpp)。
- [x] Centroid Decomposition — **実装あり**。 [tree/centroid_decomposition.hpp](../tree/centroid_decomposition.hpp)。
- [x] DSU on Tree — **実装あり**。 [tree/dsu_on_tree.hpp](../tree/dsu_on_tree.hpp)。
- [ ] 木のハッシュ — **未実装**。
- [x] Cartesian Tree — **旧実装あり**。 [data_structure/cartesian_tree.hpp](../data_structure/cartesian_tree.hpp)。 3.1と同じ実装。重複実装しない。
- [x] 木上のPath Query — **実装あり**。 [tree/heavy_light_decomposition.hpp](../tree/heavy_light_decomposition.hpp)。 方向付き区間をSegtree等へ渡す既存APIを使用。examples/tree_hld.cppも参照。
- [x] 木上のMo — **実装あり**。 [offline_query/mo_on_tree.hpp](../offline_query/mo_on_tree.hpp)。 [説明](offline_query/mo_on_tree.md)。3.6と共通実装。

- **B**：Static Top Tree、Link-Cut Tree、Euler Tour Tree、動的全方位木DP、木の同型性判定、木上の距離分布、Contour Query、木の直径の動的管理、木のマージ過程

- **C**：Dynamic Top Tree、Lazy Top Tree、動的部分木集約、Fully Dynamic Tree DP

補足：HLD + セグ木 と LCA + Virtual Tree は、組み合わせとして検証しておくと実戦で使いやすくなります。

<a id="3-12"></a>

### 3.12 DP・最適化

主な配置先: `dp_optimization/`。

- **S**：LIS、LCS、編集距離、0-1/完全/個数制限ナップサック、Bitset Knapsack、区間DP、桁DP、部分集合DP、Largest Rectangle

**A（優先実装）**

- [ ] Divide and Conquer DP Optimization — **一部対応**。 [dp_optimization/monotone_minima.hpp](../dp_optimization/monotone_minima.hpp)。 各段の遷移行列探索は対応済み。DP段更新・解復元を含む専用入口は未実装。
- [ ] Knuth Optimization — **未実装**。
- [x] Monotone Minima — **実装あり**。 [dp_optimization/monotone_minima.hpp](../dp_optimization/monotone_minima.hpp)。
- [x] SMAWK — **実装あり**。 [dp_optimization/smawk.hpp](../dp_optimization/smawk.hpp)。
- [ ] Convex Hull Trick DP — **一部対応**。 [data_structure/ConvexHullTrick.hpp](../data_structure/ConvexHullTrick.hpp)。 CHT本体は存在。DP遷移への組込みと比較検証・使用例を補う。
- [ ] Li Chao Tree DP — **一部対応**。 [dp_optimization/li_chao_tree.hpp](../dp_optimization/li_chao_tree.hpp)。 Li Chao本体は存在。DP遷移への組込みと比較検証・使用例を補う。
- [ ] Slope Trick — **未実装**。
- [ ] Aliens Trick（WQS Binary Search） — **未実装**。
- [x] SOS DP — **実装あり**。 [set_function/sos_dp.hpp](../set_function/sos_dp.hpp)。

- **B**：Min-Plus Convolution（凸・凹）、Monge Shortest Path、Mongeの辺数制約付き最短路、LARSCH、オンライン・オフライン変換、分枝限定ナップサック、Four Russians法

- **C**：高度なMonge行列積、複雑なSlope Trick、動的な最適化DP、特殊な高速ナップサック

補足：SMAWK、Monotone Minima、Min-Plus Convolutionは独立した便利な部品として整備する価値があります。

<a id="3-13"></a>

### 3.13 数論：基本整数演算

主な配置先: `number_theory_basic/`。

- **S**：GCD/LCM、拡張Euclid、mod累乗、mod逆元、素数篩、SPF（最小素因数）、素因数分解、約数列挙

**A（優先実装）**

- [x] Linear Sieve — **実装あり**。 [number_theory_basic/smallest_prime_factor.hpp](../number_theory_basic/smallest_prime_factor.hpp)。 SmallestPrimeFactorは線形篩を実装済み。
- [ ] Euler φ — **未実装**。
- [ ] Möbius μ — **未実装**。
- [ ] CRT — **一部対応**。 [convolution/convolution_mod.hpp](../convolution/convolution_mod.hpp)。 3法固定のCRT復元は内部に存在。汎用の合同式を解く公開CRT APIは未実装。
- [ ] 中国剰余定理の一般化 — **未実装**。
- [ ] Garner — **一部対応**。 [convolution/convolution_mod.hpp](../convolution/convolution_mod.hpp)。 固定3法の混合基数復元は存在。任意の互いに素な法列を受けるGarner APIは未実装。
- [ ] Floor Sum — **未実装**。
- [x] Miller–Rabin — **実装あり**。 [number_theory_convolution/miller_rabin.hpp](../number_theory_convolution/miller_rabin.hpp)。
- [x] Pollard's Rho — **実装あり**。 [number_theory_convolution/pollard_rho.hpp](../number_theory_convolution/pollard_rho.hpp)。
- [ ] Primitive Root — **一部対応**。 [convolution/detail/common.hpp](../convolution/detail/common.hpp)。 与えた値が原始根かを検査する内部関数は存在。原始根を求める公開APIは未実装。

- **B**：Discrete Log（BSGS）、mod平方根（Tonelli–Shanks）、整数k乗根、modテトレーション、商列挙、線形合同方程式、Lucasの定理、素数カウント

- **C**：mod k乗根、64bit高性能modint、Gaussian Integer、連分数・有理数近似、Pell方程式、二平方数和の表示

<a id="3-14"></a>

### 3.14 数論：乗法的関数・高度な整数問題

主な配置先: `number_theory_convolution/`。

**A（優先実装）**

- [x] 約数ゼータ変換 — **実装あり**。 [number_theory_convolution/divisor_zeta_transform.hpp](../number_theory_convolution/divisor_zeta_transform.hpp)。
- [x] 約数メビウス変換 — **実装あり**。 [number_theory_convolution/divisor_mobius_transform.hpp](../number_theory_convolution/divisor_mobius_transform.hpp)。
- [x] 倍数ゼータ変換 — **実装あり**。 [number_theory_convolution/multiple_zeta_transform.hpp](../number_theory_convolution/multiple_zeta_transform.hpp)。
- [x] Dirichlet Convolution — **実装あり**。 [number_theory_convolution/dirichlet_convolution.hpp](../number_theory_convolution/dirichlet_convolution.hpp)。
- [x] GCD/LCM Convolution — **実装あり**。 [number_theory_convolution/all.hpp](../number_theory_convolution/all.hpp)。 gcd_convolution.hpp・lcm_convolution.hpp。

- **B**：約数個数和、Euler φの総和、Mertens Function、素数個数計算（Lucy DP / Min_25系）、乗法的関数の総和、Square-Free Counting

- **C**：高速素数和、一般化されたMin_25 Sieve、Dirichlet逆元、任意乗法的関数の大規模総和、特殊なmod和

<a id="3-15"></a>

### 3.15 組合せ論・数え上げ

主な配置先: `combinatorics/`。

- **S**：階乗・逆階乗、二項係数、順列・組合せ、Catalan数、重複組合せ、包除原理

**A（優先実装）**

- [ ] Stirling数（第1種・第2種） — **未実装**。
- [ ] Bell数 — **未実装**。
- [ ] 分割数 — **未実装**。
- [ ] Montmort数 — **未実装**。
- [ ] Burnsideの補題 — **未実装**。
- [ ] Pólya Enumeration — **未実装**。
- [x] Lagrange補間 — **実装あり**。 [fps/interpolation.hpp](../fps/interpolation.hpp)。 相異なる点の多項式補間は対応済み。等間隔点から1点を評価する特化版を追加するなら拡張として扱う。
- [x] 二項係数の前計算 — **実装あり**。 [combinatorics/binomial.hpp](../combinatorics/binomial.hpp)。 FactorialTableとPascalBinomialを実装済み。

- **B**：Eulerian数、Bernoulli数、Narayana数、Lagrange Inversion、Stirling Transform、二項変換、階乗べきとの基底変換、標本点シフト、二項係数のPrefix Sum

- **C**：RSK対応、Young Tableaux、Hook-Length Formula、P-recursive数列、高度な組合せ恒等式用の変換

補足：数論・組合せ論・母関数の高度な部分は、Nyaan、maspypy、hitonanodeが特に参考になります。

<a id="3-16"></a>

### 3.16 畳み込み・高速変換

主な配置先: `convolution/`。

- **S**：NTT、通常の畳み込み、FFT（実数・複素数）

**A（優先実装）**

- [x] 任意mod畳み込み（CRT/Garner） — **実装あり**。 [convolution/convolution_mod.hpp](../convolution/convolution_mod.hpp)。
- [ ] Karatsuba — **未実装**。
- [ ] FWHT — **未実装**。
- [ ] AND/OR/XOR Convolution — **未実装**。 3.19のXOR/AND/OR Convolutionと共通実装する。
- [x] Subset/Superset Zeta・Möbius Transform — **実装あり**。 [set_function/all.hpp](../set_function/all.hpp)。 subset/superset両方向のzeta/mobiusを実装済み。
- [x] Subset Convolution — **実装あり**。 [set_function/subset_convolution.hpp](../set_function/subset_convolution.hpp)。

- **B**：Relaxed/Online Convolution、GCD/LCM Convolution、Min-Plus Convolution、多次元畳み込み、巨大配列の畳み込み、Middle Product、Chirp-Z Transform

- **C**：Rader FFT、Bluestein FFT、高性能64bit NTT、AVX2 NTT、多変数巡回畳み込み、Multiplicative Convolution

補足：重要度が高く、ライブラリ化の効果も大きい分野です。

<a id="3-17"></a>

### 3.17 多項式・形式的冪級数（FPS）

主な配置先: `fps/`。

- **S**：多項式加減乗、微分、積分、係数切り詰め

**A（優先実装）**

- [x] FPS逆数 inv — **実装あり**。 [fps/inv.hpp](../fps/inv.hpp)。
- [x] log — **実装あり**。 [fps/log.hpp](../fps/log.hpp)。
- [x] exp — **実装あり**。 [fps/exp.hpp](../fps/exp.hpp)。
- [x] pow — **実装あり**。 [fps/pow.hpp](../fps/pow.hpp)。
- [x] sqrt — **実装あり**。 [fps/sqrt.hpp](../fps/sqrt.hpp)。
- [x] 多項式除算・剰余 — **実装あり**。 [fps/division.hpp](../fps/division.hpp)。
- [x] 多点評価 — **実装あり**。 [fps/multipoint_evaluation.hpp](../fps/multipoint_evaluation.hpp)。
- [x] 補間 — **実装あり**。 [fps/interpolation.hpp](../fps/interpolation.hpp)。
- [x] Taylor Shift — **実装あり**。 [fps/taylor_shift.hpp](../fps/taylor_shift.hpp)。
- [x] Bostan–Mori — **実装あり**。 [fps/bostan_mori.hpp](../fps/bostan_mori.hpp)。

- **B**：FPS合成、合成逆関数、Sparse FPS、Online FPS、Polynomial GCD、Polynomial Mod Pow、Subproduct Tree、Partial Fraction Decomposition

- **C**：Power Projection、多項式の根の高速計算、高速FPS合成、特殊なFPS変換、二変数FPS、多変数FPS、FPSの微分方程式、多項式行列の行列式・Prefix Product、Composed Sum/Product、高速中間積

<a id="3-18"></a>

### 3.18 線形漸化式・母関数

主な配置先: `linear_recurrence/`。

**A（優先実装）**

- [x] 行列累乗による漸化式 — **実装あり**。 [linear_recurrence/matrix_recurrence.hpp](../linear_recurrence/matrix_recurrence.hpp)。
- [x] Kitamasa — **実装あり**。 [linear_recurrence/kitamasa.hpp](../linear_recurrence/kitamasa.hpp)。
- [x] Berlekamp–Massey — **実装あり**。 [linear_recurrence/berlekamp_massey.hpp](../linear_recurrence/berlekamp_massey.hpp)。
- [x] Bostan–Mori — **実装あり**。 [linear_recurrence/bostan_mori.hpp](../linear_recurrence/bostan_mori.hpp)。

- **B**：線形漸化式の推定と第N項、P-recursive数列、指数型母関数（EGF）の演算、母関数の係数抽出、有理型母関数、等比数列の多項式重み付き和

- **C**：Holonomic Sequence、Reeds–Sloane、特殊な母関数の合成、線形漸化式の大量項計算

補足：最近扱っていたようなEGF・スターリング変換・オンライン畳み込み・一次分数変換を使う問題まで視野に入れるなら、ここは重点的に整備するとよい分野です。

参考：Nyaan FPS、maspypy Polynomial。

<a id="3-19"></a>

### 3.19 集合関数・Bitmask変換

主な配置先: `set_function/`。

- **S**：Submask/Supermask列挙、SOS DP、集合ゼータ・メビウス変換

**A（優先実装）**

- [ ] XOR/AND/OR Convolution — **未実装**。 3.16のFWHT・AND/OR/XOR Convolutionと共通実装する。
- [x] Subset Convolution — **実装あり**。 [set_function/subset_convolution.hpp](../set_function/subset_convolution.hpp)。
- [ ] Ranked Zeta Transform — **一部対応**。 [set_function/subset_convolution.hpp](../set_function/subset_convolution.hpp)。 rank別変換は内部処理として存在。再利用できる公開Ranked Zeta APIは未実装。

- **B**：Set Power Series（SPS）の exp/log/inv、SPSの合成、集合分割DP、高速な部分集合畳み込み

- **C**：SPS Power Projection、Online Set Function Convolution、特殊な集合関数の変換

補足：Library CheckerにはSubset ConvolutionやSet Power Seriesの検証問題があります。

<a id="3-20"></a>

### 3.20 線形代数

主な配置先: `linear_algebra/`。

- **S**：Matrix、行列積、行列累乗、連立一次方程式（Gauss–Jordan）、XOR基底

**A（優先実装）**

- [x] 行列式 — **実装あり**。 [graph_combinatorics/determinant_mod.hpp](../graph_combinatorics/determinant_mod.hpp)。 素数modの行列式は実装済み。汎用体版・整数版を追加するなら拡張として扱う。
- [ ] 逆行列 — **一部対応**。 [linear_algebra/gauss_jordan.hpp](../linear_algebra/gauss_jordan.hpp)。 連立方程式の核はある。逆行列の公開API・専用検証が未実装。
- [x] 階数 — **実装あり**。 [linear_algebra/gauss_jordan.hpp](../linear_algebra/gauss_jordan.hpp)。 返り値rankを利用。
- [ ] GF(2)上のBitset Gauss — **未実装**。
- [ ] 線形空間の交差 — **未実装**。
- [ ] 特性多項式 — **未実装**。

- **B**：Sparse Matrix、Black-Box Linear Algebra、Wiedemann Algorithm、最小多項式、Matrix Tree Theorem、Low-Rank Update

- **C**：Frobenius Normal Form、行列のHafnian、多項式行列の行列式、高速な特性多項式計算

<a id="3-21"></a>

### 3.21 文字列アルゴリズム

主な配置先: `string_algorithm/`。

- **S**：Z Algorithm、KMP / Prefix Function、Rolling Hash、Trie、Suffix Array、LCP Array、Manacher

**A（優先実装）**

- [x] Aho–Corasick — **実装あり**。 [string_algorithm/aho_corasick.hpp](../string_algorithm/aho_corasick.hpp)。
- [ ] Suffix Automaton — **未実装**。
- [x] Palindromic Tree（Eertree） — **実装あり**。 [string_algorithm/eertree.hpp](../string_algorithm/eertree.hpp)。
- [ ] Lyndon Factorization — **未実装**。
- [ ] Longest Common Substring — **未実装**。
- [ ] 2D Rolling Hash — **未実装**。

- **B**：Runs Enumeration、Suffix Tree、Wildcard Pattern Matching、Dynamic Rolling Hash、文字列の最小巡回シフト、複数文字列の比較、回文分割DP

- **C**：Double-Ended Palindromic Tree、Online Z Algorithm、Generalized Suffix Automaton、高度な文字列周期問題

補足：文字列は基礎アルゴリズムの割に実装ミスの影響が大きいため、特にverifyを重視したい分野です。

<a id="3-22"></a>

### 3.22 計算幾何

主な配置先: `computational_geometry/`。

- **S**：Point/Vector、内積・外積、CCW、線分交差判定、直線交点、射影、距離、Convex Hull

**A（優先実装）**

- [x] 点の偏角ソート — **実装あり**。 [computational_geometry/angular_sort.hpp](../computational_geometry/angular_sort.hpp)。
- [x] Polygon Area — **実装あり**。 [computational_geometry/polygon.hpp](../computational_geometry/polygon.hpp)。
- [x] Point in Polygon — **実装あり**。 [computational_geometry/polygon.hpp](../computational_geometry/polygon.hpp)。
- [x] Convex Polygon Contains — **実装あり**。 [computational_geometry/polygon.hpp](../computational_geometry/polygon.hpp)。
- [x] Convex Polygon Cut — **実装あり**。 [computational_geometry/convex_cut.hpp](../computational_geometry/convex_cut.hpp)。
- [x] Rotating Calipers — **実装あり**。 [computational_geometry/rotating_calipers.hpp](../computational_geometry/rotating_calipers.hpp)。
- [x] 最近点対 — **実装あり**。 [computational_geometry/closest_pair.hpp](../computational_geometry/closest_pair.hpp)。

- **B**：Minimum Enclosing Circle、Circle-Line/Circle-Circle Intersection、Convex Layers、Minkowski Sum、Farthest Pair、Manhattan MST、格子点数（Pick）、半平面交差

- **C**：Delaunay Triangulation、Voronoi Diagram、Dynamic Convex Hull、Polygon Triangulation、Convex Polygon Union Area、三次元幾何

補足：2D幾何は、整数座標による厳密計算と浮動小数点計算のAPIを分けて実装する方が安全です。

特にmaspypyはDelaunay Triangulation、動的凸包、Minkowski Sumなどの上級幾何も収録しています。

<a id="3-23"></a>

### 3.23 組合せ最適化・マトロイド

主な配置先: `combinatorial_optimization/`。

**A（優先実装）**

- [x] Hungarian Algorithm — **実装あり**。 [combinatorial_optimization/hungarian.hpp](../combinatorial_optimization/hungarian.hpp)。
- [x] 最大マッチング — **実装あり**。 [combinatorial_optimization/general_matching.hpp](../combinatorial_optimization/general_matching.hpp)。
- [x] 最小費用流 — **実装あり**。 [combinatorial_optimization/min_cost_flow.hpp](../combinatorial_optimization/min_cost_flow.hpp)。
- [x] Assignment Problem — **実装あり**。 [combinatorial_optimization/assignment_problem.hpp](../combinatorial_optimization/assignment_problem.hpp)。

- **B**：Matroid Intersection、Graphic Matroid、Partition Matroid、Maximum Weight Independent Set、Linear Matroid Parity

- **C**：Weighted Matroid Intersection、Base Polyhedron、Submodular Optimization、特殊な整数計画法

補足：組合せ最適化は一般的な競プロライブラリでは手薄になりがちですが、hitonanodeにはMatroid Intersection、Linear Matroid Parityなどが収録されています。

<a id="3-24"></a>

### 3.24 ゲーム理論

主な配置先: `game_theory/`。

- **S**：Nim、Misère Nim、Grundy数計算、DAGゲーム

**A（優先実装）**

- [ ] Nim Product — **未実装**。
- [ ] 周期的Grundy数列の検出 — **未実装**。
- [x] ゲームの直和 — **実装あり**。 [game_theory/dag_game.hpp](../game_theory/dag_game.hpp)。 独立な駒のnim_sum・勝ち手復元。grundy.hppにも基本集約あり。

- **B**：Nimberの体演算、グラフゲーム、Partizan Gameの基本操作

- **C**：超限的なゲーム値・高度な組合せゲーム理論

<a id="3-25"></a>

### 3.25 探索・ヒューリスティック

主な配置先: `heuristic_search/`。

- **S**：乱数生成器、Timer、Simulated Annealing、Beam Search、局所探索

**A（優先実装）**

- [ ] Zobrist Hash — **未実装**。
- [ ] Tabu Search — **未実装**。
- [x] Hill Climbing — **実装あり**。 [heuristic_search/local_search.hpp](../heuristic_search/local_search.hpp)。 first/best improvementによるHill Climbing。
- [ ] Beam Searchの重複除去 — **未実装**。 [heuristic_search/beam_search.hpp](../heuristic_search/beam_search.hpp)。 既存beam_searchは重複を残す契約。dedupオプションまたは別APIを追加。
- [ ] スコア差分更新 — **一部対応**。 [heuristic_search/simulated_annealing.hpp](../heuristic_search/simulated_annealing.hpp)。 評価関数の共通入口はある。差分スコア・状態apply/undoを扱う専用APIと使用例は未実装。

- **B**：Multi-Start SA、温度制御、パラメータチューニング、多腕バンディット、焼きなましの近傍管理

- **C**：多点並列探索、特殊な確率分布の高速サンプリング

補足：これはAtCoder Heuristic Contest（AHC）やマラソン系も対象にする場合です。
