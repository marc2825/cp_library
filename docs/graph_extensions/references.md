# 一次資料・設計判断

確認日: 2026-10-25。外部ライブラリを実行依存にせず、既存のGraph・SegmentTreeCover・matching・Dinic・MinCostFlow・LCT・Cactus・DAGReachabilityを共有します。

| 資料 | 確認した内容 |
|---|---|
| [KACTL EdgeColoring.h 固定版](https://github.com/kth-competitive-programming/kactl/blob/b3548e973f8772010c1ef84e6d427c749733fd2c/content/graph/EdgeColoring.h) | Simon Lindholm、ファイル内CC0宣言。fan、2色交互路、回転。G-12のC++と図解に改変して取り込み。 |
| [KACTL GomoryHu.h 固定版](https://github.com/kth-competitive-programming/kactl/blob/b3548e973f8772010c1ef84e6d427c749733fd2c/content/graph/GomoryHu.h) / [NetworkX公式仕様](https://networkx.org/documentation/stable/reference/algorithms/generated/networkx.algorithms.flow.gomory_hu_tree.html) | n−1回の最大流とカット木。こちらは既存Dinicをresetし、親交換と容量総和の検査を実装。 |
| [Even–Tarjan: Computing an st-numbering](https://collaborate.princeton.edu/en/publications/computing-an-st-numbering/) | 双極順序と線形時間アルゴリズム。DFSのlowpointと符号付きリスト挿入を、本Graph・反復DFS・std::listで実装。 |
| [maspypyのst-Numbering公開API](https://maspypy.github.io/library/graph/st_numbering.hpp) | 非隣接の端点と構成不能時の扱いを比較。ソースのライセンス宣言は確認できず、コードの取り込み元にはしていません。 |
| [Karp: A Characterization of the Minimum Cycle Mean in a Digraph](https://www2.eecs.berkeley.edu/Pubs/TechRpts/1977/29121.html) | 長さkのwalk DPによるmin-max比。全頂点を開始点として初期化し、厳密な分数比較とtight閉路の復元を追加。 |
| [NetworkX: to_vertex_cover](https://networkx.org/documentation/stable/reference/algorithms/generated/networkx.algorithms.bipartite.matching.to_vertex_cover.html) / [min_edge_cover](https://networkx.org/documentation/stable/reference/algorithms/generated/networkx.algorithms.covering.min_edge_cover.html) | Königの交互到達集合、最大matchingを辺被覆へ拡張する規約。既存の二部・一般matchingを共有。 |
| [NetworkX: is_graphical](https://networkx.org/documentation/stable/reference/algorithms/generated/networkx.algorithms.graphical.is_graphical.html) | Erdős–GallaiとHavel–Hakimi、単純無向次数列の条件。 |

## 取り込みとライセンス

G-12はcommit `b3548e973f8772010c1ef84e6d427c749733fd2c` のKACTLを使用。cp名前空間、Graph入力、単純性の例外検査、色数・最大次数・元辺IDを持つ結果型へ変更しました。図解は同じfan法をJavaScriptへ移植し、手順と彩色の状態を保存します。[CC0本文](../range_query/ACL-LICENSE.txt)と[出典台帳](../../verification/provenance.json)に固定版・元hash・変更説明・ローカルhashを記録しています。

他の追加ヘッダは、上記アルゴリズムの定義と既存APIを組み合わせた本ライブラリ向けの実装です。[既存グラフの出典](../graph_basic/references.md)、[フローの出典](../graph_flow/references.md)、[組合せ最適化の出典](../combinatorial_optimization/references.md)も参照してください。

G-01は標準区間ノード、G-08は成分別最小辺oracle、G-09はLCTの最大辺交換を公開用途にまとめました。G-11は推移閉包を明示し、元辺のDAG path coverと区別。G-13/14は証拠付きBellman–Ford coreを共有。G-15は既存の最小費用流による残余辺の組み替えで指定本数を最適化し、無向の逆向き流を相殺して元辺の重複使用を防ぎます。

オンライン提出は未実施です。全探索・独立実装との比較は[ローカル検証](README.md)に記録します。
