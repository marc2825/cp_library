# 参考資料・設計判断

2026-10-15 に以下の一次資料・ライブラリの著者が公開する解説と問題仕様を確認しました。以前に整理したグラフ実装をカテゴリ化し、以下の原理・境界条件と照合しました。この追加で外部ライブラリのソースを転載していません。

| 参照先 | 確認した点・今回の扱い |
|---|---|
| [CP-Algorithms: BFS](https://cp-algorithms.com/graph/breadth-first-search.html) / [DFS](https://cp-algorithms.com/graph/depth-first-search.html) | 層順の最短辺数と DFS の訪問順。再帰を避け、単一始点と全成分 DFS を分離 |
| [CP-Algorithms: 0-1 BFS](https://cp-algorithms.com/graph/01_bfs.html) | 0 の先頭追加・1 の末尾追加。古い候補を破棄 |
| [CP-Algorithms: sparse Dijkstra](https://cp-algorithms.com/graph/dijkstra_sparse.html) / [経路復元](https://cp-algorithms.com/graph/dijkstra.html) | priority_queue に古い候補を残す方式、親の逆走査。頂点と論理辺 ID を別々に保持 |
| [CP-Algorithms: Bellman–Ford](https://cp-algorithms.com/graph/bellman_ford.html) | V−1 周後の改善、到達可能な負閉路とその先への影響。孤立点の多い入力では辺列を直接走査 |
| [CP-Algorithms: Floyd–Warshall](https://cp-algorithms.com/graph/all-pair-shortest-path-floyd-warshall.html) | k を最外周にする更新、負の対角と影響する始終点。結果を始点別の共通型で返す |
| [CP-Algorithms: Topological Sort](https://cp-algorithms.com/graph/topological-sort.html) / [KACTL: TopoSort](https://github.com/kth-competitive-programming/kactl/blob/main/content/graph/TopoSort.h) | DAG の条件、入次数による削除。閉路時は途中列を返さず nullopt、空 DAG は成功した空列 |
| [CP-Algorithms: Cycle Detection](https://cp-algorithms.com/graph/finding-cycle.html) | 3 色 DFS。親辺 ID で除外して自己ループ・多重辺にも対応 |
| [CP-Algorithms: Kruskal+DSU](https://cp-algorithms.com/graph/mst_kruskal_with_dsu.html) / [Prim](https://cp-algorithms.com/graph/mst_prim.html) | cut による軽い辺の採用。無向・非連結・負辺を扱い、最小全域森を返す |
| [LC: Shortest Path 仕様](https://github.com/yosupo06/library-checker-problems/blob/master/graph/shortest_path/task.md) | 距離と経路の出力。複数解を許す検証を用意 |
| [LC: Cycle Detection 仕様](https://github.com/yosupo06/library-checker-problems/blob/master/graph/cycle_detection/task.md) | 辺素な有向閉路の出力。返した辺の向きと重複を検証 |
| [LC: MST 仕様](https://github.com/yosupo06/library-checker-problems/blob/master/graph/minimum_spanning_tree/task.md) | 最小コストと元の辺 ID。Kruskal / Prim それぞれを検証 |
| [AtCoder ABC077 D](https://atcoder.jp/contests/abc077/tasks/arc084_b) | 余りの状態に ×10（重み0）と +1（重み1）の辺を張る 0-1 BFS の応用 |

AOJ の BFS・DFS・単一始点／全点対最短路・トポロジカル順序の形式も個別説明から参照できます。オンライン AC は主張せず、ローカルで独立オラクルと比較します。

共有の Graph と入力補助は既存 `graph/` に残し、アルゴリズム本体は `graph_basic/`、従来のパスは互換入口としました。木・LowLink 等で同じ Graph と辺 ID を使い続けられます。Kruskal の成分管理は本リポジトリ [3.2](../union_find/README.md) を再利用します。

図解は主要な状態変化を示し、C++ の各命令やメモリ配置との一致を保証しません。優先度付きキューは取り出し順、DFS は全成分版、負閉路の影響はフラグを −∞ 表示へ変換しています。C++ の数値契約と図解の小さな整数入力の条件は区別します。

[検証方法](README.md#検証) / [既存実装の履歴](../graph_tree_history.md)

3.7〜3.11 A補完の[一次資料・問題例・設計判断](../graph_a/references.md)。旧Radix Heap・SCC・Block-Cut・Dinic・BinaryLiftingは共通実装を利用しています。
