[![AtCoder Algorithm Rating](https://badgen.org/img/atcoder/marc2825/rating/algorithm)](https://atcoder.jp/users/marc2825)
[![AtCoder Heuristic Rating](https://badgen.org/img/atcoder/marc2825/rating/heuristic)](https://atcoder.jp/users/marc2825?contestType=heuristic)
[![Codeforces Rating](https://codeforces-readme-stats.vercel.app/api/badge?username=marc2825)](https://codeforces.com/profile/marc2825)

# cp_library

競技中に必要な機能だけ読み込む C++ ライブラリです。グラフ・木は C++17 以降、外部ライブラリ不要で利用できます。

## グラフ・木の入口

```cpp
#include "graph/dijkstra.hpp"
#include "tree/doubling_lca.hpp"

cp::Graph<long long> g(3); // 既定は無向。添字・辺番号は0始まり。
g.add_edge(0, 1, 5);
g.add_edge(1, 2, 2);

auto shortest = cp::dijkstra(g, 0);
auto path = shortest.vertex_path(2); // {0,1,2}
cp::DoublingLCA<long long> tree(g, 0);
auto distance = tree.distance(0, 2); // 7
```

ライブラリのルートに include パスを通します。

```sh
g++ -std=c++17 -O2 -I /path/to/cp_library main.cpp
```

| やりたいこと | ヘッダ | API |
|---|---|---|
| グラフを保持する | [graph/graph.hpp](graph/graph.hpp) | `cp::Graph<T>` |
| 辺リストを入力する | [graph/input.hpp](graph/input.hpp) | `cp::read_graph<T>` |
| 辺の本数による最短路 | [graph/bfs.hpp](graph/bfs.hpp) | `cp::bfs` |
| 非負重みの最短路 | [graph/dijkstra.hpp](graph/dijkstra.hpp) | `cp::dijkstra` |
| 重み0/1の最短路 | [graph/zero_one_bfs.hpp](graph/zero_one_bfs.hpp) | `cp::zero_one_bfs` |
| 負辺を含む最短路 | [graph/bellman_ford.hpp](graph/bellman_ford.hpp) | `cp::bellman_ford` |
| 全頂点対の最短路 | [graph/floyd_warshall.hpp](graph/floyd_warshall.hpp) | `cp::floyd_warshall` |
| 最小全域木・森 | [graph/minimum_spanning_forest.hpp](graph/minimum_spanning_forest.hpp) | `cp::kruskal`, `cp::prim` |
| DFSを行う | [graph/dfs.hpp](graph/dfs.hpp) | `cp::depth_first_search` |
| 連結成分に分ける | [graph/connected_components.hpp](graph/connected_components.hpp) | `cp::connected_components` |
| 二部グラフを判定する | [graph/bipartite.hpp](graph/bipartite.hpp) | `cp::bipartite_coloring` |
| DAGを順序付ける | [graph/topological_sort.hpp](graph/topological_sort.hpp) | `cp::topological_sort` |
| 閉路を1つ復元する | [graph/cycle.hpp](graph/cycle.hpp) | `cp::find_cycle` |
| グリッドを探索する | [graph/grid_bfs.hpp](graph/grid_bfs.hpp) | `cp::grid_bfs` |
| 橋・関節点を求める | [graph/low_link.hpp](graph/low_link.hpp) | `cp::low_link` |
| 二重辺連結成分に分ける | [graph/two_edge_connected_components.hpp](graph/two_edge_connected_components.hpp) | `cp::two_edge_connected_components` |
| 二重頂点連結成分・Block Cut Forest | [graph/biconnected_components.hpp](graph/biconnected_components.hpp) | `cp::biconnected_components`, `cp::block_cut_forest` |
| 親・深さ・部分木を求める | [tree/rooted_tree.hpp](tree/rooted_tree.hpp) | `cp::RootedTree<T>` |
| LCA・距離・パス上の移動 | [tree/doubling_lca.hpp](tree/doubling_lca.hpp) | `cp::DoublingLCA<T>` |
| Euler Tourを構築する | [tree/euler_tour.hpp](tree/euler_tour.hpp) | `cp::EulerTour<T>` |
| 部分木・パスクエリを処理する | [tree/heavy_light_decomposition.hpp](tree/heavy_light_decomposition.hpp) | `cp::HeavyLightDecomposition<T>` |
| 木の直径を求める | [tree/diameter.hpp](tree/diameter.hpp) | `cp::tree_diameter` |
| 重心・重心分解を求める | [tree/centroid_decomposition.hpp](tree/centroid_decomposition.hpp) | `cp::tree_centroids`, `cp::CentroidDecomposition<T>` |

[使い方・仕様・旧APIからの移行](docs/graph_tree.md) / [動く使用例](examples/) / [以前の検証履歴と参考資料](docs/graph_tree_history.md)

すべて必要な場合は `graph/all.hpp` と `tree/all.hpp` を利用できます。その他のカテゴリは従来のヘッダ構成です。

## ローカル検証

```sh
python3 verify/run_graph_tree.py
python3 verify/run_graph_tree.py --sanitize --stress
```

単独 include、include 順序、複数翻訳単位でのリンク、使用例、1,920個のランダムなグラフ・木、旧APIを検証します。`--stress` は20万頂点のパスを使います。`--compiler clang++` などでコンパイラを切り替えられます。
