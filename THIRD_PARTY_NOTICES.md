# 第三者コードの出典とライセンス

ルートの [LICENSE](LICENSE)（CC0 1.0）が適用されるのは、このリポジトリの作者が書いた部分だけです。
外部のライブラリ・記事に由来するコードは CC0 の対象外で、それぞれの元のライセンスに従います。
再利用するときは、元のライセンスの条件（著作権表示やライセンス文の同梱など）を守ってください。

## ai_generated/

移植元・取り込み版・改変内容・ハッシュは [出典台帳](ai_generated/verification/provenance.json)（[説明](ai_generated/docs/development_records/provenance.md)）で管理しています。

| ファイル | 出典 | ライセンス | ライセンス文 |
|---|---|---|---|
| [range_query/segtree.hpp](ai_generated/range_query/segtree.hpp)<br>[range_query/lazy_segtree.hpp](ai_generated/range_query/lazy_segtree.hpp) | [AtCoder Library](https://github.com/atcoder/ac-library/blob/864245a00b00dd008d1abfdc239618fdb7d139da/atcoder/segtree.hpp) | CC0-1.0 | [原文](ai_generated/docs/range_query/ACL-LICENSE.txt) |
| [graph_basic/manhattan_mst.hpp](ai_generated/graph_basic/manhattan_mst.hpp) | [KACTL](https://raw.githubusercontent.com/kth-competitive-programming/kactl/b3548e973f8772010c1ef84e6d427c749733fd2c/content/geometry/ManhattanMST.h) | CC0-1.0 | [原文](ai_generated/docs/range_query/ACL-LICENSE.txt) |
| [graph_connectivity/three_edge_connected_components.hpp](ai_generated/graph_connectivity/three_edge_connected_components.hpp) | [Luzhiled's Library (ei1333/library)](https://raw.githubusercontent.com/ei1333/library/088b4c8b49f85194099305d998e7113dc3d5dd2e/graph/connected-components/three-edge-connected-components.hpp) | Unlicense | [原文](ai_generated/verification/licenses/Unlicense.txt) |
| [dp_optimization/larsch.hpp](ai_generated/dp_optimization/larsch.hpp) | [noshi91/Library](https://raw.githubusercontent.com/noshi91/Library/991c68f07261b88f1fb90abe1d588844d710ec38/algorithm/larsch.cpp) | CC0-1.0 | [原文](ai_generated/docs/range_query/ACL-LICENSE.txt) |
| [data_structure/radix_heap_iwiwi.hpp](ai_generated/data_structure/radix_heap_iwiwi.hpp) | [iwiwi/radix-heap](https://raw.githubusercontent.com/iwiwi/radix-heap/f54eba0a19782c67a9779c28263a7ce680995eda/radix_heap.h) | 未確認 | — |
| [basic_utility/permutation_tree.hpp](ai_generated/basic_utility/permutation_tree.hpp) | [hitonanode/cplib-cpp](https://hitonanode.github.io/cplib-cpp/other_algorithms/permutation_tree.hpp.html) | MIT | [原文](ai_generated/verification/licenses/hitonanode-MIT.txt) |
| [static_query/static_range_lis.hpp](ai_generated/static_query/static_range_lis.hpp) | [tko919/library](https://github.com/tko919/library/blob/3925f75892c0705fb22d6e3f892ae1b5b4fac32d/DataStructure/rangelis.hpp) | CC0-1.0 | [原文](ai_generated/verification/licenses/tko919-CC0.txt) |
| [tree/detail/top_tree_core.hpp](ai_generated/tree/detail/top_tree_core.hpp) | [Luzhiled's Library (ei1333/library)](https://raw.githubusercontent.com/ei1333/library/088b4c8b49f85194099305d998e7113dc3d5dd2e/structure/dynamic-tree/lazy-top-tree.hpp) | Unlicense | [原文](ai_generated/verification/licenses/Unlicense.txt) |
| [graph_combinatorics/vizing_edge_coloring.hpp](ai_generated/graph_combinatorics/vizing_edge_coloring.hpp)<br>[visualizations/graph_extensions/model.js](ai_generated/visualizations/graph_extensions/model.js) | [KACTL](https://raw.githubusercontent.com/kth-competitive-programming/kactl/b3548e973f8772010c1ef84e6d427c749733fd2c/content/graph/EdgeColoring.h) | CC0-1.0 | [原文](ai_generated/docs/range_query/ACL-LICENSE.txt) |
| [graph_flow/detail/weighted_blossom.hpp](ai_generated/graph_flow/detail/weighted_blossom.hpp)<br>[graph_flow/general_weighted_matching.hpp](ai_generated/graph_flow/general_weighted_matching.hpp) | [Boost.Graph](https://raw.githubusercontent.com/boostorg/graph/54a94911e68dccf5812b32d018566566700499f3/include/boost/graph/maximum_weighted_matching.hpp) | BSL-1.0 | [原文](ai_generated/verification/licenses/Boost-1.0.txt) |

[Boost.Multiprecision](https://www.boost.org/doc/libs/release/libs/multiprecision/)（BSL-1.0）は `basic_utility/big_integer.hpp` などから `#include` して使う外部依存で、本体は同梱していません。

ライセンスが「未確認」のものは、元のリポジトリでライセンス表記を確認できていません。再配布・再利用の前に作者に確認してください。

## depricated/ とテンプレート

`depricated/` の旧ライブラリには、外部ライブラリを参考にしたり移植したりしたコードが含まれます。
個別の確認はしていないため、ファイル内に出典（URL・作者名）が書かれているコードは、出典元のライセンスに従うものとして扱ってください。
`my_template*.cpp` など、その他のファイルで出典が付記されている部分も同様です。

| ファイル | ファイル内に記載された出典 |
|---|---|
| [depricated/fast_io_nyaan.hpp](depricated/fast_io_nyaan.hpp) | Nyaan's Library |
| [depricated/fast_io_yosupo.hpp](depricated/fast_io_yosupo.hpp) | yosupo のライブラリ |
| [depricated/data_structure/radix_heap_iwiwi.hpp](depricated/data_structure/radix_heap_iwiwi.hpp) | iwiwi/radix-heap |
| [depricated/data_structure/ConvexHullTrick.hpp](depricated/data_structure/ConvexHullTrick.hpp) | KACTL |
| [depricated/data_structure/Priority_Sum_Structure.hpp](depricated/data_structure/Priority_Sum_Structure.hpp) | Luzhiled's Library |
| [depricated/data_structure/2D_segtree.hpp](depricated/data_structure/2D_segtree.hpp) | Nyaan's Library |
| [depricated/2D_CumulativeSum.hpp](depricated/2D_CumulativeSum.hpp) | Luzhiled's Library |
| [depricated/Combination/Combination_ModNotPrime.hpp](depricated/Combination/Combination_ModNotPrime.hpp) | Nyaan's Library |
| [depricated/geometry/2D_geometry_template.hpp](depricated/geometry/2D_geometry_template.hpp) | Luzhiled's Library、Nyaan's Library、tjkendev/procon-library |
| [depricated/graph_theory/graph_template.hpp](depricated/graph_theory/graph_template.hpp) | Luzhiled's Library、Nyaan's Library、KACTL、kopricky、nok0、magurofly |
| [depricated/graph_theory/Tree/tree_template.hpp](depricated/graph_theory/Tree/tree_template.hpp) | Nyaan's Library、kopricky |
| [depricated/graph_theory/dijkstra.hpp](depricated/graph_theory/dijkstra.hpp) | Luzhiled's Library |
| [depricated/parsing/parsing_template.hpp](depricated/parsing/parsing_template.hpp) | dai1741/maximum-algo-2012 |
| [depricated/string/RunLengthEncoding.hpp](depricated/string/RunLengthEncoding.hpp) | Nyaan's Library |
| [depricated/string/rolling_hash.hpp](depricated/string/rolling_hash.hpp) | Nyaan's Library |
