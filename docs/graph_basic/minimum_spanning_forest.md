---
title: Kruskal / Prim の共通入口
documentation_of: //graph_basic/minimum_spanning_forest.hpp
---

# 最小全域森の共通入口

`#include "graph_basic/minimum_spanning_forest.hpp"` は [Kruskal](kruskal.md) と [Prim](prim.md) の両方を読み込みます。必要な側だけなら個別のヘッダを使えます。以前の `graph/minimum_spanning_forest.hpp` もこの入口へ転送します。

```cpp
#include <cassert>
#include "graph_basic/minimum_spanning_forest.hpp"
int main() {
    cp::Graph<> g(2); g.add_edge(0,1,5);
    assert(cp::kruskal(g).cost==5 && cp::prim(g).cost==5);
}
```

使用条件・計算量・使用する問題・図解は各関数の説明へ、戻り値は [共通結果型](spanning_forest.md) へまとめています。個別入口と共通入口の同時 include、複数翻訳単位、既存 graph/tree の混在を [検証](README.md#検証) します。[参考資料](references.md)。
