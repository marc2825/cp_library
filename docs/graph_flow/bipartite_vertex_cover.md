---
title: 二部最小頂点被覆・最大独立集合
documentation_of: //graph_flow/bipartite_vertex_cover.hpp
---

# 二部最小頂点被覆・最大独立集合

最大マッチングから未対応の左頂点を始点とする交互到達集合Zを求め、(L−Z)∪(R∩Z)を返します。補集合が最大独立集合です。

[操作できる図解](../../visualizations/graph_extensions/index.html#bipartite_vertex_cover) / [15項目の検証](../graph_extensions/README.md) / [実装](../../graph_flow/bipartite_vertex_cover.hpp)

## usage

```cpp
#include <iostream>
#include "graph_flow/bipartite_vertex_cover.hpp"
int main() {
    auto r=cp::bipartite_vertex_cover(3,3,{{0,0},{1,0},{2,1}});
    std::cout << r.matching.cardinality << ' ' << r.left_cover.size()+r.right_cover.size()
              << ' ' << r.left_independent.size()+r.right_independent.size() << '\n';
}
```

出力:

```text
2 2 4
```

## API・使用条件

`bipartite_vertex_cover(L,R,edges)`。辺は左右それぞれの0始まり番号のpair。結果は`matching`、`left_cover/right_cover`、`left_independent/right_independent`、`reachable_left/right`。各頂点集合は昇順。

左右の片側が空でも可。平行辺可。複数の最適解のうち辞書順最小を保証しません。サイズ負値はinvalid_argument、頂点範囲外はout_of_range。元matchingのAPIは変更しません。

## 計算量

O((V+E)√(V+1))時間、O(V+E)空間。

## 使用する問題・検証

作業と資源の組合せの最小監視集合、二部グラフで互いに隣接しない頂点の最大集合。

[動く使用例](../../examples/graph_flow/bipartite_vertex_cover.cpp)と[独立比較テスト](../../verify/graph_extensions/)を`make verify-graph-extensions`で実行します。小規模全探索、復元した頂点・辺ID、最適値、境界・不正入力を確認します。オンラインACは未確認です。

[一次資料・設計判断](../graph_extensions/references.md) / [カテゴリ索引](README.md)。
