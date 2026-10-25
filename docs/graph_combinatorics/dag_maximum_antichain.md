---
title: DAGの最大反鎖
documentation_of: //graph_combinatorics/dag_maximum_antichain.hpp
---

# DAGの最大反鎖

到達可能性を順序関係とするDAGで、互いに到達しない頂点の最大集合を返します。厳密な推移閉包に二部matchingを適用し、Dilworthの定理から最小鎖分割も復元します。

[操作できる図解](../../visualizations/graph_extensions/index.html#dag_maximum_antichain) / [15項目の検証](../graph_extensions/README.md) / [実装](../../graph_combinatorics/dag_maximum_antichain.hpp)

## usage

```cpp
#include <iostream>
#include "graph_combinatorics/dag_maximum_antichain.hpp"
int main() {
    cp::Graph<int> g(5,cp::GraphKind::Directed);
    g.add_edge(0,2); g.add_edge(1,2); g.add_edge(2,3); g.add_edge(2,4);
    auto r=cp::dag_maximum_antichain(g);
    std::cout << r.antichain.size() << ' ' << r.chains.size() << '\n';
}
```

出力:

```text
2 2
```

## API・使用条件

`dag_maximum_antichain(g)`。`antichain`は頂点列、`chains`は全頂点を1回ずつ含む最小本数の鎖。反鎖サイズと鎖数は等しくなります。

有向DAGが必要で、無向・閉路ありはinvalid_argument。空グラフ可。鎖の連続要素は到達可能ですが、元の辺で直接接続されるとは限りません。鎖を元の経路に展開したときの内部頂点素性は保証しません。元辺のpath coverとは区別します。

## 計算量

推移閉包の辺数をR、W=ceil(V/64)とするとO((V+E)W+V²+(V+R)√(V+1))時間、O(VW+V+R)空間。

## 使用する問題・検証

依存関係の下で同時実行できる最大の作業集合、半順序の幅。

[動く使用例](../../examples/graph_combinatorics/dag_maximum_antichain.cpp)と[独立比較テスト](../../verify/graph_extensions/)を`make verify-graph-extensions`で実行します。小規模全探索、復元した頂点・辺ID、最適値、境界・不正入力を確認します。オンラインACは未確認です。

[一次資料・設計判断](../graph_extensions/references.md) / [カテゴリ索引](README.md)。
