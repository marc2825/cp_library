---
title: 最小平均閉路 / Karp
documentation_of: //graph_basic/minimum_mean_cycle.hpp
---

# 最小平均閉路 / Karp

閉路の重み和÷辺数を最小化します。Karpの長さkのwalk DPから厳密な最小比を求め、その比を引いた重みのtight辺に含まれる零閉路を復元します。

[操作できる図解](../../visualizations/graph_extensions/index.html#minimum_mean_cycle) / [15項目の検証](../graph_extensions/README.md) / [実装](../../graph_basic/minimum_mean_cycle.hpp)

## usage

```cpp
#include <iostream>
#include "graph_basic/minimum_mean_cycle.hpp"
int main() {
    cp::Graph<long long> g(3,cp::GraphKind::Directed);
    g.add_edge(0,1,2); g.add_edge(1,2,-4); g.add_edge(2,0,1);
    auto r=cp::minimum_mean_cycle(g);
    if (r) std::cout << cp::optimization_to_string(r->numerator) << ' ' << r->denominator << ' ' << r->edge_ids.size() << '\n';
}
```

出力:

```text
-1 3 3
```

## API・使用条件

`minimum_mean_cycle(g)`。閉路なしはnullopt。結果の`numerator`はOptimizationWide、`denominator`は正のintで既約分数。`vertices`は始点を末尾に繰り返す閉路、`edge_ids`は元辺ID。

有向グラフ、bool以外の64bit以下の整数重み。負辺・多重辺・自己ループ・非連結可。全成分が対象。有向以外はinvalid_argument。比較と中間値は検査付き128bitで、浮動小数点への変換はしません。平均が0以上の閉路も返します。

## 計算量

O(VE+V²)時間、O(V²+E)空間。大きい疎グラフでもV²のDPメモリが必要。

## 使用する問題・検証

繰返し操作の1回あたり最小コスト、周期の平均利得最適化。最短閉路や負閉路判定とは目的関数が異なります。

[動く使用例](../../examples/graph_basic/minimum_mean_cycle.cpp)と[独立比較テスト](../../verify/graph_extensions/)を`make verify-graph-extensions`で実行します。小規模全探索、復元した頂点・辺ID、最適値、境界・不正入力を確認します。オンラインACは未確認です。

[一次資料・設計判断](../graph_extensions/references.md) / [カテゴリ索引](README.md)。
