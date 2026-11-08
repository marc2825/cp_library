---
title: 負閉路の抽出
documentation_of: //graph_basic/negative_cycle.hpp
---

# 負閉路の抽出

Bellman–FordのV回目の緩和から親をV回辿り、負閉路の内部へ入って頂点列と元辺IDを取り出します。影響範囲を返す既存Bellman–Fordと併用できます。

[操作できる図解](../../visualizations/graph_extensions/index.html#negative_cycle) / [15項目の検証](../graph_extensions/README.md) / [実装](../../graph_basic/negative_cycle.hpp)

## usage

```cpp
#include <iostream>
#include "graph_basic/negative_cycle.hpp"
int main() {
    cp::Graph<long long> g(4,cp::GraphKind::Directed);
    g.add_edge(0,1,2); g.add_edge(2,3,-3); g.add_edge(3,2,1);
    auto cycle=cp::find_negative_cycle(g);
    std::cout << bool(cycle) << ' ' << cp::optimization_to_string(cycle->weight)
              << ' ' << bool(cp::find_negative_cycle(g,0)) << '\n';
}
```

出力:

```text
1 -2 0
```

## API・使用条件

`find_negative_cycle(g,source=-1)`。既定は全成分、指定時はsourceから到達可能な範囲。存在しなければnullopt。結果は閉じた`vertices`、`edge_ids`、128bitの負の`weight`。

bool以外64bit以下整数重み。有向/無向、loop、平行辺、非連結可。無向負辺は同じ元辺IDを往復する長さ2の負閉路になり得ます。sourceの不正値はout_of_range。空グラフを全成分指定で探索するとnullopt。距離は128bitで保持し飽和による見逃しを避けます。

## 計算量

O(V(V+E))時間、O(V)追加空間。

## 使用する問題・検証

負費用循環の証明、制約の矛盾箇所の提示。

[動く使用例](../../examples/graph_basic/negative_cycle.cpp)と[独立比較テスト](../../verify/graph_extensions/)を`make verify-graph-extensions`で実行します。小規模全探索、復元した頂点・辺ID、最適値、境界・不正入力を確認します。オンラインACは未確認です。

[一次資料・設計判断](../graph_extensions/references.md) / [カテゴリ索引](README.md)。
