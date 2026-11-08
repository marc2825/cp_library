---
documentation_of: //dp_optimization/bidirectional_search.hpp
---

# 双方向状態空間BFS

[P-07の対応表・検証](../dp_extensions/README.md) / [操作できる図解](../../visualizations/dp_extensions/index.html#bidirectional_search) / [ヘッダ](../../dp_optimization/bidirectional_search.hpp)

`bidirectional_bfs(start,target,forward,reverse,hash={},equal={})` は単位費用の有向状態空間を両端から探索します。forward(u)が元のu→vを`{v,1,action}`で返すとき、reverse(v)は同じ辺を`{u,1,action}`で返してください。逆向きの操作IDではなく元の操作IDを渡すため、結果のactionsをそのまま順向きに実行できます。

小さいfrontierを**層全体**で展開し、相手側の訪問済み集合との合流候補の最短を選びます。一層の途中で最初に見つけた交点を即採用しません。両側とも未探索部分へ既知の深さを一層進めるため、合流した最小距離が最短です。

結果は共通`StateSearchResult`、到達不能はnullopt、start==targetは費用0・状態1個です。cost!=1はinvalid_argument。逆遷移の正しさは利用者の前提です。有限状態空間で時間O(V+E)、領域O(V)、hashは期待O(1)。用途: 逆操作を生成できるパズルや文字列変換。非対称な有向遷移600件で通常BFSと距離・復元を比較します。

## 使用例

```cpp
#include <iostream>
#include "dp_optimization/bidirectional_search.hpp"
int main() {
    std::vector<std::vector<cp::StateSearchEdge<int>>> forward(5), reverse(5);
    std::vector<std::pair<int,int>> edges{{0,1},{0,2},{1,3},{2,3},{3,4}};
    for (int i = 0; i < int(edges.size()); ++i) {
        auto [u,v] = edges[i]; forward[u].push_back({v,1,i}); reverse[v].push_back({u,1,i});
    }
    auto result = cp::bidirectional_bfs(0,4,[&](int v) { return forward[v]; },[&](int v) { return reverse[v]; });
    std::cout << result->cost;
    for (int v : result->states) std::cout << ' ' << v;
    std::cout << '\n';
}
```

出力:

```text
3 0 1 3 4
```

## 検証・参考資料

`make verify-dp-extensions` で単独・重複include、Debug/NDEBUG、使用例、独立した素朴解との比較を実行します。詳細と実行記録は[14項目の検証](../dp_extensions/README.md#検証)、理論・既存実装との関係は[一次資料・設計判断](../dp_extensions/references.md)を参照。オンライン提出ACは未確認です。
