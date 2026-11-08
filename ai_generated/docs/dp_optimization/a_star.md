---
documentation_of: //dp_optimization/a_star.hpp
---

# A* 状態空間探索

[P-05の対応表・検証](../dp_extensions/README.md) / [操作できる図解](../../visualizations/dp_extensions/index.html#a_star) / [ヘッダ](../../dp_optimization/a_star.hpp)

`a_star(start, neighbors, goal, heuristic, hash={}, equal={})` は最小費用の到達解をoptionalで返します。Stateはコピー可能・hash/equality可能、neighbors(state)は`StateSearchEdge<State>{to,cost,action}`の列です。actionは利用者の操作IDで、重複辺も区別できます。

費用は非負long long、heuristicは非負・許容的（真の残り費用以下）・目標で0を前提とします。状態はMarkov的で、遷移とheuristicは探索中に変化しないこと。整合性を満たさないheuristicでも、より短いgが見つかれば再展開します。負値・目標の非零hは検出時にinvalid_argument、g+hなどの整数超過はoverflow_error。許容性自体は自動検査しません。

返値は`cost, states, actions, expanded`。statesは始点・終点を含み、actionsは辺数と同じ長さ、expandedは目標判定後に遷移を展開した回数。到達不能はnulloptです。各改善時に不変の親履歴を保持するので、再展開後も復元が崩れません。

到達状態空間が有限なら非負整数費用で終了します。整合hなら通常O((V+E)log(V+E))、不整合hでは再展開を含む実際の改善数Rと調査辺数XについてO(X+R log R)、領域O(V+R)。hash操作は期待O(1)。無限状態空間では、h=0や費用0の遷移により終了しない場合があります。

用途: パズルや暗黙グラフの最短操作列。Floyd比較、非整合hによる再展開、費用0の循環、復元辺ID、整数極値を検証します。

## 使用例

```cpp
#include <iostream>
#include "dp_optimization/a_star.hpp"
int main() {
    std::vector<std::vector<cp::StateSearchEdge<int>>> edges{
        {{1,5,0},{2,1,1}}, {{3,10,2}}, {{1,1,3}}, {}};
    std::vector<int> h{0,0,11,0};
    auto result = cp::a_star(0, [&](int v) { return edges[v]; },
        [](int v) { return v == 3; }, [&](int v) { return h[v]; });
    std::cout << result->cost;
    for (int v : result->states) std::cout << ' ' << v;
    std::cout << '\n';
}
```

出力:

```text
12 0 2 1 3
```

## 検証・参考資料

`make verify-dp-extensions` で単独・重複include、Debug/NDEBUG、使用例、独立した素朴解との比較を実行します。詳細と実行記録は[14項目の検証](../dp_extensions/README.md#検証)、理論・既存実装との関係は[一次資料・設計判断](../dp_extensions/references.md)を参照。オンライン提出ACは未確認です。
