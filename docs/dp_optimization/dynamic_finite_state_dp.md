---
documentation_of: //dp_optimization/dynamic_finite_state_dp.hpp
---

# 遷移を更新できる有限状態DP

[P-22の対応表・検証](../dp_extensions/README.md) / [操作できる図解](../../visualizations/dp_extensions/index.html#dynamic_finite_state_dp) / [ヘッダ](../../dp_optimization/dynamic_finite_state_dp.hpp)

`DynamicFiniteStateDP(states, transitions)` は位置ごとのk×k遷移行列を並べたDPです。`Matrix=vector<vector<optional<long long>>>`、行は遷移前、列は遷移後、nulloptは遷移不可。負費用も許容します。k>0、位置数は固定（0も可）。

`set(position,matrix)` が一つの遷移を置換。`product(l,r)` は半開区間のmin-plus積、`all_product()`は全体のconst参照です。空区間は対角0・他nulloptの単位行列。結合の順序は左から右で、非可換性を保持します。

`restore(l,r,source,target)` は`{cost,states}`、到達不能はnullopt。状態列は長さr-l+1。同点なら各段階で先に調べる小さい直前状態を採用します。復元だけは葉を順にDPし、O((r-l)k²)時間・O((r-l)k)領域です。

segment treeによる構築O(nk³)、set/product O(k³ log(n+1))、all_productの参照O(1)、領域O(nk²)。setは更新経路を先に計算してから反映し、整数超過時に元の状態を保持します。全ての評価される途中和がlong longに収まることが必要で、超過はoverflow_error。添字・形は実行時検査します。

用途: 色数固定の隣接制約付き最小費用、有限オートマトン上の列DP、品物変更後の最適状態列。任意のDP全体を動的化するAPIではなく、固定状態数の線形遷移列を対象とします。更新を含む6,000段階で逐次DP・復元と比較します。

## 使用例

```cpp
#include <iostream>
#include "dp_optimization/dynamic_finite_state_dp.hpp"
int main() {
    using Matrix = cp::DynamicFiniteStateDP::Matrix;
    cp::DynamicFiniteStateDP dp(2, {Matrix{{0,3},{2,1}}, Matrix{{1,0},{4,2}}});
    std::cout << *dp.all_product()[0][1] << ' ';
    dp.set(0, Matrix{{2,5},{2,1}});
    std::cout << *dp.all_product()[0][1] << '\n';
    auto path = dp.restore(0,2,0,1);
    for (std::size_t i = 0; i < path->states.size(); ++i) std::cout << (i ? " " : "") << path->states[i];
    std::cout << '\n';
}
```

出力:

```text
0 2
0 0 1
```

## 検証・参考資料

`make verify-dp-extensions` で単独・重複include、Debug/NDEBUG、使用例、独立した素朴解との比較を実行します。詳細と実行記録は[14項目の検証](../dp_extensions/README.md#検証)、理論・既存実装との関係は[一次資料・設計判断](../dp_extensions/references.md)を参照。オンライン提出ACは未確認です。
