---
documentation_of: //dp_optimization/exact_cover.hpp
---

# Exact Cover・Algorithm X・Dancing Links

[P-04の対応表・検証](../dp_extensions/README.md) / [操作できる図解](../../visualizations/dp_extensions/index.html#exact_cover) / [ヘッダ](../../dp_optimization/exact_cover.hpp)

`ExactCover(primary, secondary=0)` の列0..primary-1は丁度1回、後続secondary列は高々1回選ぶ制約です。`add_row(columns)` が0始まりの行IDを返します。行は重複のない有効な列番号と1個以上のprimary列を必要とします。空行・secondaryだけの行は拒否。内容が同じ別行は別の解として数えます。

`solve()` は一解の行ID列、解なしはnullopt。`enumerate(callback, limit=SIZE_MAX)` は全解を通知し、callbackがfalseまたは通知回数がlimitに達したら止まります。返値は通知回数。limit=0は通知なし、primary=0は空解1個です。callbackの行列は呼び出し中だけ参照可能。

最小サイズの未被覆primary列を選び、双方向リンクをcover/uncoverして探索します。構築領域O(列数+全行の1の数)。探索は最悪指数時間で、列選択走査と取り外したノード数に比例。再帰深さはprimary数以下です。全列数+1と全ノード数はint範囲に制限し、巨大な再帰深さは避けてください。

solve/enumerateは毎回作業コピーを使い、早期終了・callback例外後も元の問題を再利用できます。用途: 数独、敷き詰め、N-Queensの対角線をsecondaryにした列挙。行部分集合全探索と8-Queens=92を検証します。

## 使用例

```cpp
#include <iostream>
#include "dp_optimization/exact_cover.hpp"
int main() {
    cp::ExactCover cover(3);
    cover.add_row({0,1}); cover.add_row({2});
    cover.add_row({0}); cover.add_row({1,2});
    std::cout << cover.enumerate([](const auto&) { return true; });
    auto solution = cover.solve();
    for (int row : *solution) std::cout << ' ' << row;
    std::cout << '\n';
}
```

出力:

```text
2 0 1
```

## 検証・参考資料

`make verify-dp-extensions` で単独・重複include、Debug/NDEBUG、使用例、独立した素朴解との比較を実行します。詳細と実行記録は[14項目の検証](../dp_extensions/README.md#検証)、理論・既存実装との関係は[一次資料・設計判断](../dp_extensions/references.md)を参照。オンライン提出ACは未確認です。
