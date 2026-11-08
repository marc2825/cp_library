---
title: 区間のマージ・分割
documentation_of: //basic_utility/interval_operations.hpp
---

# 区間のマージ・分割

半開区間の和集合と、任意の点による一つの区間の分割。

## API・使用条件

merge_intervals(vector<pair<T,T>>)、split_interval(l,r,cuts)。l<=r、Tの<で順序付け可能。

空区間は除外し接する区間もまとめる。分割点は重複を除き、端点と区間外を無視。動的な追加/削除はRangeSetを使う。

C++17、namespace cp。特記以外は0始まり・半開区間。前提条件は呼出し側が守り、添字/範囲の違反はdebug assertで検査します。整数の全中間値とコンテナの添字が使用型に収まることが必要です。計算量は要素の演算/比較/コピーをO(1)としたものです。

## Usage

```cpp
#include <iostream>
#include "basic_utility/interval_operations.hpp"
int main() {
    auto a=cp::merge_intervals<int>({{1,4},{3,6},{6,8}});
    for(auto [l,r]:cp::split_interval(a[0].first,a[0].second,{3,5})) std::cout<<l<<":"<<r<<" ";
    std::cout<<"\n";
}
```

出力: `1:3 3:5 5:8`。

## 計算量

K区間/分割点をO(K log K)でソート、O(K)領域。


## 問題例・検証・参考資料

用途に対応する[問題例と一次資料](../a_priority/references.md)を参照。`python3 verify/run_a_priority.py`（`--sanitize`対応）で単独/重複include、複数翻訳単位、上記使用例と独立した素朴解の差分検証を行います。[検証範囲](../a_priority/README.md)。オンラインACは未確認です。

[操作できる図解](../../visualizations/a_priority/index.html#interval_operations) / [図の範囲](../../visualizations/a_priority/README.md) / [カテゴリ索引](README.md) / [動く例](../../examples/basic_utility/interval_operations.cpp)
