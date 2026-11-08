---
title: Ordered Setの共通入口
documentation_of: //static_query/ordered_set.hpp
---

# Ordered Setの共通入口

3.4から3.5の順位付き集合を利用するforward include。

## API・使用条件

cp::OrderedSet<Key,Compare>。実体と契約はordered_structure/ordered_set.hppと同じ。

実装は共有し、座標圧縮を不要にする。

C++17、namespace cp。特記以外は0始まり・半開区間。前提条件は呼出し側が守り、添字/範囲の違反はdebug assertで検査します。整数の全中間値とコンテナの添字が使用型に収まることが必要です。計算量は要素の演算/比較/コピーをO(1)としたものです。

## Usage

```cpp
#include <iostream>
#include "static_query/ordered_set.hpp"
int main() {
    cp::OrderedSet<int> s; s.insert(9); s.insert(2);
    std::cout<<*s.kth(0)<<"\n";
}
```

出力: `2`。

## 計算量

各操作期待O(log(N+1))。


## 問題例・検証・参考資料

用途に対応する[問題例と一次資料](../a_priority/references.md)を参照。`python3 verify/run_a_priority.py`（`--sanitize`対応）で単独/重複include、複数翻訳単位、上記使用例と独立した素朴解の差分検証を行います。[検証範囲](../a_priority/README.md)。オンラインACは未確認です。

[操作できる図解](../../visualizations/a_priority/index.html#ordered_set) / [図の範囲](../../visualizations/a_priority/README.md) / [カテゴリ索引](README.md) / [動く例](../../examples/static_query/ordered_set.cpp)
