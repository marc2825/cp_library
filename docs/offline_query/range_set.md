---
title: Range Set
documentation_of: //offline_query/range_set.hpp
---

# Range Set

半開区間の動的な和集合。追加で併合、削除で分割。

## API・使用条件

RangeSet<T=long long>、insert(l,r)、erase(l,r)、contains(x)、containing(x)->optional<pair>、covers(l,r)、intervals()、interval_count/empty/clear。

l<=r。接する区間は併合、空の追加/削除は何もしない、空のcoversはtrue。番兵と±1を使わず整数極値も可。Tはコピー可能かつ<で順序付けできればよい。

C++17、namespace cp。特記以外は0始まり・半開区間。前提条件は呼出し側が守り、添字/範囲の違反はdebug assertで検査します。整数の全中間値とコンテナの添字が使用型に収まることが必要です。計算量は要素の演算/比較/コピーをO(1)としたものです。

## Usage

```cpp
#include <iostream>
#include "offline_query/range_set.hpp"
int main() {
    cp::RangeSet<> s; s.insert(1,8); s.erase(3,5);
    for(auto [l,r]:s.intervals()) std::cout<<l<<":"<<r<<" ";
    std::cout<<"\n";
}
```

出力: `1:3 5:8`。

## 計算量

K区間、今回変更する区間数tに対し更新O(log(K+1)+t)、検索O(log(K+1))、列挙/領域O(K)。


## 問題例・検証・参考資料

用途に対応する[問題例と一次資料](../a_priority/references.md)を参照。`python3 verify/run_a_priority.py`（`--sanitize`対応）で単独/重複include、複数翻訳単位、上記使用例と独立した素朴解の差分検証を行います。[検証範囲](../a_priority/README.md)。オンラインACは未確認です。

[操作できる図解](../../visualizations/a_priority/index.html#range_set) / [図の範囲](../../visualizations/a_priority/README.md) / [カテゴリ索引](README.md) / [動く例](../../examples/offline_query/range_set.cpp)
