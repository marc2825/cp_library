---
title: Range Majority
documentation_of: //static_query/range_majority.hpp
---

# Range Majority（D-04）

区間の長さの半分を厳密に超えて出現する値を返します。最頻値を返す Range Mode とは異なり、過半数がない場合は nullopt です。

## API・使用条件

`RangeMajority<T,Compare>(values,compare={})`、`candidate(l,r)`、`frequency(l,r,x)`、`majority(l,r)`、`size()`。

候補と確定結果は optional<T>、frequency は int。比較器が両方向 false の値を同値とし、`==` は不要です。空区間は候補・過半数とも nullopt、頻度 0。n <= INT_MAX。

異なる候補を個数分だけ相殺する木を作ります。厳密な過半数は任意の相殺で残るので、候補を出現位置の二分探索で検証します。相殺結果自体は結合的ではありません。**candidate は過半数の証明ではなく、存在しないときにも値を返すことがあります。** 公開のモノイドとはせず、majority が実際の頻度を再確認します。

C++17、namespace cp、0始まり・半開区間。添字・範囲の前提は debug assert で検査します。比較・コピー・集約演算を O(1) として計算量を示します。整数の途中値は使用型に収まることが必要です。

## Usage

```cpp
#include <iostream>
#include "static_query/range_majority.hpp"
int main() {
    cp::RangeMajority<int> a({2,1,2,2,3,1});
    std::cout << a.majority(0,4).value_or(-1) << " " << a.majority(0,6).has_value() << "\n";
    std::cout << a.frequency(1,5,2) << "\n";
}
```

```text
2 0
2
```

## 計算量

前処理 O(n log(n+1))、領域 O(n)。candidate・frequency・majority は O(log(n+1))。比較と T のコピーを O(1) とします。

## 問題例・検証・参考資料

例題: 区間で過半数を得る投票。三値配列の全区間を map の頻度と比較し、ちょうど半数・過半数なし・空・文字列・独自比較器と20万要素を検証します。

[共通の検証入口・対応範囲](../data_structure_extensions/README.md) / [一次資料と設計判断](../data_structure_extensions/references.md)。オンライン AC は未確認です。

[操作できる図解](../../visualizations/data_structure_extensions/index.html#range_majority) / [図解の範囲](../../visualizations/data_structure_extensions/README.md) / [動く例](../../examples/static_query/range_majority.cpp) / [カテゴリ索引](README.md)
