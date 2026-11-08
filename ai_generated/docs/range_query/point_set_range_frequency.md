---
title: Point Set Range Frequency
documentation_of: //range_query/point_set_range_frequency.hpp
---

# Point Set Range Frequency（D-05）

点の値をオンラインに上書きし、区間内の指定値の出現回数を返します。更新値を先に列挙する必要はありません。

## API・使用条件

`PointSetRangeFrequency<T,Compare>(values,compare={})`、`set(p,value)`、`get(p)`、`frequency(l,r,x)`、`size()`。

各位置の標準区間に map<T,int> を置きます。点更新は根までの map の旧値を減らして新値を増やし、0 個になったキーを削除します。区間を標準ノードに分けて頻度を合計します。過去の更新値だけで領域は増え続けません。

比較器は厳密弱順序。同値は比較器で決まり、T の `==`・デフォルト構築は不要。コピー・代入可能な T を使います。get の参照は更新や破棄をまたいで保持しないでください（bool の場合は値）。空区間の頻度は 0。n <= INT_MAX。set 内の値の代入・比較・確保は正常終了する使用条件とし、例外後の継続利用は保証しません。

C++17、namespace cp、0始まり・半開区間。添字・範囲の前提は debug assert で検査します。比較・コピー・集約演算を O(1) として計算量を示します。整数の途中値は使用型に収まることが必要です。

## Usage

```cpp
#include <iostream>
#include "range_query/point_set_range_frequency.hpp"
int main() {
    cp::PointSetRangeFrequency<int> a({2,1,2,3});
    std::cout << a.frequency(0,4,2) << "\n";
    a.set(1,2); a.set(3,100);
    std::cout << a.frequency(0,3,2) << " " << a.get(3) << "\n";
}
```

```text
2
3 100
```

## 計算量

構築 O(n log²(n+1))、set・frequency O(log²(n+1))、get/size O(1)、領域 O(n log(n+1))。値を事前圧縮しない決定的な map 版です。

## 問題例・検証・参考資料

Library Checker Point Set Range Frequency。142,000 のランダム更新・問い合わせ、未登録値・負数・独自比較器、20万要素を vector の直接カウントと比較します。

[共通の検証入口・対応範囲](../data_structure_extensions/README.md) / [一次資料と設計判断](../data_structure_extensions/references.md)。オンライン AC は未確認です。

[操作できる図解](../../visualizations/data_structure_extensions/index.html#point_set_range_frequency) / [図解の範囲](../../visualizations/data_structure_extensions/README.md) / [動く例](../../examples/range_query/point_set_range_frequency.cpp) / [カテゴリ索引](README.md)
