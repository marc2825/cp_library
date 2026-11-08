---
title: Static Range MEX
documentation_of: //static_query/static_range_mex.hpp
---

# Static Range MEX（D-03）

固定配列の任意区間に現れない最小の非負整数を返します。集合全体を更新する DynamicMex とは異なり、前処理後の区間問い合わせを扱います。

## API・使用条件

`StaticRangeMex(vector<Integer>)`、`size()`、`mex(l,r)`、`node_count()`。

入力は標準の整数型（符号付き・符号なし、64 bit まで）。負数および n より大きい値は MEX に寄与しません。空区間の答えは 0、全体の長さは INT_MAX 未満です。ノード ID も int に収まる範囲で使用し、上限超過は length_error です。

値域 [0,n+1) 上で、各値の最終出現位置の最小値を持つ永続セグ木を作ります。prefix r の版から「最終出現位置 < l」の最小値を探せば MEX です。未出現位置は -1。長さ n の配列には 0..n の全 n+1 種類は現れないため必ず見つかります。

C++17、namespace cp、0始まり・半開区間。添字・範囲の前提は debug assert で検査します。比較・コピー・集約演算を O(1) として計算量を示します。整数の途中値は使用型に収まることが必要です。

## Usage

```cpp
#include <iostream>
#include <vector>
#include "static_query/static_range_mex.hpp"
int main() {
    cp::StaticRangeMex a(std::vector<int>{0,1,0,3,2});
    std::cout << a.mex(0,5) << " " << a.mex(1,4) << " " << a.mex(2,2) << "\n";
}
```

```text
4 2 0
```

## 計算量

前処理 O(n log(n+1)) 時間・領域、mex O(log(n+1))、size/node_count O(1)。負数・値域外の要素では新規ノードを作りません。

## 問題例・検証・参考資料

例題: 静的配列に対する任意区間 MEX。長さ 0〜8 の三値配列の全区間、空区間、負数、LLONG_MIN/MAX、ULLONG_MAX、20万連続整数を集合による MEX と比較します。

[共通の検証入口・対応範囲](../data_structure_extensions/README.md) / [一次資料と設計判断](../data_structure_extensions/references.md)。オンライン AC は未確認です。

[操作できる図解](../../visualizations/data_structure_extensions/index.html#static_range_mex) / [図解の範囲](../../visualizations/data_structure_extensions/README.md) / [動く例](../../examples/static_query/static_range_mex.cpp) / [カテゴリ索引](README.md)
