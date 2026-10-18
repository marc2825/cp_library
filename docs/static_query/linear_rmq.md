---
title: Linear RMQ
documentation_of: //static_query/linear_rmq.hpp
---

# Linear RMQ

log N/2程度の小ブロック内の単調stackをbitmaskにし、端のRMQをbit操作で答えます。ブロック最小値だけのSparse Tableは全体で線形サイズです。

## 使用例

```cpp
#include <iostream>
#include "static_query/linear_rmq.hpp"
int main() {
    cp::LinearRMQ<int> s({4,1,1,3});
    std::cout << s.argmin(0,4) << "\n";
}
```

出力: `1`。

## APIと使用条件

`LinearRMQ<T,Compare>(vector,comp)`、argmin(l,r)、size。

同値は最左、空区間は-1。静的、比較器は厳密弱順序。int添字と64bit wordの計算をO(1)とするword RAM上の評価です。

## 計算量

構築O(N)、領域O(N) machine word、検索O(1)。 演算・比較・値のコピーが O(1) の場合です。文字列などを格納すると、その処理時間・領域が加わります。

## 使用する問題と検証

[Library Checker: Static RMQ](https://judge.yosupo.jp/problem/staticrmq)（Sqrt Treeではminモノイドを指定）。空区間、重複、全負値、非可換な文字列の順序を比較します。

`python3 verify/run_b_1_6.py`（`--sanitize` 対応）で単独／重複include、debug/releaseの比較検証、上の使用例を確認します。検証範囲の詳細は[追加分の一覧](../b_1_6/README.md)。オンライン提出・ACは未確認です。

[操作できる図解](../../visualizations/b_1_6/index.html#linear_rmq) / [カテゴリ索引](README.md) / [参考資料と設計判断](../b_1_6/references.md#range) / [図解の簡略化](../../visualizations/b_1_6/README.md)
