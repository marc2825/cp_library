---
title: Sqrt Tree
documentation_of: //static_query/sqrt_tree.hpp
---

# Sqrt Tree

平方分割をlog log N層に再帰し、各層でprefix・suffixと完全ブロック間の積を前計算します。左右端の最高相違bitから使う層を定数時間で選びます。

## 使用例

```cpp
#include <iostream>
#include "static_query/sqrt_tree.hpp"
long long op(long long a,long long b) { return a+b; }
long long e() { return 0; }
int main() {
    cp::SqrtTree<long long,op,e> s({1,2,3,4});
    std::cout << s.prod(1,4) << "\n";
}
```

出力: `9`。

## APIと使用条件

`SqrtTree<S,op,e>(vector)`、prod(l,r)、size。

静的な結合的演算、eは両側単位元。可換性・冪等性は不要。空区間はe、0/1要素も可。点更新は提供しません。

## 計算量

構築・領域O(N log log(N+2))、prod O(1)。 演算・比較・値のコピーが O(1) の場合です。文字列などを格納すると、その処理時間・領域が加わります。

## 使用する問題と検証

[Library Checker: Static RMQ](https://judge.yosupo.jp/problem/staticrmq)はminモノイドを指定すれば解けます。検証では、長さ0〜179の配列で、非可換な文字列連結の区間積を空区間を含む全区間で直接計算した値と比較し、131,072要素で構築が線形・問い合わせが2回以内の演算で済むことも確認します。

`python3 verify/run_b_1_6.py`（`--sanitize` 対応）で単独／重複include、debug/releaseの比較検証、上の使用例を確認します。検証範囲の詳細は[追加分の一覧](../b_1_6/README.md)。オンライン提出・ACは未確認です。

[操作できる図解](../../visualizations/b_1_6/index.html#sqrt_tree) / [カテゴリ索引](README.md) / [参考資料と設計判断](../b_1_6/references.md#range) / [図解の簡略化](../../visualizations/b_1_6/README.md)
