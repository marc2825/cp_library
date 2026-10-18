---
title: Narayana数
documentation_of: //combinatorics/narayana.hpp
---

# Narayana数

[カテゴリ索引](README.md) / [B項目の対応表](../b_12_15/README.md) / [操作できる図解](../../visualizations/b_12_15/index.html#narayana) / [実装](../../combinatorics/narayana.hpp) / [一次資料と設計](../b_12_15/references.md)

Dyckパスのpeak数などを数えるNarayana数を、既存二項係数表から計算します。

## usage

```cpp
#include <iostream>
#include "combinatorics/narayana.hpp"
int main() {
    cp::FactorialTable table(5); std::cout << cp::narayana(table,5,3) << '\n';
}
```

出力:

```text
20
```

## API・使用条件

`narayana(const FactorialTable& table,n,k)`→long long。N(0,0)=1、それ以外はn>0、1<=k<=n。範囲外は0、table上限外はout_of_range。

## 仕組み・境界条件

N(n,k)=C(n,k)C(n,k−1)/n。FactorialTableのunit条件により1..nも可逆です。法1も既存規約に従い0。

## 計算量

前計算後O(1)個のmod乗算とO(log m)の逆元。 標準整数の演算をO(1)として数えています。64bitの汎用mod乗算は最大O(log m)の加算を使うため、それを使う処理は別途その係数が掛かります。

## 使用する問題・検証

peakがちょうどk個の括弧列／Dyckパスの数。 [動く小問題](../../examples/combinatorics/narayana_b.cpp)。

`python3 verify/run_b_12_15.py` / `--sanitize`。[独立した比較方法](../b_12_15/README.md#ローカル検証)に従い、全列挙・定義からの直接計算・Python任意精度と比較します。使用例は文書中のコードと一致を検査してコンパイル・実行します。オンライン提出・ACは未確認です。図解の小入力化と代替計算は[図解の範囲](../../visualizations/b_12_15/README.md)に明記しています。
