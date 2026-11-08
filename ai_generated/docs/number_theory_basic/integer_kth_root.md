---
title: 整数k乗根（数論入口）
documentation_of: //number_theory_basic/integer_kth_root.hpp
---

# 整数k乗根（数論入口）

[カテゴリ索引](README.md) / [B項目の対応表](../b_12_15/README.md) / [操作できる図解](../../visualizations/b_12_15/index.html#integer_kth_root) / [実装](../../number_theory_basic/integer_kth_root.hpp) / [一次資料と設計](../b_12_15/references.md)

3.1で整備済みのuint64整数k乗根を数論カテゴリから共有します。

## usage

```cpp
#include <iostream>
#include "number_theory_basic/integer_kth_root.hpp"
int main() {
    std::cout << cp::integer_kth_root(125,3) << '\n';
}
```

出力:

```text
5
```

## API・使用条件

`integer_kth_root(uint64_t n,unsigned k)`。k>=1、返値はfloor(n^(1/k))。0は0、k=0はinvalid_argument。

## 仕組み・境界条件

浮動小数点を使わず、乗算前にn/xと比較します。UINT64_MAXでも判定があふれません。基本ユーティリティの[実体](../basic_utility/integer_kth_root.md)を再利用します。

## 計算量

O(min(k,64) log(n+1))時間、O(1)領域。 標準整数の演算をO(1)として数えています。64bitの汎用mod乗算は最大O(log m)の加算を使うため、それを使う処理は別途その係数が掛かります。

## 使用する問題・検証

[Kth Root (Integer)](https://judge.yosupo.jp/problem/kth_root_integer)。 [動く小問題](../../examples/number_theory_basic/integer_kth_root_b.cpp)。

`python3 verify/run_b_12_15.py` / `--sanitize`。[独立した比較方法](../b_12_15/README.md#ローカル検証)に従い、全列挙・定義からの直接計算・Python任意精度と比較します。使用例は文書中のコードと一致を検査してコンパイル・実行します。オンライン提出・ACは未確認です。図解の小入力化と代替計算は[図解の範囲](../../visualizations/b_12_15/README.md)に明記しています。
