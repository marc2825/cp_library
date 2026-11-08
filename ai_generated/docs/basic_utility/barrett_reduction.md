---
title: Barrett Reduction
documentation_of: //basic_utility/barrett_reduction.hpp
---

# Barrett Reduction

32bitの法について、逆数の高位積から商を近似し、1回の補正で正しい余りを得ます。繰り返す剰余のための公開部品です。

## 使用例

```cpp
#include <iostream>
#include "basic_utility/barrett_reduction.hpp"
int main() {
    cp::BarrettReduction reducer(1000000007);
    std::cout << reducer.reduce(1000000009ULL) << " " << reducer.multiply(1000000006, 1000000006) << "\n";
}
```

出力:

```text
2 1
```

## APIと使用条件

- `BarrettReduction(uint32_t m)`：`m>0`。0は`invalid_argument`。
- `mod()`：法を取得。
- `reduce(uint64_t x)`：`x % m`。**uint64全域**に対応します。
- `multiply(uint32_t a,uint32_t b)`：数学的な`a*b % m`。`a,b`が`m`以上でも構いません。

`m>1`で`reciprocal=ceil(2^64/m)`を構築します。`q=floor(x*reciprocal/2^64)`は真の商か、それより1大きい値です。`q*m>x`なら法を足して補正します。**`q*m`の中間値も128bit**にすることで、uint64全域で正しく動作します。法1は常に0です。構築・各演算O(1)時間、O(1)領域。GCC/Clangの128bit整数を使います。

## 使用する問題と検証

実行時法の積やDPで同じ剰余を繰り返す場面。`DynamicModInt`の乗算から使用します。独立した`uint64 % uint32`・Python任意精度整数との比較を行います。

`python3 verify/run_foundation.py` / `--sanitize`（`make verify-foundation`）で単独・重複include、全カテゴリとの混在、複数翻訳単位、Debug/Release、使用例、独立した比較を検証します。オンライン提出・ACは未確認です。[検証範囲](foundation.md)。

[操作できる図解](../../visualizations/foundation/index.html#barrett_reduction) / [図解の範囲](../../visualizations/foundation/README.md) / [カテゴリ索引](README.md) / [一次資料・設計判断](references.md)
