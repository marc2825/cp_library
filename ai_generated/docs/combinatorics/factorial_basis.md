---
title: 階乗べきとの基底変換
documentation_of: //combinatorics/factorial_basis.hpp
---

# 階乗べきとの基底変換

[カテゴリ索引](README.md) / [B項目の対応表](../b_12_15/README.md) / [操作できる図解](../../visualizations/b_12_15/index.html#factorial_basis) / [実装](../../combinatorics/factorial_basis.hpp) / [一次資料と設計](../b_12_15/references.md)

通常の冪x^kと下降階乗(x)_k、上昇階乗x^(overline k)の係数列を相互変換します。

## usage

```cpp
#include <iostream>
#include "combinatorics/factorial_basis.hpp"
int main() {
    auto b=cp::power_to_factorial_basis(std::vector<long long>{0,0,1});
    std::cout << b[1] << ' ' << b[2] << '\n';
}
```

出力:

```text
1 1
```

## API・使用条件

`power_to_factorial_basis(a,modulus=998244353,rising=false)`、`factorial_basis_to_power(...)`。rising=trueが上昇。入出力は次数の昇順、同じ長さ、空は空。

## 仕組み・境界条件

冪→下降に第2種、下降→冪に符号付き第1種Stirling数を使用します。上昇は対応する符号を切替。逆元不要で合成数法可。aの添字方向がStirling Transformと転置関係にある点に注意してください。

## 計算量

O(N²)時間・領域。 標準整数の演算をO(1)として数えています。64bitの汎用mod乗算は最大O(log m)の加算を使うため、それを使う処理は別途その係数が掛かります。

## 使用する問題・検証

多項式の有限差分、階乗べきの係数、通常の冪への戻し。 [動く小問題](../../examples/combinatorics/factorial_basis_b.cpp)。

`python3 verify/run_b_12_15.py` / `--sanitize`。[独立した比較方法](../b_12_15/README.md#ローカル検証)に従い、全列挙・定義からの直接計算・Python任意精度と比較します。使用例は文書中のコードと一致を検査してコンパイル・実行します。オンライン提出・ACは未確認です。図解の小入力化と代替計算は[図解の範囲](../../visualizations/b_12_15/README.md)に明記しています。
