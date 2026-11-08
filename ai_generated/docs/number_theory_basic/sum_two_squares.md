---
title: 二平方数和の全表示
documentation_of: //number_theory_basic/sum_two_squares.hpp
---

# 二平方数和の全表示（N-24）

[カテゴリ索引](README.md) / [N全項目と検証](../number_theory_extensions/README.md) / [操作できる図解](../../visualizations/number_theory_extensions/index.html#sum_two_squares)

## API・条件

`sum_two_squares(uint64 n,max_states=1000000)` はx²+y²=nとなる非負x≤yのpairを辞書順・重複なしで返します。n=0は(0,0)。解なしは空。max_statesは重複除去前のGaussian候補数の上限で、超過はlength_error。

## 使用例

[そのままコンパイルできるソース](../../examples/number_theory_basic/sum_two_squares.cpp)。C++17、`namespace cp`。例はリポジトリのルートをinclude pathに指定します。

```cpp
#include <iostream>
#include "number_theory_basic/sum_two_squares.hpp"

int main() {
    for (auto [x, y] : cp::sum_two_squares(65)) std::cout << x << ' ' << y << '\n';
}
```

出力:

```text
1 8
4 7
```

## 原理・計算量

素因数p≡3 mod4の指数が奇数なら不可。p≡1 mod4は原始根から√−1を作りGaussian gcdでp=x²+y²に分解し、共役へ指数を分配します。候補数K=∏_{p≡1 mod4}(e_p+1)。分解/原始根探索を除きO(K log n + K log K)程度の任意精度演算、領域O(K)。内部BigIntでuint64の二乗和overflowを避けます。

BigInt / BigRationalを使用するヘッダにはBoostのヘッダが必要です。記載した任意精度演算回数とビット計算量は区別してください。

## 使用する問題

整数格子上の円周点、二乗ノルムの全表現。

## 検証・参考資料

`make verify-number-theory-extensions` のDebug/Release、libstdc++ debug mode、ASan/UBSan/LeakSanitizerを対象にしています。小入力の独立な全探索・素朴式、空/不正入力・予算境界、大引数を確認します。対象別の比較方法と実行結果は[検証一覧](../number_theory_extensions/README.md#検証)を参照。オンライン提出ACは未確認です。

[一次資料・共有部品・設計判断](../number_theory_extensions/references.md)。外部ソースコードの転載はせず、既存の共通部品と数学的恒等式から実装しています。
