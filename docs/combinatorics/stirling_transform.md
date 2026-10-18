---
title: Stirling Transform
documentation_of: //combinatorics/stirling_transform.hpp
---

# Stirling Transform

[カテゴリ索引](README.md) / [B項目の対応表](../b_12_15/README.md) / [操作できる図解](../../visualizations/b_12_15/index.html#stirling_transform) / [実装](../../combinatorics/stirling_transform.hpp) / [一次資料と設計](../b_12_15/references.md)

第2種Stirling数による三角変換と、符号付き第1種による逆変換を提供します。

## usage

```cpp
#include <iostream>
#include "combinatorics/stirling_transform.hpp"
int main() {
    auto b=cp::stirling_transform(std::vector<long long>{1,1,1,1,1}); std::cout << b[4] << '\n';
}
```

出力:

```text
15
```

## API・使用条件

`stirling_transform(vector<long long> a,modulus=998244353,inverse=false)`。b[n]=Σ_{k<=n}S(n,k)a[k]。inverse=trueはs(n,k)。出力は同じ長さ、空なら空。

## 仕組み・境界条件

既存Stirling表を共有します。逆元を要求しないため正の合成数法・法1も可。入力の負数を正規化し、mod加算・積は既存の安全な演算を使用。階乗基底変換は転置方向が異なるので専用APIを使います。

## 計算量

O(N²)時間・領域。 標準整数の演算をO(1)として数えています。64bitの汎用mod乗算は最大O(log m)の加算を使うため、それを使う処理は別途その係数が掛かります。

## 使用する問題・検証

集合分割で各block数kに重みa[k]を付けた個数。 [動く小問題](../../examples/combinatorics/stirling_transform_b.cpp)。

`python3 verify/run_b_12_15.py` / `--sanitize`。[独立した比較方法](../b_12_15/README.md#ローカル検証)に従い、全列挙・定義からの直接計算・Python任意精度と比較します。使用例は文書中のコードと一致を検査してコンパイル・実行します。オンライン提出・ACは未確認です。図解の小入力化と代替計算は[図解の範囲](../../visualizations/b_12_15/README.md)に明記しています。
