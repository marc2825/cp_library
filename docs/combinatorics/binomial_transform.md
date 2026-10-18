---
title: 二項変換
documentation_of: //combinatorics/binomial_transform.hpp
---

# 二項変換

[カテゴリ索引](README.md) / [B項目の対応表](../b_12_15/README.md) / [操作できる図解](../../visualizations/b_12_15/index.html#binomial_transform) / [実装](../../combinatorics/binomial_transform.hpp) / [一次資料と設計](../b_12_15/references.md)

二項係数で畳み込む三角変換と、交代符号による逆変換です。

## usage

```cpp
#include <iostream>
#include "combinatorics/binomial_transform.hpp"
int main() {
    auto b=cp::binomial_transform(std::vector<long long>{1,2,3,4}); std::cout << b[3] << '\n';
}
```

出力:

```text
20
```

## API・使用条件

`binomial_transform(a,modulus=998244353,inverse=false)`。b[n]=Σ C(n,k)a[k]、逆は(-1)^(n−k)。長さ保持、空は空。

## 仕組み・境界条件

PascalBinomialを共有するので合成数法・法1にも対応。素数法でのNTTによる高速な二項変換ではなく、一般の正の法で動く表方式です。

## 計算量

O(N²)時間・領域。 標準整数の演算をO(1)として数えています。64bitの汎用mod乗算は最大O(log m)の加算を使うため、それを使う処理は別途その係数が掛かります。

## 使用する問題・検証

部分集合のサイズ別の重み和、EGFのexp因子による変換。 [動く小問題](../../examples/combinatorics/binomial_transform_b.cpp)。

`python3 verify/run_b_12_15.py` / `--sanitize`。[独立した比較方法](../b_12_15/README.md#ローカル検証)に従い、全列挙・定義からの直接計算・Python任意精度と比較します。使用例は文書中のコードと一致を検査してコンパイル・実行します。オンライン提出・ACは未確認です。図解の小入力化と代替計算は[図解の範囲](../../visualizations/b_12_15/README.md)に明記しています。
