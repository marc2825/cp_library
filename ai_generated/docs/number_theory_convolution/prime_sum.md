---
title: 高速素数和
documentation_of: //number_theory_convolution/prime_sum.hpp
---

# 高速素数和（N-25）

[カテゴリ索引](README.md) / [N全項目と検証](../number_theory_extensions/README.md) / [操作できる図解](../../visualizations/number_theory_extensions/index.html#prime_sum)

## API・条件

`prime_sum(N,max_root=5000000)` はN以下の素数の和をBigIntで返します。`prime_sum_table` は全商での和をQuotientPrefix<BigInt>として返します。`prime_sum_sieve<T>(N,prefix,weight,max_root)` は完全乗法的hについてprefix(x)=Σ_{i≤x}h(i)、weight(p)=h(p)を受け、素数上の和を返します。h(1)=1が前提。一般の乗法的関数ではなく、完全乗法的な重みが必要です。

## 使用例

[そのままコンパイルできるソース](../../examples/number_theory_convolution/prime_sum.cpp)。C++17、`namespace cp`。例はリポジトリのルートをinclude pathに指定します。

```cpp
#include <iostream>
#include "number_theory_convolution/prime_sum.hpp"

int main() {
    std::cout << cp::prime_sum(100) << '\n';
}
```

出力:

```text
1060
```

## 原理・計算量

最初に1を除く全整数の重み和を置き、小さい素数pからh(p)*(S(x/p)−S(p−1))を引きます。xは降順、商集合内で参照。prefix/weightがO(1)なら保守的にO(N^(3/4))環演算、領域O(√N)。√Nの予算を検査します。

BigInt / BigRationalを使用するヘッダにはBoostのヘッダが必要です。記載した任意精度演算回数とビット計算量は区別してください。

## 使用する問題

素数の個数だけではなく、総和や完全乗法的重みの素数上の集計。

## 検証・参考資料

`make verify-number-theory-extensions` のDebug/Release、libstdc++ debug mode、ASan/UBSan/LeakSanitizerを対象にしています。小入力の独立な全探索・素朴式、空/不正入力・予算境界、大引数を確認します。対象別の比較方法と実行結果は[検証一覧](../number_theory_extensions/README.md#検証)を参照。オンライン提出ACは未確認です。

[一次資料・共有部品・設計判断](../number_theory_extensions/references.md)。外部ソースコードの転載はせず、既存の共通部品と数学的恒等式から実装しています。
