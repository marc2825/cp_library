---
title: 大引数・任意mod二項係数
documentation_of: //combinatorics/prime_power_binomial.hpp
---

# 大引数・任意mod二項係数（N-06）

[カテゴリ索引](README.md) / [N全項目と検証](../number_theory_extensions/README.md) / [操作できる図解](../../visualizations/number_theory_extensions/index.html#prime_power_binomial)

## API・条件

`PrimePowerBinomial(m,max_table_entries=2000000)` は正のuint64法を固定し、`choose(uint64 n,uint64 k)` を繰り返し呼べます。k>nは0、法1も0。各素数冪qについてq+1セルが必要で、その合計が予算を超えればlength_error。例えば10^9+7の表は既定予算では作れません。巨大nのAPIであり、巨大な素数冪テーブルを無制限に確保するAPIではありません。

## 使用例

[そのままコンパイルできるソース](../../examples/combinatorics/prime_power_binomial.cpp)。C++17、`namespace cp`。例はリポジトリのルートをinclude pathに指定します。

```cpp
#include <iostream>
#include "combinatorics/prime_power_binomial.hpp"

int main() {
    cp::PrimePowerBinomial binomial(72);
    std::cout << binomial.choose(100, 30) << '\n';
}
```

出力:

```text
0
```

## 原理・計算量

pを除いた階乗積の周期とLegendreの指数からmod p^eの二項係数を計算し、前計算済みCRT係数で合成します。分解を除く前処理O(Σp^e)、領域も同程度。1 queryはO(Σ(log_p n·log(n+1)+log p^e))回程度のmod演算。指数がe以上ならその成分は0。uint64積のoverflowを避けます。

BigInt / BigRationalを使用するヘッダにはBoostのヘッダが必要です。記載した任意精度演算回数とビット計算量は区別してください。

## 使用する問題

nが64bitで、固定の小さい合成数法に対する多数の組合せ問い合わせ。

## 検証・参考資料

`make verify-number-theory-extensions` のDebug/Release、libstdc++ debug mode、ASan/UBSan/LeakSanitizerを対象にしています。小入力の独立な全探索・素朴式、空/不正入力・予算境界、大引数を確認します。対象別の比較方法と実行結果は[検証一覧](../number_theory_extensions/README.md#検証)を参照。オンライン提出ACは未確認です。

[一次資料・共有部品・設計判断](../number_theory_extensions/references.md)。外部ソースコードの転載はせず、既存の共通部品と数学的恒等式から実装しています。
