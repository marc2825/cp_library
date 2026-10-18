---
title: 母関数の係数抽出
documentation_of: //linear_recurrence/coefficient_extraction.hpp
---

# 母関数の係数抽出

有理式のprefixと、有限多項式の積の指定次数を抽出します。

[カテゴリ索引](README.md) / [今回の一覧](../b_16_20/README.md) / [操作できる図解](../../visualizations/b_16_20/index.html#coefficient_extraction) / [一次資料と設計判断](../b_16_20/references.md)

## API・使用条件

`generating_function_coefficients<Mod,Root>(P,Q,n)`→P/Qの低次n項。`product_coefficient<Mod,Root>(factors,degree)`→積の指定係数。

Q[0]≠0、n≤L、degree<L。prefixのn=0は分母条件を検査せず空。積の空因子列は1、空多項式を含む積は0。各段で不要な高次を切り、積木状に乗算。遠い有理係数には既存Bostan–Moriを使います。

## usage

```cpp
#include <iostream>
#include <vector>
#include "../../linear_recurrence/coefficient_extraction.hpp"

int main() {
    std::cout << cp::product_coefficient(std::vector<std::vector<int>>{{1, 1}, {1, 1}, {1, 1}}, 2) << "\n";
}
```

出力:

```text
3
```

[コンパイルして動く例](../../examples/linear_recurrence/coefficient_extraction_b.cpp)。

## 計算量

有理prefixはO(M(n))、積は因子数kとして保守的にO(k M(degree+1))時間。領域は入力を除きO(k(degree+1))の保守的上界。

## 使用する問題・検証範囲

多項式の積による数え上げ、有理母関数の初期項生成。

直接積、逆数の係数漸化式と比較。 `python3 verify/run_b_16_20.py` / `--sanitize`。使用例とこのusageの一致、単独・重複include、debug/releaseを確認します。オンライン提出・AC、全ジャッジ最大制約での時間内完了は未確認です。
