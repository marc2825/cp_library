---
title: Polynomial Mod Pow
documentation_of: //fps/polynomial_mod_pow.hpp
---

# Polynomial Mod Pow

多項式環の剰余として非負整数冪を計算します。

[カテゴリ索引](README.md) / [今回の一覧](../b_16_20/README.md) / [操作できる図解](../../visualizations/b_16_20/index.html#polynomial_mod_pow) / [一次資料と設計判断](../b_16_20/references.md)

## API・使用条件

`polynomial_mod_pow<Mod,Root>(a,uint64 exponent,modulus)`。法多項式は非零、定数法の結果は空。

二分累乗ごとに剰余を取ります。指数0でも法を検査。既存polynomial_divmodの有効長上限Lは全中間積にも適用されるので、安全な十分条件は|a|≤L、2D−1≤L、|modulus|≤L。出力はtrimされます。

## usage

```cpp
#include <iostream>
#include <vector>
#include "../../fps/polynomial_mod_pow.hpp"

int main() {
    auto c = cp::polynomial_mod_pow(std::vector<int>{0, 1}, 5, std::vector<int>{-1, -1, 1});
    std::cout << c[0] << " " << c[1] << "\n";
}
```

出力:

```text
3 5
```

[コンパイルして動く例](../../examples/fps/polynomial_mod_pow_b.cpp)。

## 計算量

法次数DとしてO(M(D)log(exponent+1))時間、O(D)領域。最初のaの剰余費用は別。

## 使用する問題・検証範囲

[Pow of Formal Power Series](https://judge.yosupo.jp/problem/pow_of_formal_power_series)の法x^n版や、線形漸化式の特性多項式による還元。

小指数の繰り返し積と剰余、uint64最大指数、定数法を比較。 `python3 verify/run_b_16_20.py` / `--sanitize`。使用例とこのusageの一致、単独・重複include、debug/releaseを確認します。オンライン提出・AC、全ジャッジ最大制約での時間内完了は未確認です。
