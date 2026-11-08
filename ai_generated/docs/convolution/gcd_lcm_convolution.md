---
title: GCD / LCM Convolution（共通入口）
documentation_of: //convolution/gcd_lcm_convolution.hpp
---

# GCD / LCM Convolution（共通入口）

約数・倍数の変換を使う3.14の実装を共有します。

[カテゴリ索引](README.md) / [今回の一覧](../b_16_20/README.md) / [操作できる図解](../../visualizations/b_16_20/index.html#gcd_lcm_convolution) / [一次資料と設計判断](../b_16_20/references.md)

## API・使用条件

`gcd_convolution(vector<T>,vector<T>)`、`lcm_convolution(...)`。添字0は未使用、同じ長さ。既存APIと同じ条件。

[GCDの既存説明](../number_theory_convolution/gcd_convolution.md)と[LCMの既存説明](../number_theory_convolution/lcm_convolution.md)を参照。LCMは出力範囲外の組を捨てます。標準整数の途中超過は例外。

## usage

```cpp
#include <iostream>
#include <vector>
#include "../../convolution/gcd_lcm_convolution.hpp"

int main() {
    auto c = cp::gcd_convolution(std::vector<int>{0, 1, 2}, std::vector<int>{0, 3, 4});
    std::cout << c[1] << " " << c[2] << "\n";
}
```

出力:

```text
13 8
```

[コンパイルして動く例](../../examples/convolution/gcd_lcm_convolution_b.cpp)。

## 計算量

長さNでO(N log N)時間、O(N)領域。係数型の演算費用を掛けます。

## 使用する問題・検証範囲

[Gcd Convolution](https://judge.yosupo.jp/problem/gcd_convolution)。

全添字対のgcd/lcmと比較。既存3.14の検証も実行。 `python3 verify/run_b_16_20.py` / `--sanitize`。使用例とこのusageの一致、単独・重複include、debug/releaseを確認します。オンライン提出・AC、全ジャッジ最大制約での時間内完了は未確認です。
