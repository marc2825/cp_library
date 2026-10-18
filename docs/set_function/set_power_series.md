---
title: SPSのexp / log / inv
documentation_of: //set_function/set_power_series.hpp
---

# SPSのexp / log / inv

互いに交わらない部分集合の畳み込みを積とする集合冪級数を扱います。

[カテゴリ索引](README.md) / [今回の一覧](../b_16_20/README.md) / [操作できる図解](../../visualizations/b_16_20/index.html#set_power_series) / [一次資料と設計判断](../b_16_20/references.md)

## API・使用条件

`sps_inv<Mod>(f)`、`sps_exp<Mod>(f)`、`sps_log<Mod>(f)`→vector<int>。正の2冪長N=2^n、素数Mod。

invはf[0]≠0、expはf[0]=0、logはf[0]=1。f=f0+x_last*f1と分け、expの上半分はexp(f0)*f1、logはf1/f0、invは−f1/f0²。expは集合分割の重み和、logはその逆の整数多項式として定義し、標数≤nでも動きます。Σf^k/k!の表示は階乗が可逆な範囲でのみ使います。

## usage

```cpp
#include <iostream>
#include <vector>
#include "../../set_function/set_power_series.hpp"

int main() {
    auto a = cp::sps_exp(std::vector<int>{0, 2, 3, 5});
    std::cout << a[0] << " " << a[3] << "\n";
}
```

出力:

```text
1 11
```

[コンパイルして動く例](../../examples/set_function/set_power_series_b.cpp)。

## 計算量

O(n² 2^n)時間、O(n 2^n)作業領域。inv/logでは定数回の部分集合積を各分割段で実行します。

## 使用する問題・検証範囲

[Exp of Set Power Series](https://judge.yosupo.jp/problem/exp_of_set_power_series)。

ラベルなし集合分割の全列挙、独立した部分集合積による逆、法2/3と通常法を比較。 `python3 verify/run_b_16_20.py` / `--sanitize`。使用例とこのusageの一致、単独・重複include、debug/releaseを確認します。オンライン提出・AC、全ジャッジ最大制約での時間内完了は未確認です。
