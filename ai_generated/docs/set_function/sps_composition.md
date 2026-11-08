---
title: SPSの合成
documentation_of: //set_function/sps_composition.hpp
---

# SPSの合成

有限の1変数多項式を集合冪級数へ代入します。

[カテゴリ索引](README.md) / [今回の一覧](../b_16_20/README.md) / [操作できる図解](../../visualizations/b_16_20/index.html#sps_composition) / [一次資料と設計判断](../b_16_20/references.md)

## API・使用条件

`sps_compose<Mod>(outer,inner)`。outerは昇冪係数、innerは正の2冪長。素数法。

inner[0]≠0も可。outer(c+y)をy^(n+1)未満まで計算し、非定数部分が冪零であることを使います。外側の高次も使用。除算を使わず小標数でも有限多項式の合成として正しいです。

## usage

```cpp
#include <iostream>
#include <vector>
#include "../../set_function/sps_composition.hpp"

int main() {
    auto c = cp::sps_compose(std::vector<int>{1, 2, 1}, std::vector<int>{0, 1, 1, 0});
    std::cout << c[0] << " " << c[3] << "\n";
}
```

出力:

```text
1 2
```

[コンパイルして動く例](../../examples/set_function/sps_composition_b.cpp)。

## 計算量

外側長d、集合の変数数nとしてO(dn+n³2^n)時間、O(n2^n+d)の保守的領域上界。

## 使用する問題・検証範囲

[Polynomial Composite Set Power Series](https://judge.yosupo.jp/problem/polynomial_composite_set_power_series)。

部分集合積を繰り返す独立な多項式評価、非零定数項、小標数を比較。 `python3 verify/run_b_16_20.py` / `--sanitize`。使用例とこのusageの一致、単独・重複include、debug/releaseを確認します。オンライン提出・AC、全ジャッジ最大制約での時間内完了は未確認です。
