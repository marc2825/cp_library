---
title: Polynomial GCD
documentation_of: //fps/polynomial_gcd.hpp
---

# Polynomial GCD

2多項式のmonicな最大公約多項式をEuclid法で求めます。

[カテゴリ索引](README.md) / [今回の一覧](../b_16_20/README.md) / [操作できる図解](../../visualizations/b_16_20/index.html#polynomial_gcd) / [一次資料と設計判断](../b_16_20/references.md)

## API・使用条件

`polynomial_gcd<Mod,Root>(a,b)`→vector<int>。gcd(0,0)は空。入力の有効長≤fps_max_size。

零多項式は空に正規化。非零結果の最高次係数は1。片方が0なら他方をmonic化。根の共通部分だけでなく重複度も扱います。

## usage

```cpp
#include <iostream>
#include <vector>
#include "../../fps/polynomial_gcd.hpp"

int main() {
    auto g = cp::polynomial_gcd(std::vector<int>{-1, 0, 1}, std::vector<int>{1, -2, 1});
    std::cout << g[0] << " " << g[1] << "\n";
}
```

出力:

```text
998244352 1
```

[コンパイルして動く例](../../examples/fps/polynomial_gcd_b.cpp)。

## 計算量

次数上限Dとして最悪O(D M(D))時間、O(D)作業領域。高速half-GCDは実装しません。

## 使用する問題・検証範囲

[GCD of Polynomials](https://judge.yosupo.jp/problem/gcd_of_polynomials)。

共通因子付きランダム積の両方が余り0、monic、零入力を検証。 `python3 verify/run_b_16_20.py` / `--sanitize`。使用例とこのusageの一致、単独・重複include、debug/releaseを確認します。オンライン提出・AC、全ジャッジ最大制約での時間内完了は未確認です。
