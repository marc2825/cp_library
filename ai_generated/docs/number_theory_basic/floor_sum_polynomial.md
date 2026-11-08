---
title: 多項式Floor Sum・万能Euclid
documentation_of: //number_theory_basic/floor_sum_polynomial.hpp
---

# 多項式Floor Sum・万能Euclid（N-04）

[カテゴリ索引](README.md) / [N全項目と検証](../number_theory_extensions/README.md) / [操作できる図解](../../visualizations/number_theory_extensions/index.html#floor_sum_polynomial)

## API・条件

`floor_sum_moments(n,m,a,b,D,max_degree=12)` は `[p][q]=Σ_{0≤i<n} i^p floor((a*i+b)/m)^q`、p+q≤DのBigInt表を返します。範囲外のセルは0。0^0=1。n≥0,m>0、a,bは符号付きBigInt。次数予算超過およびD>64はlength_error。`floor_sum_big(n,m,a,b)` は通常和の任意精度版です。`floor_word_product(n,m,a,b,U,R,e,op)` は非負a,bと任意モノイドを受け、`U^floor(b/m) R` に続き i=1..n−1 の `U^(floor((ai+b)/m)−floor((a(i−1)+b)/m)) R` を連結します。n=0はe。非可換可、逆元不要、opは左から右です。

## 使用例

[そのままコンパイルできるソース](../../examples/number_theory_basic/floor_sum_polynomial.cpp)。C++17、`namespace cp`。例はリポジトリのルートをinclude pathに指定します。

```cpp
#include <iostream>
#include "number_theory_basic/floor_sum_polynomial.hpp"

int main() {
    const auto moments = cp::floor_sum_moments(6, 5, 3, 1, 2);
    std::cout << moments[0][1] << ' ' << moments[1][1] << ' ' << moments[0][2] << '\n';
}
```

出力:

```text
8 31 18
```

## 原理・計算量

傾きの整数部をRへ吸収し、残りの水平/垂直方向を交換するEuclid再帰で操作列を圧縮します。単一元の冪は二分累乗。L=2+n+m+|a|+|b|として保守的にO(log² L)モノイド演算、再帰深さO(log L)。momentsは点の平行移動を二項展開するモノイドで、O((D+1)⁴ log² L)任意精度演算、領域O((D+1)² log L)。通常floor_sum_bigはO(log L)演算です。

BigInt / BigRationalを使用するヘッダにはBoostのヘッダが必要です。記載した任意精度演算回数とビット計算量は区別してください。

## 使用する問題

格子点の重み付き総和、床関数の二乗・混合moment、周期的操作の行列積。

## 検証・参考資料

`make verify-number-theory-extensions` のDebug/Release、libstdc++ debug mode、ASan/UBSan/LeakSanitizerを対象にしています。小入力の独立な全探索・素朴式、空/不正入力・予算境界、大引数を確認します。対象別の比較方法と実行結果は[検証一覧](../number_theory_extensions/README.md#検証)を参照。オンライン提出ACは未確認です。

[一次資料・共有部品・設計判断](../number_theory_extensions/references.md)。外部ソースコードの転載はせず、既存の共通部品と数学的恒等式から実装しています。
