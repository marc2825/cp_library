---
title: Gaussian Integer
documentation_of: //number_theory_basic/gaussian_integer.hpp
---

# Gaussian Integer（N-21）

[カテゴリ索引](README.md) / [N全項目と検証](../number_theory_extensions/README.md) / [操作できる図解](../../visualizations/number_theory_extensions/index.html#gaussian_integer)

## API・条件

`GaussianInteger(real,imag=0)` はBigIntの公開成分を持ち、加減乗算、比較、共役 `conjugate()`、`norm()` を提供します。`gaussian_divmod(a,b)` はa=b*q+r、2norm(r)≤norm(b)。商の各成分は最近整数、半整数なら+∞側に丸めます。b=0はdomain_error。`gaussian_gcd(a,b)` はreal>0,imag≥0のassociateへ正規化し、gcd(0,0)=0。

## 使用例

[そのままコンパイルできるソース](../../examples/number_theory_basic/gaussian_integer.cpp)。C++17、`namespace cp`。例はリポジトリのルートをinclude pathに指定します。

```cpp
#include <iostream>
#include "number_theory_basic/gaussian_integer.hpp"

int main() {
    const cp::GaussianInteger a(17, 4), b(4, 1);
    const auto [q, r] = cp::gaussian_divmod(a, b);
    const auto g = cp::gaussian_gcd(a, b);
    std::cout << q.real << ' ' << q.imag << ' ' << r.real << ' ' << r.imag << '\n';
    std::cout << g.real << ' ' << g.imag << '\n';
}
```

出力:

```text
4 0 1 0
1 0
```

## 原理・計算量

複素数除算の実虚成分を厳密な有理数として丸め、Euclidを実行します。normが各段階で半分以下となるためO(log norm)回の任意精度演算。各演算のビット計算量は別。浮動小数点は使いません。

BigInt / BigRationalを使用するヘッダにはBoostのヘッダが必要です。記載した任意精度演算回数とビット計算量は区別してください。

## 使用する問題

ガウス整数の整除、二平方数和、格子の回転・ノルム。

## 検証・参考資料

`make verify-number-theory-extensions` のDebug/Release、libstdc++ debug mode、ASan/UBSan/LeakSanitizerを対象にしています。小入力の独立な全探索・素朴式、空/不正入力・予算境界、大引数を確認します。対象別の比較方法と実行結果は[検証一覧](../number_theory_extensions/README.md#検証)を参照。オンライン提出ACは未確認です。

[一次資料・共有部品・設計判断](../number_theory_extensions/references.md)。外部ソースコードの転載はせず、既存の共通部品と数学的恒等式から実装しています。
