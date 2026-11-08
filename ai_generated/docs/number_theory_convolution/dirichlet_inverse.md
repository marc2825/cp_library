---
title: Dirichlet逆元
documentation_of: //number_theory_convolution/dirichlet_inverse.hpp
---

# Dirichlet逆元（N-10）

[カテゴリ索引](README.md) / [N全項目と検証](../number_theory_extensions/README.md) / [操作できる図解](../../visualizations/number_theory_extensions/index.html#dirichlet_inverse)

## API・条件

`dirichlet_inverse<T>(f,inverse)` はfと同じ長さの配列をoptionalで返し、添字0は無視して0にします。`dirichlet_prefix_inverse(f,inverse)` は商prefix表版。callbackは `inverse(f(1))->optional<T>`、非unitならnullopt。空/添字1のない配列、N=0の表はそのまま空の有効範囲を返しcallbackを呼びません。可換環での逆元条件です。通常整数環のunitは±1で、単なる整数除算1/f(1)ではありません。

## 使用例

[そのままコンパイルできるソース](../../examples/number_theory_convolution/dirichlet_inverse.cpp)。C++17、`namespace cp`。例はリポジトリのルートをinclude pathに指定します。

```cpp
#include "basic_utility/big_integer.hpp"
#include <iostream>
#include "number_theory_convolution/dirichlet_inverse.hpp"

int main() {
    const auto unit = [](cp::BigInt x) -> std::optional<cp::BigInt> {
        return x == 1 ? std::optional<cp::BigInt>(1) : std::nullopt;
    };
    const auto inverse = cp::dirichlet_inverse(std::vector<cp::BigInt>(11, 1), unit);
    for (int i = 1; i <= 10; ++i) std::cout << (*inverse)[i] << ' ';
    std::cout << '\n';
    cp::QuotientPrefix<cp::BigInt> ones(20, [](auto x) { return cp::BigInt(x); });
    std::cout << cp::dirichlet_prefix_inverse(ones, unit)->total() << '\n';
}
```

出力:

```text
1 -1 -1 0 -1 1 -1 0 0 1
-3
```

## 原理・計算量

f(1)g(n)=δ(n)−Σ_{d|n,d>1}f(d)g(n/d)を小さいnから計算。配列版O(n log n)時間/O(n)領域。表版は区間ごとに同じ商をまとめ、昇順xでG(x)を復元します。O(N^(3/4))環演算/O(√N)領域。

BigInt / BigRationalを使用するヘッダにはBoostのヘッダが必要です。記載した任意精度演算回数とビット計算量は区別してください。

## 使用する問題

定数1の逆元としてMöbius関数/その総和、Dirichlet方程式の解法。

## 検証・参考資料

`make verify-number-theory-extensions` のDebug/Release、libstdc++ debug mode、ASan/UBSan/LeakSanitizerを対象にしています。小入力の独立な全探索・素朴式、空/不正入力・予算境界、大引数を確認します。対象別の比較方法と実行結果は[検証一覧](../number_theory_extensions/README.md#検証)を参照。オンライン提出ACは未確認です。

[一次資料・共有部品・設計判断](../number_theory_extensions/references.md)。外部ソースコードの転載はせず、既存の共通部品と数学的恒等式から実装しています。
