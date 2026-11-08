---
title: 商prefix表のDirichlet積
documentation_of: //number_theory_convolution/dirichlet_prefix_product.hpp
---

# 商prefix表のDirichlet積（N-09）

[カテゴリ索引](README.md) / [N全項目と検証](../number_theory_extensions/README.md) / [操作できる図解](../../visualizations/number_theory_extensions/index.html#dirichlet_prefix_product)

## API・条件

`QuotientPrefix<T>(N,prefix,max_root=5000000)` は各x=⌊N/j⌋でF(x)=Σ_{i≤x}f(i)を保存します。`sum(0)=0`、`sum(x)` は登録された商以外ならinvalid_argument。`total(),limit(),root(),quotients(),set(x,value)` を提供し、商は降順です。N=0は空表。`dirichlet_prefix_product(f,g)` は同じNの2表からΣ_{ij≤x}f(i)g(j)を返します。Tは可換環で0,1、加減乗算を持つ型。組込み整数の演算overflowは例外です。

## 使用例

[そのままコンパイルできるソース](../../examples/number_theory_convolution/dirichlet_prefix_product.cpp)。C++17、`namespace cp`。例はリポジトリのルートをinclude pathに指定します。

```cpp
#include "basic_utility/big_integer.hpp"
#include <iostream>
#include "number_theory_convolution/dirichlet_prefix_product.hpp"

int main() {
    cp::QuotientPrefix<cp::BigInt> ones(20, [](auto x) { return cp::BigInt(x); });
    std::cout << cp::dirichlet_prefix_product(ones, ones).total() << '\n';
}
```

出力:

```text
66
```

## 原理・計算量

s=⌊√x⌋に対しΣ_{i≤s}f(i)G(x/i)+Σ_{i≤s}g(i)F(x/i)−F(s)G(s)を各商で計算します。すべての参照先が同じ商集合に入る閉包性を利用。O(N^(3/4))環演算、領域O(√N)。表構築はO(√N)回のprefix評価。√Nの予算超過はlength_error。

BigInt / BigRationalを使用するヘッダにはBoostのヘッダが必要です。記載した任意精度演算回数とビット計算量は区別してください。

## 使用する問題

約数総和、巨大NのDirichlet畳み込み、商列挙だけで保持するsummatory関数。

## 検証・参考資料

`make verify-number-theory-extensions` のDebug/Release、libstdc++ debug mode、ASan/UBSan/LeakSanitizerを対象にしています。小入力の独立な全探索・素朴式、空/不正入力・予算境界、大引数を確認します。対象別の比較方法と実行結果は[検証一覧](../number_theory_extensions/README.md#検証)を参照。オンライン提出ACは未確認です。

[一次資料・共有部品・設計判断](../number_theory_extensions/references.md)。外部ソースコードの転載はせず、既存の共通部品と数学的恒等式から実装しています。
