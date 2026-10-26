---
title: 重み付き多項式mod和
documentation_of: //number_theory_basic/mod_sum_polynomial.hpp
---

# 重み付き多項式mod和（N-28）

[カテゴリ索引](README.md) / [N全項目と検証](../number_theory_extensions/README.md) / [操作できる図解](../../visualizations/number_theory_extensions/index.html#mod_sum_polynomial)

## API・条件

今回の具体化: `polynomial_mod_sum(left,right,a,b,m,p,q,max_degree=12)` はΣ_{left≤i<right} i^p·((a*i+b) mod m)^qの厳密BigInt。left,a,bは負でも可、m>0、p,qは非負、p+q≤max_degreeかつ≤64。空区間は0。`polynomial_mod_sum_mod(...,output_modulus,max_degree)` はさらに任意の正のuint64法で剰余を返します。0^0=1。入力側の法と出力側の法は独立です。

## 使用例

[そのままコンパイルできるソース](../../examples/number_theory_basic/mod_sum_polynomial.cpp)。C++17、`namespace cp`。例はリポジトリのルートをinclude pathに指定します。

```cpp
#include <iostream>
#include "number_theory_basic/mod_sum_polynomial.hpp"

int main() {
    std::cout << cp::polynomial_mod_sum(0, 10, 7, 3, 11, 1, 2) << '\n';
}
```

出力:

```text
1215
```

## 原理・計算量

剰余=ai+b−m floor((ai+b)/m)を二項展開し、N-04のfloor momentsを共有します。D=p+q、L=2+区間長+m+|a|+|a*left+b|としてO((D+1)⁴ log² L)任意精度演算、O((D+1)² log L)領域。区間長に比例した走査は行いません。

BigInt / BigRationalを使用するヘッダにはBoostのヘッダが必要です。記載した任意精度演算回数とビット計算量は区別してください。

## 使用する問題

周期的剰余の平均・二乗和、添字による重み付け、合成数法への出力。

## 検証・参考資料

`make verify-number-theory-extensions` のDebug/Release、libstdc++ debug mode、ASan/UBSan/LeakSanitizerを対象にしています。小入力の独立な全探索・素朴式、空/不正入力・予算境界、大引数を確認します。対象別の比較方法と実行結果は[検証一覧](../number_theory_extensions/README.md#検証)を参照。オンライン提出ACは未確認です。

[一次資料・共有部品・設計判断](../number_theory_extensions/references.md)。外部ソースコードの転載はせず、既存の共通部品と数学的恒等式から実装しています。
