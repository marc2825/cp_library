---
title: 二進浮動小数点と多倍長有理数
documentation_of: //basic_utility/binary_rational.hpp
---

# 二進浮動小数点と多倍長有理数

[実装](../../basic_utility/binary_rational.hpp) / [BigRational](big_rational.md) / [数値解析](../numerical_analysis/README.md)

`cp::rational_from_binary(long double)` は有限なradix-2の表現値をBigRationalへ厳密変換します。subnormalも扱い、NaN/∞はinvalid_argumentです。10進文字列の理想値を復元するAPIではありません。仮数のbit数だけ走査し、分子／分母のbit数は指数範囲に依存します。

`cp::rational_to_long_double(const BigRational&)` は分子／分母それぞれの上位61bitから近似します。正しく丸められた変換という保証はなく、underflowは0、overflowはoverflow_errorです。多倍長値のコピーとshiftに必要な領域・時間を使います。

幾何の既存 `geometry::robust::dyadic` と `rational::approximate` の変換実体を共有します。既存の幾何APIと挙動は維持し、数値解析の二次判別式とSturm分離にも使います。Boostヘッダが必要です。単独／重複include、指数±10000、subnormal、符号、既存H回帰を数値解析runnerと幾何runnerで検証します。

```cpp
#include "basic_utility/binary_rational.hpp"
// 0.5L の実際の二進値は厳密に1/2。
auto half = cp::rational_from_binary(0.5L);
auto approximate = cp::rational_to_long_double(half);
```
