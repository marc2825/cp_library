---
title: pivot付き三重対角ソルバ
documentation_of: //numerical_analysis/tridiagonal.hpp
---

# Y-05 pivot付き三重対角ソルバ

[カテゴリ](README.md) / [実装](../../numerical_analysis/tridiagonal.hpp) / [操作できる図解](../../visualizations/numerical_analysis/index.html#tridiagonal) / [参考資料](references.md)

最大絶対値のpivotを隣接2行から選び、交換で生じる第2上対角も保存して後退代入する。

## API・使用条件

`TridiagonalFactorization(lower,diagonal,upper,pivot_tolerance)`、solve(rhs)、status/row_swaps/singular_index。`solve_tridiagonal`も公開。

lower/upperはn−1、diagonal/rhsはn。空系可、入力は保持。隣接行の部分pivotを使う。singularはpivotがabsolute+relative·入力最大絶対値以下との数値判定。既定は両方0。数学的な特異性・良条件性の証明や不定/矛盾の区別はしない。成功時も残差と入力の条件を確認する。

lower[i]=A[i+1][i]、diagonal[i]=A[i][i]、upper[i]=A[i][i+1]です。`TridiagonalFactorization` は係数を値で受け取り、因子を保持して同じ行列の複数右辺に再利用できます。`solve(rhs)` は元の因子・右辺を変更しません。`solve_tridiagonal(lower,diagonal,upper,rhs,pivot_tolerance)` は一度だけ解く簡易入口です。

隣接2行の大きい絶対値をpivotへ選びます。行交換で生じる第2上対角も保存するので、先頭の対角が0でも直ちに失敗しません。`row_swaps()` は交換回数（O(n)で数える）、`singular_index()` は失敗pivot位置（成功時n）、`size()` はnです。後退代入は上対角2本を使います。

`pivot_tolerance={0,0}` が既定。入力最大絶対値scaleを使って |pivot|≤absolute+relative*scale なら `singular` です。浮動小数点消去の丸めで数学的に特異な行列が小さい非零pivotを生むこともあります。必要な閾値を指定し、残差 A x−b と問題の条件数を確認してください。閾値を上げると非特異な悪条件系も拒否します。`converged` は解の前進誤差を保証しません。不定系・矛盾系の分類は[Gauss–Jordan](../linear_algebra/README.md)などを使います。

## 計算量

factor O(n)時間・領域、右辺1本のsolve O(n)時間・出力領域。係数は再利用可能。

## 使用例

```cpp
#include "numerical_analysis/tridiagonal.hpp"
#include <iomanip>
#include <iostream>
using namespace cp::numerical;
int main() {
    std::cout << std::setprecision(6);
    TridiagonalFactorization factor({1, 1}, {0, 2, 2}, {1, 1});
    const auto result = factor.solve({2, 8, 8});
    for (std::size_t i = 0; i < result.solution.size(); ++i)
        std::cout << (i ? " " : "") << result.solution[i];
    std::cout << "\n";
}
```

出力:

```text
1 2 3
```

## 使用する問題・検証

一次元差分法、三次スプライン、帯幅1の連立方程式。 特定ジャッジでのオンラインACは未確認です。

[検証runner](../../verify/run_numerical_analysis.py)と[境界・比較テスト](../../verify/numerical_analysis/unit_test.cpp)で、解析解・Python Decimal/Fraction・既存の厳密Gauss–Jordanとの比較を行います。図解のサンプルはこのC++使用例の出力と照合します。共通の例外・丸め条件と検証コマンドは[カテゴリREADME](README.md)を参照してください。
