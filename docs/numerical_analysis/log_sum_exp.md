---
title: Log-Sum-Exp
documentation_of: //numerical_analysis/stable_arithmetic.hpp
---

# Y-03 Log-Sum-Exp

[カテゴリ](README.md) / [実装](../../numerical_analysis/stable_arithmetic.hpp) / [操作できる図解](../../visualizations/numerical_analysis/index.html#log_sum_exp) / [参考資料](references.md)

最大値を引いてexpのoverflowを防ぎ、最大項以外の和にlog1pを使う。

## API・使用条件

`log_sum_exp(vector<long double>)`。

空・全て−∞は−∞、+∞を含めば+∞、NaNは拒否。指数のunderflowは寄与0として扱う。最終値が表現範囲外ならoverflow_error。

`log_sum_exp(values)` は log(Σexp(values[i])) を返します。m=max(values)を引き、m+log(Σexp(x−m))へ変形して指数のoverflowを避けます。最大項を1個取り除いた補償和に `log1p` を適用するため、残りの寄与が小さい場合にも対応します。

空列・全要素−∞は−∞、+∞を含む列は+∞。NaNは+∞の共存にかかわらず拒否します。小さいexpのunderflowは0への丸めとして許容します。正規化確率や対数空間DP、確率の混合の集計に使います。

## 計算量

n項でO(n)時間、O(1)追加領域。

## 使用例

```cpp
#include "numerical_analysis/stable_arithmetic.hpp"
#include <iomanip>
#include <iostream>
using namespace cp::numerical;
int main() {
    std::cout << std::setprecision(6);
    std::cout << log_sum_exp({1000, 1000}) << "\n";
}
```

出力:

```text
1000.69
```

## 使用する問題・検証

対数尤度、対数空間DP、確率の混合。 特定ジャッジでのオンラインACは未確認です。

[検証runner](../../verify/run_numerical_analysis.py)と[境界・比較テスト](../../verify/numerical_analysis/unit_test.cpp)で、解析解・Python Decimal/Fraction・既存の厳密Gauss–Jordanとの比較を行います。図解のサンプルはこのC++使用例の出力と照合します。共通の例外・丸め条件と検証コマンドは[カテゴリREADME](README.md)を参照してください。
