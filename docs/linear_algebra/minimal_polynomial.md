---
title: 行列の最小多項式
documentation_of: //linear_algebra/minimal_polynomial.hpp
---

# 行列の最小多項式

行列全体を消去する最小次数のmonic多項式を決定的に求めます。

[カテゴリ索引](README.md) / [今回の一覧](../b_16_20/README.md) / [操作できる図解](../../visualizations/b_16_20/index.html#minimal_polynomial) / [一次資料と設計判断](../b_16_20/references.md)

## API・使用条件

`matrix_minimal_polynomial_mod<Mod>(Matrix<int>)`→昇冪係数。正方行列・素数法、原始根やNTT容量に依存しません。

標準基底e_jそれぞれについてKrylov列を係数履歴付きで消去し、得た最小多項式のlcmを取ります。次数nに到達すれば終了。0×0は{1}。高速Frobenius標準形や確率的推定とは別の、検証しやすい決定的実装です。

## usage

```cpp
#include <iostream>
#include <vector>
#include "../../linear_algebra/minimal_polynomial.hpp"

int main() {
    auto p = cp::matrix_minimal_polynomial_mod(cp::Matrix<int>{{0, 1}, {0, 0}});
    std::cout << p[0] << " " << p[1] << " " << p[2] << "\n";
}
```

出力:

```text
0 0 1
```

[コンパイルして動く例](../../examples/linear_algebra/minimal_polynomial_b.cpp)。

## 計算量

n次正方行列で最悪O(n⁴+n³log Mod)時間、O(n²)作業領域。低次数が早く揃えば短縮されます。

## 使用する問題・検証範囲

線形変換の最小次数の漸化式、疎求解の小規模オラクル。

m(A)=0、法3の低次数monic多項式の全探索による最小性、零・反復固有値・冪零を比較。 `python3 verify/run_b_16_20.py` / `--sanitize`。使用例とこのusageの一致、単独・重複include、debug/releaseを確認します。オンライン提出・AC、全ジャッジ最大制約での時間内完了は未確認です。
