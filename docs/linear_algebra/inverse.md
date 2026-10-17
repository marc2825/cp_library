---
title: 逆行列
documentation_of: //linear_algebra/inverse.hpp
---

# 逆行列

行基本変形で逆行列を求め、特異な場合はnulloptを返します。

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/transform_a/index.html#inverse) / [補完一覧・検証](../transform_a/README.md) / [一次資料](../transform_a/references.md)

## API・条件

`matrix_inverse<T>(const Matrix<T>& A) -> optional<Matrix<T>>`は体型T用。通常の整数除算はコンパイル時に拒否します。実数版は第2引数epsilon（既定1e-10、正・有限）を持ち、絶対値最大pivotと絶対epsilonを使います。

`matrix_inverse_mod<Mod=998244353>(const Matrix<int>& A)`は素数Mod=2〜2×10^9。int係数を正規化しintで返します。合成数法はコンパイル時拒否。全版正方行列が必要で違反はinvalid_argument。0×0は逆を持ちます。入力不変。実数の非有限入力はinvalid_argument、非有限の途中値はoverflow_error。独自型の演算条件は利用者が管理します。実数の判定はスケール・悪条件に影響され、数学的な階数や精度を保証しません。

## Usage

```cpp
#include <iostream>
#include "linear_algebra/inverse.hpp"
int main() {
    auto b = cp::matrix_inverse_mod(cp::Matrix<int>{{1, 2}, {3, 4}});
    if (b) for (std::size_t i = 0; i < b->rows(); ++i) {
        for (std::size_t j = 0; j < b->cols(); ++j) std::cout << (*b)(i, j) << ' ';
        std::cout << '\n';
    }
}
```

出力:

```text
998244351 1
499122178 499122176
```

[コンパイルできる使用例](../../examples/linear_algebra/inverse.cpp)。

## 原理

[A|I]に同じ行操作を適用します。左側を単位行列にできたとき、右側が逆行列です。pivotがないと特異です。既存Gauss–Jordanと同じ体・実数pivotの規約を使い、複数右辺をまとめて消去します。

## 計算量

n次でO(n³)時間・O(n²)領域。 型の加減乗除をO(1)として数えます。独自型ではその演算費用を掛けます。

## 使用する問題と検証

[Library Checker Inverse Matrix](https://judge.yosupo.jp/problem/inverse_matrix)。

`python3 verify/run_transform_a.py`（`--sanitize`対応）で単独include・複数翻訳単位・このusageと出力・独立した定義との比較・境界と例外を確認します。各項目の具体的な検証範囲は[補完一覧](../transform_a/README.md)へ。オンラインACは未確認です。
