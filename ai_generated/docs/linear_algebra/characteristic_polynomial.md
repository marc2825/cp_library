---
title: 特性多項式
documentation_of: //linear_algebra/characteristic_polynomial.hpp
---

# 特性多項式

正方行列Aに対しdet(xI−A)を求めます。標数が小さい素数体にも使えます。

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/transform_a/index.html#charpoly) / [補完一覧・検証](../transform_a/README.md) / [一次資料](../transform_a/references.md)

## API・条件

`characteristic_polynomial<T>(Matrix<T> A) -> vector<T>`は除算できる体型用で、通常の整数除算はコンパイル時に拒否します。`characteristic_polynomial_mod<Mod=998244353>(const Matrix<int>& A)`は素数Mod=2〜2×10^9のint版。合成数法は拒否します。

入力は正方行列で、違反はinvalid_argument。返り値は定数項から昇冪順のn+1係数、最高次は1、0×0では{1}。入力不変です。実数では最大絶対値pivotを使いますがepsilonによる0への切捨てはせず、数値的安定性・相対精度を保証しません。非有限入力はinvalid_argument、非有限の途中値はoverflow_error。独自型の演算条件は利用者が管理します。

## Usage

```cpp
#include <iostream>
#include "linear_algebra/characteristic_polynomial.hpp"
int main() {
    for (auto x : cp::characteristic_polynomial_mod(cp::Matrix<int>{{1, 2}, {3, 4}})) std::cout << x << ' ';
    std::cout << '\n';
}
```

出力:

```text
998244351 998244348 1
```

[コンパイルできる使用例](../../examples/linear_algebra/characteristic_polynomial.cpp)。

## 原理

行の消去と逆向きの列操作を組み合わせ、相似変形で上Hessenberg行列H（対角の一つ下より下が0）を作ります。行と列の交換も同時に行うため多項式は不変です。主小行列の特性多項式P_mを、(x−H[m−1,m−1])P_(m−1)から上右成分と下副対角の積の寄与を引いて求めます。1,2,...,nで割る方法を使わないため、Mod<=nでも成立します。

## 計算量

O(n³)時間・O(n²)領域。 型の加減乗除をO(1)として数えます。独自型ではその演算費用を掛けます。

## 使用する問題と検証

[Library Checker Characteristic Polynomial](https://judge.yosupo.jp/problem/characteristic_polynomial)。

`python3 verify/run_transform_a.py`（`--sanitize`対応）で単独include・複数翻訳単位・このusageと出力・独立した定義との比較・境界と例外を確認します。各項目の具体的な検証範囲は[補完一覧](../transform_a/README.md)へ。オンラインACは未確認です。
