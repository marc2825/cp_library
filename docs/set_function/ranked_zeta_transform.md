---
title: Ranked Zeta Transform
documentation_of: //set_function/ranked_zeta_transform.hpp
---

# Ranked Zeta Transform

集合の要素数ごとに部分集合の和を保持する、再利用可能な公開APIです。

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/transform_a/index.html#ranked) / [補完一覧・検証](../transform_a/README.md) / [一次資料](../transform_a/references.md)

## API・条件

`ranked_zeta_transform<T>(const vector<T>& f) -> vector<vector<T>>`。入力長N=2^nは正、結果は[n+1][N]で、F[k][S]=Σ_(T⊆S,|T|=k)f[T]。入力は変更しません。

`ranked_mobius_transform<T>(vector<vector<T>>& F)`は各rank層に部分集合メビウス変換をin-placeで適用します。元のf[S]は結果[popcount(S)][S]。rank方向の積等を行った一般の表にも使え、逆変換後に他rankが0であることは要求しません。rank数n+1・正の2冪N・全行の長さを変更前に検査し、違反はinvalid_argument。T(0)・+=、逆では-=を使い、標準整数の途中超過はoverflow_error。途中の算術例外では更新済みの行が残ります。

## Usage

```cpp
#include <iostream>
#include "set_function/ranked_zeta_transform.hpp"
int main() {
    auto f = cp::ranked_zeta_transform(std::vector<long long>{1, 2, 3, 4});
    std::cout << f[1][3] << '\n';
    cp::ranked_mobius_transform(f);
    std::cout << f[2][3] << '\n';
}
```

出力:

```text
5
4
```

[コンパイルできる使用例](../../examples/set_function/ranked_zeta_transform.cpp)。

## 原理

最初にf[S]をrank=|S|の層だけに置き、全rank層で部分集合zetaを計算します。既存Subset Convolutionもこの公開APIを共有し、rankの積とメビウスの後にrank=|S|を取り出します。

## 計算量

O(n²2^n)時間、O(n2^n)領域。逆変換の追加領域O(1)。 型の加減乗除をO(1)として数えます。独自型ではその演算費用を掛けます。

## 使用する問題と検証

[Library Checker Subset Convolution](https://judge.yosupo.jp/problem/subset_convolution)。

`python3 verify/run_transform_a.py`（`--sanitize`対応）で単独include・複数翻訳単位・このusageと出力・独立した定義との比較・境界と例外を確認します。各項目の具体的な検証範囲は[補完一覧](../transform_a/README.md)へ。オンラインACは未確認です。
