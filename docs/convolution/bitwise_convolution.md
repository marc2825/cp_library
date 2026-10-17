---
title: AND / OR / XOR Convolution
documentation_of: //convolution/bitwise_convolution.hpp
---

# AND / OR / XOR Convolution

添字のAND・OR・XORで係数を集計します。3.16と3.19は同じ公開関数を共有します。

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/transform_a/index.html#bitwise) / [補完一覧・検証](../transform_a/README.md) / [一次資料](../transform_a/references.md)

## API・条件

`bitwise_and_convolution<T>(a,b)` / `bitwise_or_convolution<T>(a,b)` / `bitwise_xor_convolution<T>(a,b)` はvector<T>を受け取りvector<T>を返します。両入力は同じ正の2冪長N。空・非2冪・長さ不一致はinvalid_argument。自動paddingは行わず、入力を変更しません。c[k]=Σ_(i op j=k)a[i]*b[j]です。

AND/ORはT(0)、+=、-=、積を持つ環で可。合成数法・除算なしの型も使え、積の順はa[i]*b[j]を保持します。XORはさらにFWHTの2で割れる条件が必要です。標準整数の途中の加減乗算超過はoverflow_error。XORの逆で割り切れなければdomain_error。符号なしは変換途中の負数でも拒否します。boolは未対応。

## Usage

```cpp
#include <iostream>
#include "convolution/bitwise_convolution.hpp"
int main() {
    const std::vector<long long> a{1, 2}, b{3, 4};
    for (auto x : cp::bitwise_and_convolution(a, b)) std::cout << x << ' ';
    std::cout << '\n';
    for (auto x : cp::bitwise_or_convolution(a, b)) std::cout << x << ' ';
    std::cout << '\n';
    for (auto x : cp::bitwise_xor_convolution(a, b)) std::cout << x << ' ';
    std::cout << '\n';
}
```

出力:

```text
13 8
3 18
11 10
```

[コンパイルできる使用例](../../examples/convolution/bitwise_convolution.cpp)。

## 原理

ANDは上位集合zeta、ORは部分集合zeta、XORはFWHTで両入力を変換し、各点積から逆変換します。ORは重なる集合も含むため、互いに素な集合だけを集計するSubset Convolutionとは式が異なります。

## 計算量

各O(N log N)時間・O(N)追加領域。 型の加減乗除をO(1)として数えます。独自型ではその演算費用を掛けます。

## 使用する問題と検証

[Bitwise AND](https://judge.yosupo.jp/problem/bitwise_and_convolution) / [Bitwise XOR](https://judge.yosupo.jp/problem/bitwise_xor_convolution)。ORも補集合でAND問題へ接続します。

`python3 verify/run_transform_a.py`（`--sanitize`対応）で単独include・複数翻訳単位・このusageと出力・独立した定義との比較・境界と例外を確認します。各項目の具体的な検証範囲は[補完一覧](../transform_a/README.md)へ。オンラインACは未確認です。
