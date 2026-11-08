---
title: Subset Convolution
documentation_of: //set_function/subset_convolution.hpp
---

# Subset Convolution

[カテゴリ索引](README.md) / [図解](../../visualizations/advanced/index.html#subset) / [実装](../../set_function/subset_convolution.hpp) / [一次資料・設計判断](../advanced/references.md)

## usage

```cpp
#include <iostream>
#include "set_function/subset_convolution.hpp"
int main() {
    auto c = cp::subset_convolution(std::vector<long long>{1, 2, 3, 4}, std::vector<long long>{5, 6, 7, 8});
    for (auto x : c) std::cout << x << ' ';
    std::cout << '\n';
}
```

[実行できる使用例](../../examples/set_function/subset_convolution.cpp)。C++17、namespace cp。ライブラリルートをincludeパスに追加してください。

## API・使用条件

`subset_convolution(vector<T> a,vector<T> b)->vector<T>`。同じ正の2冪長2^kを要求、空・非2冪・長さ不一致はinvalid_argument。h[S]=Σ(T⊆S) a[T]*b[S\T]。入力不変。係数は環（T(0),+=,−=,*）、bool不可。標準整数の途中の加減乗算を検査し、超過はoverflow_error。最終結果が小さくても途中の超過で拒否する場合があります。mod型は素数法・合成数法どちらも可。符号なし整数はメビウスで負になる途中値も拒否。

## 計算量・原理

O(k² 2^k)時間、O(k 2^k)領域。k=0（長さ1）も可。

要素数rank別にゼータ変換し、rank方向に畳み込み、メビウス変換して|S|のrankだけを取り出します。|T|+|U|=|S|の条件により、T∪U=Sの組の重複部分を除外できます。OR畳み込みとは異なります。

## 使用する問題・検証

[関連問題](https://judge.yosupo.jp/problem/subset_convolution)。関連問題は用途の例であり、このAPIだけでそのまま全て解けるという意味ではありません。特に動的連結性の関連問題は追加の成分和が必要です。

[追加分の検証入口](../../verify/run_advanced.py)で、素朴解との比較、空・重複・境界、非可換集約、履歴分岐、繰返し実行、20万要素の主要構造を検査します。実際の範囲は[検証一覧](../advanced/README.md)を参照。オンライン提出のACは未確認です。

図解は学習用の小さい入力で内部状態・候補・論理的な操作を追えます。[簡略化の範囲](../../visualizations/advanced/README.md)を参照してください。
