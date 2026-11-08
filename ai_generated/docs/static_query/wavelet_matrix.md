---
title: Wavelet Matrix
documentation_of: //static_query/wavelet_matrix.hpp
---

# Wavelet Matrix

[カテゴリ索引](README.md) / [図解](../../visualizations/advanced/index.html#wavelet) / [実装](../../static_query/wavelet_matrix.hpp) / [一次資料・設計判断](../advanced/references.md)

## usage

```cpp
#include <iostream>
#include "static_query/wavelet_matrix.hpp"
int main() {
    cp::WaveletMatrix<int> w({5, 1, 4, 1, 3});
    std::cout << *w.kth_smallest(1, 5, 2) << ' ' << w.count_less(0, 5, 4) << ' ' << *w.predecessor(0, 5, 3) << '\n';
}
```

[実行できる使用例](../../examples/static_query/wavelet_matrix.cpp)。C++17、namespace cp。ライブラリルートをincludeパスに追加してください。

## API・使用条件

`WaveletMatrix<T>(vector<T>)`、`size/access(p)`、`kth_smallest(l,r,k)`（kは0始まりoptional）、`count_less(l,r,x)`、`count(l,r,x)`、`range_freq(l,r,low,high)`、`predecessor(l,r,x)`（厳密にx未満）、`successor(l,r,x)`（x以上）。該当なしはnullopt。値に `<` とコピーが必要で、負数・64bit極値・重複も可。更新は不可。境界assert、要素数はintで扱える範囲。

## 計算量・原理

座標圧縮を含む構築O(n log(n+1))。各検索O(log(σ+1))、閾値検索は圧縮辞書の二分探索込み。同値1種類ならO(1)。格納O(n+σ+(n/64+1)log(σ+1))ワード。

座標圧縮した整数の上位bitから安定分割し、各レベルのbit列と0側の長さを保持します。64bitブロックのpopcount接頭辞でrankをO(1)にし、区間を0/1側へ写します。全レベルの整数prefix配列を保存する実装よりbit列部分を節約します。

## 使用する問題・検証

[関連問題](https://judge.yosupo.jp/problem/range_kth_smallest)。関連問題は用途の例であり、このAPIだけでそのまま全て解けるという意味ではありません。特に動的連結性の関連問題は追加の成分和が必要です。

[追加分の検証入口](../../verify/run_advanced.py)で、素朴解との比較、空・重複・境界、非可換集約、履歴分岐、繰返し実行、20万要素の主要構造を検査します。実際の範囲は[検証一覧](../advanced/README.md)を参照。オンライン提出のACは未確認です。

図解は学習用の小さい入力で内部状態・候補・論理的な操作を追えます。[簡略化の範囲](../../visualizations/advanced/README.md)を参照してください。
