---
title: Wavelet Matrixの区間和拡張
documentation_of: //static_query/weighted_wavelet_matrix.hpp
---

# Wavelet Matrixの区間和拡張

keyのbitごとにstable partitionを行い、並べ替えた重みの累積和も保持します。値域内の重み和と、keyが小さいk点の重み和を求めます。

## 使用例

```cpp
#include <iostream>
#include "static_query/weighted_wavelet_matrix.hpp"
int main() {
    cp::WeightedWaveletMatrix<int> s({3,1,2},{10,20,30});
    std::cout << s.range_sum(0,3,1,3) << " " << s.sum_smallest(0,3,2) << "\n";
}
```

出力: `50 50`。

## APIと使用条件

`WeightedWaveletMatrix<Key,Weight>(keys,weights)`、sum_less(l,r,upper)、range_sum(l,r,lower,upper)、sum_smallest(l,r,k)、size。

重みは負でも可。Weightは可換加法群。sum_smallestの同値keyは元の添字順に選びます。kは0..r-l。静的、keys/weightsの長さは一致させます。

## 計算量

構築O(N log(N+1))、領域O(N log(σ+1))、検索O(log(σ+1))。σは異なるkey数。 演算・比較・値のコピーが O(1) の場合です。文字列などを格納すると、その処理時間・領域が加わります。

## 使用する問題と検証

[Library Checker: Rectangle Sum](https://judge.yosupo.jp/problem/rectangle_sum)。同一座標・負の重み・未登録閾値を直接走査と比較します。

`python3 verify/run_b_1_6.py`（`--sanitize` 対応）で単独／重複include、debug/releaseの比較検証、上の使用例を確認します。検証範囲の詳細は[追加分の一覧](../b_1_6/README.md)。オンライン提出・ACは未確認です。

[操作できる図解](../../visualizations/b_1_6/index.html#weighted_wavelet_matrix) / [カテゴリ索引](README.md) / [参考資料と設計判断](../b_1_6/references.md#range) / [図解の簡略化](../../visualizations/b_1_6/README.md)
