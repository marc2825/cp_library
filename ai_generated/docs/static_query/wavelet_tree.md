---
title: Wavelet Tree
documentation_of: //static_query/wavelet_tree.hpp
---

# Wavelet Tree

圧縮した値域を二分する木です。各節点に左の子へ進む要素数のprefixを持ち、位置区間を子の区間へ写します。

## 使用例

```cpp
#include <iostream>
#include "static_query/wavelet_tree.hpp"
int main() {
    cp::WaveletTree<int> s({4,1,4,2});
    std::cout << *s.kth_smallest(0,4,2) << "\n";
}
```

出力: `4`。

## APIと使用条件

`WaveletTree<T>(vector)`、count_less(l,r,x)、range_freq(l,r,lower,upper)、kth_smallest(l,r,k)はoptional<T>、size。

静的配列、比較は<のみ。rankは0始まり、区間が空／rankが範囲外ならnullopt。Wavelet Matrixとは節点ごとの列を保持する別実装です。

## 計算量

構築O(N log(N+1))、領域O(N log(σ+1)+σ)、検索O(log(σ+1))。 演算・比較・値のコピーが O(1) の場合です。文字列などを格納すると、その処理時間・領域が加わります。

## 使用する問題と検証

静的／動的な区間集計、版の分岐、矩形更新などで使います。独立した配列・多重集合・全点走査と比較します。

`python3 verify/run_b_1_6.py`（`--sanitize` 対応）で単独／重複include、debug/releaseの比較検証、上の使用例を確認します。検証範囲の詳細は[追加分の一覧](../b_1_6/README.md)。オンライン提出・ACは未確認です。

[操作できる図解](../../visualizations/b_1_6/index.html#wavelet_tree) / [カテゴリ索引](README.md) / [参考資料と設計判断](../b_1_6/references.md#range) / [図解の簡略化](../../visualizations/b_1_6/README.md)
