---
title: 高速な部分集合畳み込み
documentation_of: //set_function/fast_subset_convolution.hpp
---

# 高速な部分集合畳み込み

既存のranked-zetaによる高速な部分集合畳み込みを共有し、intの素数mod配列向けの入口を加えます。

[カテゴリ索引](README.md) / [今回の一覧](../b_16_20/README.md) / [操作できる図解](../../visualizations/b_16_20/index.html#fast_subset_convolution) / [一次資料と設計判断](../b_16_20/references.md)

## API・使用条件

既存`subset_convolution<T>(a,b)`、追加`subset_convolution_mod<Mod>(vector<int>,vector<int>)`。長さは同じ正の2冪。

既存のgeneric実装を内部の素数体型で呼び出します。標準intの積・和として処理しないため、mod型を別途用意せず使えます。入力は正規化。SPSの演算でも同じ実体を使用。

## usage

```cpp
#include <iostream>
#include <vector>
#include "../../set_function/fast_subset_convolution.hpp"

int main() {
    auto c = cp::subset_convolution_mod(std::vector<int>{1, 2, 3, 4}, std::vector<int>{1, 1, 1, 1});
    std::cout << c[3] << "\n";
}
```

出力:

```text
10
```

[コンパイルして動く例](../../examples/set_function/fast_subset_convolution_b.cpp)。

## 計算量

N=2^nとしてO(n²N)時間、O(nN)領域。素朴な全submaskのO(3^n)より速いranked-zeta方式。

## 使用する問題・検証範囲

[Subset Convolution](https://judge.yosupo.jp/problem/subset_convolution)。

全submaskの直接積と各素数法で比較。既存変換の検証も実行。 `python3 verify/run_b_16_20.py` / `--sanitize`。使用例とこのusageの一致、単独・重複include、debug/releaseを確認します。オンライン提出・AC、全ジャッジ最大制約での時間内完了は未確認です。
