---
title: 任意mod畳み込み
documentation_of: //convolution/convolution_mod.hpp
---

# 任意mod畳み込み

[カテゴリ索引](README.md) / [図解](../../visualizations/advanced/index.html#crt) / [実装](../../convolution/convolution_mod.hpp) / [一次資料・設計判断](../advanced/references.md)

## usage

```cpp
#include <iostream>
#include "convolution/convolution_mod.hpp"
int main() {
    auto c = cp::convolution_mod(std::vector<int>{-1, 2}, std::vector<int>{3, 4}, 1000000007);
    for (int x : c) std::cout << x << ' ';
    std::cout << '\n';
}
```

[実行できる使用例](../../examples/convolution/convolution_mod.cpp)。C++17、namespace cp。ライブラリルートをincludeパスに追加してください。

## API・使用条件

`convolution_mod(vector<T> a,vector<U> b,int modulus)->vector<int>`。法は1..2×10^9、合成数・法1可。不正法はinvalid_argument。bool以外の64bit以下の標準整数型、異種型・負数・UINT64_MAX可。係数は先に目的の法で正規化。片方が空なら空、非空の結果はn+m−1個、末尾0も保持。結果長は2^24以下、超過length_error。既存のNTT・FFT・convolution・convolution_llは保持。

## 計算量・原理

パディング長Nで3回のNTT畳み込み、O(N log N)時間・O(N)領域。

167772161,469762049,1224736769で畳み込み、混合基数CRTの桁を目的の法へ復元します。3素数の積は約9.65×10^25。正規化後の1係数はmin(n,m)(mod−1)² < 3.36×10^25（結果長2^24以下）なのでCRTで一意に復元できます。整数全体が64bitを超えても、2桁目までの値と各積はlong long内です。NTT素数だけでなく目的の法に対する剰余を最後に計算します。

## 使用する問題・検証

[関連問題](https://judge.yosupo.jp/problem/convolution_mod_1000000007)。関連問題は用途の例であり、このAPIだけでそのまま全て解けるという意味ではありません。特に動的連結性の関連問題は追加の成分和が必要です。

[追加分の検証入口](../../verify/run_advanced.py)で、素朴解との比較、空・重複・境界、非可換集約、履歴分岐、繰返し実行、20万要素の主要構造を検査します。実際の範囲は[検証一覧](../advanced/README.md)を参照。オンライン提出のACは未確認です。

図解は学習用の小さい入力で内部状態・候補・論理的な操作を追えます。[簡略化の範囲](../../visualizations/advanced/README.md)を参照してください。
