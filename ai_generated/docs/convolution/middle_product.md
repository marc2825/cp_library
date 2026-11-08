---
title: Middle Product
documentation_of: //convolution/middle_product.hpp
---

# Middle Product

連続する窓と短い列の内積をまとめて求めます。

[カテゴリ索引](README.md) / [今回の一覧](../b_16_20/README.md) / [操作できる図解](../../visualizations/b_16_20/index.html#middle_product) / [一次資料と設計判断](../b_16_20/references.md)

## API・使用条件

`middle_product<Mod,Root>(a,b)`→h、h[k]=Σ_i a[k+i]b[i]。0<|b|≤|a|。

bを反転した全長畳み込みから中央の|a|-|b|+1個を抽出します。変換長の短縮を行う専用中間積ではありません。全長の積がNTT容量内で必要。条件違反invalid_argument。

## usage

```cpp
#include <iostream>
#include <vector>
#include "../../convolution/middle_product.hpp"

int main() {
    auto c = cp::middle_product(std::vector<int>{1, 2, 3, 4}, std::vector<int>{2, 1});
    std::cout << c[0] << " " << c[2] << "\n";
}
```

出力:

```text
4 10
```

[コンパイルして動く例](../../examples/convolution/middle_product_b.cpp)。

## 計算量

切上げた|a|+|b|-1をLとしてO(L log L)時間・O(L)領域。

## 使用する問題・検証範囲

重み付きスライディング内積、相関、畳み込みDPの必要区間の抽出。

全窓の直接内積・長さ違反を検証。 `python3 verify/run_b_16_20.py` / `--sanitize`。使用例とこのusageの一致、単独・重複include、debug/releaseを確認します。オンライン提出・AC、全ジャッジ最大制約での時間内完了は未確認です。
