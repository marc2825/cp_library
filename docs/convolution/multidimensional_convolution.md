---
title: 多次元畳み込み
documentation_of: //convolution/multidimensional_convolution.hpp
---

# 多次元畳み込み

各軸に沿って添字を加える通常の多次元積を、Kronecker置換で1回のNTT畳み込みへ変換します。

[カテゴリ索引](README.md) / [今回の一覧](../b_16_20/README.md) / [操作できる図解](../../visualizations/b_16_20/index.html#multidimensional_convolution) / [一次資料と設計判断](../b_16_20/references.md)

## API・使用条件

`multidimensional_convolution<Mod,Root>(a,a_shape,b,b_shape)`→`{shape,coefficients}`。row-major、最後の軸が最速。両shapeの次元数は同じ、全軸は正。0次元は配列長1のスカラー。

配列長とshapeの積が一致しなければinvalid_argument。出力軸はa_shape[i]+b_shape[i]-1。出力体積と切上げ2冪がNTT容量内であることが必要。サイズ超過length_error。巡回畳み込みではありません。

## usage

```cpp
#include <iostream>
#include <vector>
#include "../../convolution/multidimensional_convolution.hpp"

int main() {
    auto c = cp::multidimensional_convolution(std::vector<int>{1, 2, 3, 4}, {2, 2}, std::vector<int>{1, 1}, {1, 2});
    std::cout << c.shape[0] << " " << c.shape[1] << " " << c.coefficients[1] << "\n";
}
```

出力:

```text
2 3 3
```

[コンパイルして動く例](../../examples/convolution/multidimensional_convolution_b.cpp)。

## 計算量

出力体積V、軸数d、入力体積A,BとしてO(d(A+B)+V log V)時間、O(V+d)領域。

## 使用する問題・検証範囲

2Dのパターン・分布の通常積、複数変数の多項式積。

ランダム3次元tensorの全添字対、スカラー、形違反を独立比較。 `python3 verify/run_b_16_20.py` / `--sanitize`。使用例とこのusageの一致、単独・重複include、debug/releaseを確認します。オンライン提出・AC、全ジャッジ最大制約での時間内完了は未確認です。
