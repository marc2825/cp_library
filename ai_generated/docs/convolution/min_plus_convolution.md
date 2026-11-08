---
title: Min-Plus Convolution（共通入口）
documentation_of: //convolution/min_plus_convolution.hpp
---

# Min-Plus Convolution（共通入口）

3.12の凸・凹形状を使うMin-Plus実装を共有します。

[カテゴリ索引](README.md) / [今回の一覧](../b_16_20/README.md) / [操作できる図解](../../visualizations/b_16_20/index.html#min_plus_convolution) / [一次資料と設計判断](../b_16_20/references.md)

## API・使用条件

`min_plus_convolution_convex_convex(a,b)`、`_arbitrary_convex`、`_convex_arbitrary`、`_concave_concave`、`_arbitrary_concave`、`_concave_arbitrary`。

凸は隣接差が非減少、凹は非増加。形状の検査は行いません。一般の無条件な高速Min-Plusではありません。[全APIと形状条件](../dp_optimization/min_plus_convolution_b.md)を共有します。空を含むと空、整数の加算超過は例外。

## usage

```cpp
#include <iostream>
#include <vector>
#include "../../convolution/min_plus_convolution.hpp"

int main() {
    auto c = cp::min_plus_convolution_convex_convex(std::vector<int>{0, 1, 4}, std::vector<int>{0, 2, 7});
    std::cout << c[2] << "\n";
}
```

出力:

```text
3
```

[コンパイルして動く例](../../examples/convolution/min_plus_convolution_b.cpp)。

## 計算量

凸×凸と凹×凹はO(N+M)、任意×凸はO((N+M)log(N+M))、任意×凹はO(N log(M+1)+M)。領域O(N+M)。

## 使用する問題・検証範囲

[Min Plus Convolution (Convex and Arbitrary)](https://judge.yosupo.jp/problem/min_plus_convolution_convex_arbitrary)。

既存3.12の全添字対比較と今回の共通入口を検証。 `python3 verify/run_b_16_20.py` / `--sanitize`。使用例とこのusageの一致、単独・重複include、debug/releaseを確認します。オンライン提出・AC、全ジャッジ最大制約での時間内完了は未確認です。
