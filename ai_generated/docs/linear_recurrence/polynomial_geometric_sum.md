---
title: 等比数列の多項式重み付き和
documentation_of: //linear_recurrence/polynomial_geometric_sum.hpp
---

# 等比数列の多項式重み付き和

Σ_(k=0..count-1) polynomial(k)*ratio^kを巨大なcountでも計算します。

[カテゴリ索引](README.md) / [今回の一覧](../b_16_20/README.md) / [操作できる図解](../../visualizations/b_16_20/index.html#polynomial_geometric_sum) / [一次資料と設計判断](../b_16_20/references.md)

## API・使用条件

`polynomial_geometric_sum<Mod,Root>(polynomial,ratio,uint64 count)`→int。有限多項式は昇冪順。

重み列は(1-rx)^(d+1)で消去でき、さらに(1-x)で割るとprefix和になります。n=0、空・零多項式は0。r=0/1も可、countはuint64最大値まで。d+3≤Lが安全な容量条件。階乗除算を使わず小標数でも恒等式が成り立ちます。

## usage

```cpp
#include <iostream>
#include <vector>
#include "../../linear_recurrence/polynomial_geometric_sum.hpp"

int main() {
    std::cout << cp::polynomial_geometric_sum(std::vector<int>{0, 1}, 2, 4) << "\n";
}
```

出力:

```text
34
```

[コンパイルして動く例](../../examples/linear_recurrence/polynomial_geometric_sum_b.cpp)。

## 計算量

次数dとしてO(d²+M(d+1)log(count+1))時間、O(d+1)領域。

## 使用する問題・検証範囲

幾何級数・べき和・重み付き線形漸化式のprefix。

直接有限和、ratio=0/1、uint64最大値、小さい法を比較。 `python3 verify/run_b_16_20.py` / `--sanitize`。使用例とこのusageの一致、単独・重複include、debug/releaseを確認します。オンライン提出・AC、全ジャッジ最大制約での時間内完了は未確認です。
