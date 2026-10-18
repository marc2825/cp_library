---
title: 線形漸化式の推定と第N項
documentation_of: //linear_recurrence/infer_nth.hpp
---

# 線形漸化式の推定と第N項

既存のBMによる推定とKitamasaによる延長の公開APIを共有します。

[カテゴリ索引](README.md) / [今回の一覧](../b_16_20/README.md) / [操作できる図解](../../visualizations/b_16_20/index.html#infer_nth) / [一次資料と設計判断](../b_16_20/references.md)

## API・使用条件

`linear_recurrence_from_prefix<Mod>(samples,uint64 n)`。Modは素数。係数順は直前の項から。

[既存BMの詳細](berlekamp_massey.md)。観測範囲内は入力値を返し、空・全零は零列として延長。真の列が次数d以下の線形漸化式を持つ前提なら2d観測で復元可能。有限prefixだけで未知の将来は保証しません。

## usage

```cpp
#include <iostream>
#include <vector>
#include "../../linear_recurrence/infer_nth.hpp"

int main() {
    std::cout << cp::linear_recurrence_from_prefix(std::vector<int>{0, 1, 1, 2, 3, 5, 8, 13}, 20) << "\n";
}
```

出力:

```text
6765
```

[コンパイルして動く例](../../examples/linear_recurrence/infer_nth_b.cpp)。

## 計算量

観測S個でO(S²+S log Mod)、推定次数dで延長O(d² log(n+1))時間、O(S+d)領域。

## 使用する問題・検証範囲

[Find Linear Recurrence](https://judge.yosupo.jp/problem/find_linear_recurrence)、[K-th Term of Linear Recurrence](https://judge.yosupo.jp/problem/kth_term_of_linearly_recurrent_sequence)。

既存漸化式の逐次生成、BMとKitamasaの既存比較を共有。 `python3 verify/run_b_16_20.py` / `--sanitize`。使用例とこのusageの一致、単独・重複include、debug/releaseを確認します。オンライン提出・AC、全ジャッジ最大制約での時間内完了は未確認です。
