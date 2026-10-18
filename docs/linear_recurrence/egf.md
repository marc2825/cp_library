---
title: 指数型母関数（EGF）の演算
documentation_of: //linear_recurrence/egf.hpp
---

# 指数型母関数（EGF）の演算

EGFの配列をラベル付き個数a_nとして受け取り、階乗で通常係数へ変換して演算します。

[カテゴリ索引](README.md) / [今回の一覧](../b_16_20/README.md) / [操作できる図解](../../visualizations/b_16_20/index.html#egf) / [一次資料と設計判断](../b_16_20/references.md)

## API・使用条件

`egf_to_ordinary<Mod>(a)`、`egf_from_ordinary<Mod>(a)`、`egf_multiply<Mod,Root>(a,b,n)`、`egf_inv/log/exp(a,n)`。配列aはΣa[k]x^k/k!。

使用する精度n≤Modかつn≤fps_max_size。積c_n=Σ binom(n,k)a_k b_(n-k)。inv/log/expの定数項条件はFPS版と共通。n=0は空。加減算は通常の係数ごとの加減算です。

## usage

```cpp
#include <iostream>
#include <vector>
#include "../../linear_recurrence/egf.hpp"

int main() {
    auto c = cp::egf_multiply(std::vector<int>(5, 1), std::vector<int>(5, 1), 5);
    std::cout << c[4] << "\n";
}
```

出力:

```text
16
```

[コンパイルして動く例](../../examples/linear_recurrence/egf_b.cpp)。

## 計算量

変換O(n+log Mod)、積・inv/log/expはO(M(n)+n+log Mod)時間、O(n)領域。

## 使用する問題・検証範囲

ラベル付き集合の積・集合構成、[Exp of FPS](https://judge.yosupo.jp/problem/exp_of_formal_power_series)の階乗スケール変換。

Pascalの二項係数による直接積、階乗変換とexp/logの往復を比較。 `python3 verify/run_b_16_20.py` / `--sanitize`。使用例とこのusageの一致、単独・重複include、debug/releaseを確認します。オンライン提出・AC、全ジャッジ最大制約での時間内完了は未確認です。
