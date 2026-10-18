---
title: P-recursive数列
documentation_of: //linear_recurrence/p_recursive.hpp
---

# P-recursive数列

多項式係数を持つ既知の線形漸化式からprefixを逐次生成します。

[カテゴリ索引](README.md) / [今回の一覧](../b_16_20/README.md) / [操作できる図解](../../visualizations/b_16_20/index.html#p_recursive) / [一次資料と設計判断](../b_16_20/references.md)

## API・使用条件

`p_recursive_prefix<Mod>(initial,p,count)`。`Σ_(j=0..r) p[j](t)*a[t+j]=0`。p[j]は昇冪係数、r≥1、initialはr項。

各ステップで`p[r](t)`が非零で必要。零になればdomain_error（非一意や矛盾を自動で判定しません）。countが初期項数未満なら切り詰め。未知漸化式の推定や巨大N項の高速計算は扱いません。

## usage

```cpp
#include <iostream>
#include <vector>
#include "../../linear_recurrence/p_recursive.hpp"

int main() {
    auto a = cp::p_recursive_prefix(std::vector<int>{1}, std::vector<std::vector<int>>{{-1, -1}, {1}}, 6);
    std::cout << a[5] << "\n";
}
```

出力:

```text
120
```

[コンパイルして動く例](../../examples/linear_recurrence/p_recursive_b.cpp)。

## 計算量

生成N項、各pの長さの和DとしてO(N(D+r+log Mod))時間、出力を含むO(N)追加領域。

## 使用する問題・検証範囲

階乗、Catalan数、超幾何型の係数漸化式。

階乗と直接逐次計算、小さい法で先頭係数が消える条件を検証。 `python3 verify/run_b_16_20.py` / `--sanitize`。使用例とこのusageの一致、単独・重複include、debug/releaseを確認します。オンライン提出・AC、全ジャッジ最大制約での時間内完了は未確認です。
