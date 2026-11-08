---
title: 有理型母関数
documentation_of: //linear_recurrence/rational_generating_function.hpp
---

# 有理型母関数

x=0で正則な有理式を約分し、足し算・掛け算・prefix・遠い係数抽出を提供します。

[カテゴリ索引](README.md) / [今回の一覧](../b_16_20/README.md) / [操作できる図解](../../visualizations/b_16_20/index.html#rational_generating_function) / [一次資料と設計判断](../b_16_20/references.md)

## API・使用条件

`RationalGeneratingFunction<Mod,Root>(P={},Q={1})`、`numerator()`、`denominator()`、`prefix(n)`、`coefficient(uint64 index)`、`operator+/*`。

入力分母の定数項は非零が必要。零式は0/1、共通因子で約分して分母定数を1にします。演算の全中間多項式の長さが既存FPS容量内で必要。極を持つLaurent級数は対象外。

## usage

```cpp
#include <iostream>
#include <vector>
#include "../../linear_recurrence/rational_generating_function.hpp"

int main() {
    cp::RationalGeneratingFunction<> a({1}, {1, -1, -1});
    std::cout << a.coefficient(6) << "\n";
}
```

出力:

```text
13
```

[コンパイルして動く例](../../examples/linear_recurrence/rational_generating_function_b.cpp)。

## 計算量

構築はEuclid法で最悪O(D M(D))、prefixはO(M(n))、遠い係数はO(M(D) log(index+1))。加算・乗算は積と再約分の費用。保存O(D)。

## 使用する問題・検証範囲

[K-th Term of Linearly Recurrent Sequence](https://judge.yosupo.jp/problem/kth_term_of_linearly_recurrent_sequence)。

直接prefix漸化式、和・積、全初期係数とBostan–Moriを比較。 `python3 verify/run_b_16_20.py` / `--sanitize`。使用例とこのusageの一致、単独・重複include、debug/releaseを確認します。オンライン提出・AC、全ジャッジ最大制約での時間内完了は未確認です。
