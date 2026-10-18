---
title: 標本点シフト
documentation_of: //combinatorics/shift_sample_points.hpp
---

# 標本点シフト

[カテゴリ索引](README.md) / [B項目の対応表](../b_12_15/README.md) / [操作できる図解](../../visualizations/b_12_15/index.html#shift_sample_points) / [実装](../../combinatorics/shift_sample_points.hpp) / [一次資料と設計](../b_12_15/references.md)

f(0),...,f(N−1)から補間し、連続する別の点での値を既存の多点評価で求めます。

## usage

```cpp
#include <iostream>
#include "combinatorics/shift_sample_points.hpp"
int main() {
    auto y=cp::shift_sample_points(std::vector<int>{0,1,4},3,3);
    std::cout << y[0] << ' ' << y[1] << ' ' << y[2] << '\n';
}
```

出力:

```text
9 16 25
```

## API・使用条件

`shift_sample_points<Mod,Root>(samples,long long first,size_t count)`→vector<int>。deg f<N、N<=Mod、Nとcountはfps_max_size()未満（ProductTreeの上限）。firstは負数や法の周回も可。空samplesは零多項式として扱う。

## 仕組み・境界条件

既存polynomial_interpolationとmultipoint_evaluationを共有し、専用O(N log N)の連続点用畳み込みは実装しません。入力点は法で相異なる必要があり、出力点の重複は許します。count=0は空を返します。

## 計算量

O((N+M) log²(N+M+1))時間、ProductTreeの領域O((N+M) log(N+M+1))。 標準整数の演算をO(1)として数えています。64bitの汎用mod乗算は最大O(log m)の加算を使うため、それを使う処理は別途その係数が掛かります。

## 使用する問題・検証

[Shift of Sampling Points of Polynomial](https://judge.yosupo.jp/problem/shift_of_sampling_points_of_polynomial)。 [動く小問題](../../examples/combinatorics/shift_sample_points_b.cpp)。

`python3 verify/run_b_12_15.py` / `--sanitize`。[独立した比較方法](../b_12_15/README.md#ローカル検証)に従い、全列挙・定義からの直接計算・Python任意精度と比較します。使用例は文書中のコードと一致を検査してコンパイル・実行します。オンライン提出・ACは未確認です。図解の小入力化と代替計算は[図解の範囲](../../visualizations/b_12_15/README.md)に明記しています。
