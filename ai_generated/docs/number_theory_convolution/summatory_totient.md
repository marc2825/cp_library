---
title: Euler φの総和
documentation_of: //number_theory_convolution/summatory_totient.hpp
---

# Euler φの総和

[カテゴリ索引](README.md) / [B項目の対応表](../b_12_15/README.md) / [操作できる図解](../../visualizations/b_12_15/index.html#summatory_totient) / [実装](../../number_theory_convolution/summatory_totient.hpp) / [一次資料と設計](../b_12_15/references.md)

小さいprefixを篩で持ち、大きいNは畳み込み恒等式と商区間のmemoから計算するDu Jiao方式です。

## usage

```cpp
#include <iostream>
#include "number_theory_convolution/summatory_totient.hpp"
int main() {
    cp::SummatoryTotient sum(16); std::cout << cp::optimization_to_string(sum(10)) << '\n';
}
```

出力:

```text
32
```

## API・使用条件

`SummatoryTotient(int precompute=1000000)`、`operator()(uint64_t n)`→OptimizationWide、memo_size()。precompute>=1。S(0)=0、φ(1)=1。

## 仕組み・境界条件

Σ_{d<=N}φ(d)floor(N/d)=N(N+1)/2を利用します。三角数は掛ける前に2で割り、uint64上限付近でも128bit途中積を超過させません。memoの追加でメモリが増えます。数値型の範囲と、実用上のNの上限は別です。

## 計算量

前計算L、単一上限Nに対し期待O(L+N/sqrt L)時間、O(L+N/L)領域が目安。memoは複数問い合わせで保持。 標準整数の演算をO(1)として数えています。64bitの汎用mod乗算は最大O(log m)の加算を使うため、それを使う処理は別途その係数が掛かります。

## 使用する問題・検証

[Sum of Totient Function](https://judge.yosupo.jp/problem/sum_of_totient_function)と同じ数学的総和（本APIの返値はmodではなく整数）。 [動く小問題](../../examples/number_theory_convolution/summatory_totient_b.cpp)。

`python3 verify/run_b_12_15.py` / `--sanitize`。[独立した比較方法](../b_12_15/README.md#ローカル検証)に従い、全列挙・定義からの直接計算・Python任意精度と比較します。使用例は文書中のコードと一致を検査してコンパイル・実行します。オンライン提出・ACは未確認です。図解の小入力化と代替計算は[図解の範囲](../../visualizations/b_12_15/README.md)に明記しています。
