---
title: Mertens Function
documentation_of: //number_theory_convolution/mertens.hpp
---

# Mertens Function

[カテゴリ索引](README.md) / [B項目の対応表](../b_12_15/README.md) / [操作できる図解](../../visualizations/b_12_15/index.html#mertens) / [実装](../../number_theory_convolution/mertens.hpp) / [一次資料と設計](../b_12_15/references.md)

M(N)=Σ_{i<=N}μ(i)を、篩prefixと商区間のmemo再帰で計算します。

## usage

```cpp
#include <iostream>
#include "number_theory_convolution/mertens.hpp"
int main() {
    cp::MertensFunction m(4); std::cout << cp::optimization_to_string(m(10)) << '\n';
}
```

出力:

```text
-1
```

## API・使用条件

`MertensFunction(int precompute=1000000)`、`operator()(uint64_t n)`→OptimizationWide、memo_size()。precompute>=1、M(0)=0。

## 仕組み・境界条件

M(N)=1−Σ_{i=2}^N M(floor(N/i))を使用。負のMertens値と商区間の中間積もsigned128で保持します。複数queryでmemoを共有します。φの総和と同様、全64bit入力を高速に処理できるという保証ではありません。

## 計算量

前計算L、単一上限Nに対し期待O(L+N/sqrt L)時間、O(L+N/L)領域が目安。 標準整数の演算をO(1)として数えています。64bitの汎用mod乗算は最大O(log m)の加算を使うため、それを使う処理は別途その係数が掛かります。

## 使用する問題・検証

互いに素な対の個数やMöbius反転のprefix計算。 [動く小問題](../../examples/number_theory_convolution/mertens_b.cpp)。

`python3 verify/run_b_12_15.py` / `--sanitize`。[独立した比較方法](../b_12_15/README.md#ローカル検証)に従い、全列挙・定義からの直接計算・Python任意精度と比較します。使用例は文書中のコードと一致を検査してコンパイル・実行します。オンライン提出・ACは未確認です。図解の小入力化と代替計算は[図解の範囲](../../visualizations/b_12_15/README.md)に明記しています。
