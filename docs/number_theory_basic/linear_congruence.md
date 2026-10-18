---
title: 線形合同方程式
documentation_of: //number_theory_basic/linear_congruence.hpp
---

# 線形合同方程式

[カテゴリ索引](README.md) / [B項目の対応表](../b_12_15/README.md) / [操作できる図解](../../visualizations/b_12_15/index.html#linear_congruence) / [実装](../../number_theory_basic/linear_congruence.hpp) / [一次資料と設計](../b_12_15/references.md)

ax≡b mod mをgcdで可解判定し、全ての解を等差数列として返します。

## usage

```cpp
#include <iostream>
#include "number_theory_basic/linear_congruence.hpp"
int main() {
    auto r=cp::linear_congruence(6,8,14);
    std::cout << r->first << ' ' << r->step << ' ' << r->count << '\n';
}
```

出力:

```text
6 7 2
```

## API・使用条件

`linear_congruence(a,b,m)`→optional<LinearCongruenceResult>{first,step,count}。a,bはlong long全域、m>0。全解はfirst+t*step（0<=t<count）で[0,m)内。不可能はnullopt。

## 仕組み・境界条件

a=0,b=0なら全剰余が解。法1なら{0,1,1}。負数を正規化してからgcdを取り、LLONG_MINの絶対値を作りません。解を全件メモリへ列挙しないAPIです。

## 計算量

O(log m)のEuclidとmod乗算、O(1)領域。 標準整数の演算をO(1)として数えています。64bitの汎用mod乗算は最大O(log m)の加算を使うため、それを使う処理は別途その係数が掛かります。

## 使用する問題・検証

非互いに素な係数を含む合同式、一定周期の可解時刻。 [動く小問題](../../examples/number_theory_basic/linear_congruence_b.cpp)。

`python3 verify/run_b_12_15.py` / `--sanitize`。[独立した比較方法](../b_12_15/README.md#ローカル検証)に従い、全列挙・定義からの直接計算・Python任意精度と比較します。使用例は文書中のコードと一致を検査してコンパイル・実行します。オンライン提出・ACは未確認です。図解の小入力化と代替計算は[図解の範囲](../../visualizations/b_12_15/README.md)に明記しています。
