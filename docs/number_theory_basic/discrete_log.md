---
title: Discrete Log（BSGS）
documentation_of: //number_theory_basic/discrete_log.hpp
---

# Discrete Log（BSGS）

[カテゴリ索引](README.md) / [B項目の対応表](../b_12_15/README.md) / [操作できる図解](../../visualizations/b_12_15/index.html#discrete_log) / [実装](../../number_theory_basic/discrete_log.hpp) / [一次資料と設計](../b_12_15/references.md)

gcdで非互いに素な部分を除き、baby-stepと逆方向giant-stepを照合して最小の非負指数を返します。

## usage

```cpp
#include <iostream>
#include "number_theory_basic/discrete_log.hpp"
int main() {
    std::cout << *cp::discrete_log(8,6,10) << '\n';
}
```

出力:

```text
4
```

## API・使用条件

`discrete_log(a,b,m,max_baby_steps=2000000)`→optional<uint64_t>。a,bはlong longで負数も正規化、mは正のlong long。a^x≡b mod mの最小x、0^0=1。不存在はnullopt。

## 仕組み・境界条件

babyの同じ剰余には最小jを保存し、giantを小さい順に調べます。法1の答えは0。sqrt(m)が予算を超える場合はlength_errorを返し、不存在と混同しません。mの64bit全域でsqrt(m)領域を常に確保できるという保証はありません。

## 計算量

O(sqrt(m))期待辞書操作とmod乗算、O(sqrt(m))領域。gcd除去と逆元・累乗はO(log m)段。 標準整数の演算をO(1)として数えています。64bitの汎用mod乗算は最大O(log m)の加算を使うため、それを使う処理は別途その係数が掛かります。

## 使用する問題・検証

[Discrete Logarithm](https://judge.yosupo.jp/problem/discrete_logarithm_mod)。 [動く小問題](../../examples/number_theory_basic/discrete_log_b.cpp)。

`python3 verify/run_b_12_15.py` / `--sanitize`。[独立した比較方法](../b_12_15/README.md#ローカル検証)に従い、全列挙・定義からの直接計算・Python任意精度と比較します。使用例は文書中のコードと一致を検査してコンパイル・実行します。オンライン提出・ACは未確認です。図解の小入力化と代替計算は[図解の範囲](../../visualizations/b_12_15/README.md)に明記しています。
