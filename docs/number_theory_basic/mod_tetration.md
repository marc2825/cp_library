---
title: modテトレーション
documentation_of: //number_theory_basic/mod_tetration.hpp
---

# modテトレーション

[カテゴリ索引](README.md) / [B項目の対応表](../b_12_15/README.md) / [操作できる図解](../../visualizations/b_12_15/index.html#mod_tetration) / [実装](../../number_theory_basic/mod_tetration.hpp) / [一次資料と設計](../b_12_15/references.md)

Euler φの連鎖で指数を縮約し、実際の指数がφ以上かをcapped計算で判定して必要なφを足します。

## usage

```cpp
#include <iostream>
#include "number_theory_basic/mod_tetration.hpp"
int main() {
    std::cout << cp::mod_tetration(2,4,1000) << '\n';
}
```

出力:

```text
536
```

## API・使用条件

`mod_tetration(uint64_t a,uint64_t height,long long modulus)`。法>0、有限高さのa↑↑heightを法で返す。高さ0は1、0^0=1。

## 仕組み・境界条件

aと法が非互いに素でも、小さな指数を誤ってφ以上と扱わないよう飽和判定を行います。a=0の偶奇・a=1を個別処理し、巨大なheightを回数分ループしません。指数縮約値はuint64で保持します。無限高さの極限は対象外。

## 計算量

φ連鎖の長さL<=64に対しO(L log m)回程度のmod乗算とcapped判定、加えて各φの64bit素因数分解。領域O(L)。Pollard–Rhoの最悪時間保証なし。 標準整数の演算をO(1)として数えています。64bitの汎用mod乗算は最大O(log m)の加算を使うため、それを使う処理は別途その係数が掛かります。

## 使用する問題・検証

[Tetration Mod](https://judge.yosupo.jp/problem/tetration_mod)。 [動く小問題](../../examples/number_theory_basic/mod_tetration_b.cpp)。

`python3 verify/run_b_12_15.py` / `--sanitize`。[独立した比較方法](../b_12_15/README.md#ローカル検証)に従い、全列挙・定義からの直接計算・Python任意精度と比較します。使用例は文書中のコードと一致を検査してコンパイル・実行します。オンライン提出・ACは未確認です。図解の小入力化と代替計算は[図解の範囲](../../visualizations/b_12_15/README.md)に明記しています。
