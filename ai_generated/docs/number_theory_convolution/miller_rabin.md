---
title: 64bit Miller–Rabin
documentation_of: //number_theory_convolution/miller_rabin.hpp
---

# 64bit Miller–Rabin

[カテゴリ索引](README.md) / [図解](../../visualizations/advanced/index.html#primality) / [実装](../../number_theory_convolution/miller_rabin.hpp) / [一次資料・設計判断](../advanced/references.md)

## usage

```cpp
#include <iostream>
#include "number_theory_convolution/miller_rabin.hpp"
int main() {
    std::cout << cp::is_prime_64(18446744073709551557ULL) << ' ' << cp::is_prime_64(341550071728321ULL) << '\n';
}
```

[実行できる使用例](../../examples/number_theory_convolution/miller_rabin.cpp)。C++17、namespace cp。ライブラリルートをincludeパスに追加してください。

## API・使用条件

`is_prime_64(uint64_t n)->bool`。0・1はfalse、uint64_t全域で決定的に判定。負数はAPIの入力ではありません。符号付き負値を暗黙変換して渡さないでください。外部ライブラリ・__int128・浮動小数点を使わず、安全なmod乗算を3.13から再利用します。

## 計算量・原理

7基底、O(log n)回のmod乗算。乗算は積がuint64内ならO(1)、そうでなければ加算倍増O(log n)。したがってビット演算を定数扱いした上界はO(log² n)、領域O(1)。

n−1=2^s dと分解し、a^dからの二乗列が素数の必要条件を満たすか確認します。基底2,325,9375,28178,450775,9780504,1795265022は64bit全域の決定的集合を使います。剰余乗算が不正確な実装ではこの保証は成立しません。

## 使用する問題・検証

[関連問題](https://judge.yosupo.jp/problem/primality_test)。関連問題は用途の例であり、このAPIだけでそのまま全て解けるという意味ではありません。特に動的連結性の関連問題は追加の成分和が必要です。

[追加分の検証入口](../../verify/run_advanced.py)で、素朴解との比較、空・重複・境界、非可換集約、履歴分岐、繰返し実行、20万要素の主要構造を検査します。実際の範囲は[検証一覧](../advanced/README.md)を参照。オンライン提出のACは未確認です。

図解は学習用の小さい入力で内部状態・候補・論理的な操作を追えます。[簡略化の範囲](../../visualizations/advanced/README.md)を参照してください。
