---
title: 64bit Pollard’s Rho
documentation_of: //number_theory_convolution/pollard_rho.hpp
---

# 64bit Pollard’s Rho

[カテゴリ索引](README.md) / [図解](../../visualizations/advanced/index.html#rho) / [実装](../../number_theory_convolution/pollard_rho.hpp) / [一次資料・設計判断](../advanced/references.md)

## usage

```cpp
#include <iostream>
#include "number_theory_convolution/pollard_rho.hpp"
int main() {
    for (auto p : cp::factorize_64(1000000016000000063ULL)) std::cout << p << ' ';
    std::cout << '\n';
}
```

[実行できる使用例](../../examples/number_theory_convolution/pollard_rho.cpp)。C++17、namespace cp。ライブラリルートをincludeパスに追加してください。

## API・使用条件

`factorize_64(uint64_t n,seed=固定値)->vector<uint64_t>` は昇順・重複ありの素因数列。`prime_factorization_64(n,seed)` は昇順の(素数,指数)。1は空、0はinvalid_argument。固定seedで再現可能、正確なuint64剰余演算と決定的Miller–Rabinを併用。旧3.13の試し割りprime_factorizationは変更しません。

## 計算量・原理

最小素因数pを発見するまでの反復は経験的・確率的解析で期待O(sqrt(p))が目安、最悪時間保証なし。1反復のmod乗算は標準C++17版で最大O(log n)。結果・分割スタックはO(log n)。

f(x)=x²+c mod nの列の衝突を利用します。Brent型の長さ倍増と128個ずつの積のGCDで、毎回GCDする費用を減らします。GCDがnになれば保存位置から1ステップずつ戻って調べ、失敗した試行はseedを進めてやり直します。

## 使用する問題・検証

[関連問題](https://judge.yosupo.jp/problem/factorize)。関連問題は用途の例であり、このAPIだけでそのまま全て解けるという意味ではありません。特に動的連結性の関連問題は追加の成分和が必要です。

[追加分の検証入口](../../verify/run_advanced.py)で、素朴解との比較、空・重複・境界、非可換集約、履歴分岐、繰返し実行、20万要素の主要構造を検査します。実際の範囲は[検証一覧](../advanced/README.md)を参照。オンライン提出のACは未確認です。

図解は学習用の小さい入力で内部状態・候補・論理的な操作を追えます。[簡略化の範囲](../../visualizations/advanced/README.md)を参照してください。
