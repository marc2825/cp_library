---
title: Lucasの定理
documentation_of: //number_theory_basic/lucas.hpp
---

# Lucasの定理

[カテゴリ索引](README.md) / [B項目の対応表](../b_12_15/README.md) / [操作できる図解](../../visualizations/b_12_15/index.html#lucas) / [実装](../../number_theory_basic/lucas.hpp) / [一次資料と設計](../b_12_15/references.md)

素数法の二項係数を、n,kのp進各桁の二項係数の積へ分けます。

## usage

```cpp
#include <iostream>
#include "number_theory_basic/lucas.hpp"
int main() {
    cp::LucasBinomial c(7); std::cout << c(100,50) << '\n';
}
```

出力:

```text
4
```

## API・使用条件

`LucasBinomial(int prime)`、`operator()(uint64_t n,uint64_t k)`、modulus()。primeは素数、n,kはuint64全域。k>nは0。

## 仕組み・境界条件

p-1まで既存FactorialTableを前計算します。素数性は検査し、合成数は拒否。pをintで受けますが、大きなpの配列を無条件で確保できるという保証はありません。合成数法のLucas拡張は対象外。

## 計算量

構築O(p)時間・領域、問い合わせO(log_p(n+1))。 p<2^31 なので各 mod 乗算は直接計算で O(1) です。

## 使用する問題・検証

巨大n、小さい素数法の二項係数。 [動く小問題](../../examples/number_theory_basic/lucas_b.cpp)。

`python3 verify/run_b_12_15.py` / `--sanitize`。[検証](../../verify/b_12_15/number_test.cpp)は素数2,3,5,7,11,17,101で n≤350 の全ての k を Pascal の三角形 mod p と比較し、n=UINT64_MAX, k=0 も確認します。使用例は文書中のコードと一致を検査してコンパイル・実行します。オンライン提出・ACは未確認です。図解の小入力化と代替計算は[図解の範囲](../../visualizations/b_12_15/README.md)に明記しています。
