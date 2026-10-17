---
title: Möbius μ
documentation_of: //number_theory_basic/mobius.hpp
---

# Möbius μ

平方因子の有無と素因数個数から、Möbius関数を計算します。

[実装](../../number_theory_basic/mobius.hpp) / [操作できる図解](../../visualizations/math_a/index.html#mobius) / [カテゴリ一覧](README.md) / [補完項目・検証](../math_a/README.md)

## API・使用条件

`mobius(uint64_t n)` → -1/0/1、`mobius_sieve(int limit)` → vector<int>（添字0..limit）。

n>=1、mu(1)=1。篩の0は便宜上0。単一値の0・負の篩上限はinvalid_argument。単一値は既存64bit素因数分解を共有します。

## 原理と計算量

同じ素因数が二度現れれば0、そうでなければ異なる素因数の個数だけ符号反転します。篩ではi=p*qでpがqを割れば0、それ以外は-mu(q)。

篩O(N)時間/O(N)領域。単一値はPollard–Rhoの素因数分解時間に従い、最悪時間保証はありません。

## 使用例

```cpp
#include <iostream>
#include <vector>
#include "number_theory_basic/mobius.hpp"

int main() {
    auto values=cp::mobius_sieve(12);
    std::cout << cp::mobius(30) << " " << values[12] << "\n";
}
```

出力:

```text
-1 0
```

[動く使用例](../../examples/number_theory_basic/mobius.cpp)。C++17でリポジトリルートをincludeパスに指定します。

## 問題例・検証範囲・参考資料

使用問題: 約数包除と[既存のGCD畳み込み](../number_theory_convolution/gcd_convolution.md)。

`python3 verify/run_math_a.py` と `--sanitize` で、使用例・単独/重複include・複数翻訳単位・素朴解との比較を実行します。[独立した検証](../../verify/math_a/)では空入力、同値、法1、使用条件内の境界も扱います。オンラインACは未確認です。図解は小さい入力向けの教材で、数値範囲と簡略化は[図の仕様](../../visualizations/math_a/README.md)を参照してください。

参考: [NIST DLMF 数論的関数](https://dlmf.nist.gov/27.2)。全体の設計判断は[一次資料一覧](../math_a/references.md)に記載しています。
