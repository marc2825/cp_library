---
title: Euler φ
documentation_of: //number_theory_basic/euler_phi.hpp
---

# Euler φ

単一の64bit整数と、上限までの全整数についてEulerのφ関数を求めます。

[実装](../../number_theory_basic/euler_phi.hpp) / [操作できる図解](../../visualizations/math_a/index.html#euler_phi) / [カテゴリ一覧](README.md) / [補完項目・検証](../math_a/README.md)

## API・使用条件

`euler_phi(uint64_t n)` → uint64_t、`euler_phi_sieve(int limit)` → vector<uint64_t>（添字0..limit）。

n>=1、phi(1)=1。篩の0は便宜上0。単一値の0・負の篩上限はinvalid_argument。単一値は既存Pollard–Rho/Miller–Rabinを共有します。

## 原理と計算量

異なる素因数pごとにresult=result/p*(p-1)。篩ではi=p*qのpがqも割るかでphi(i)=p*phi(q)または(p-1)*phi(q)です。

篩は既存SPFを使いO(N)時間/O(N)領域。単一値は64bit素因数分解の時間＋異なる素因数個数の積操作。Pollard–Rhoに最悪時間保証はありません。

## 使用例

```cpp
#include <iostream>
#include <vector>
#include "number_theory_basic/euler_phi.hpp"

int main() {
    auto values=cp::euler_phi_sieve(10);
    std::cout << cp::euler_phi(36) << " " << values[10] << "\n";
}
```

出力:

```text
12 4
```

[動く使用例](../../examples/number_theory_basic/euler_phi.cpp)。C++17でリポジトリルートをincludeパスに指定します。

## 問題例・検証範囲・参考資料

問題例: [AOJ NTL_1_D](https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=NTL_1_D)。

`python3 verify/run_math_a.py` と `--sanitize` で、使用例・単独/重複include・複数翻訳単位・素朴解との比較を実行します。[独立した検証](../../verify/math_a/)では空入力、同値、法1、使用条件内の境界も扱います。オンラインACは未確認です。図解は小さい入力向けの教材で、数値範囲と簡略化は[図の仕様](../../visualizations/math_a/README.md)を参照してください。

参考: [NIST DLMF 27.3](https://dlmf.nist.gov/27.3)。全体の設計判断は[一次資料一覧](../math_a/references.md)に記載しています。
