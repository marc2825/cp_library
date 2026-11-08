---
title: Montmort数 / Derangements
documentation_of: //combinatorics/derangements.hpp
---

# Montmort数 / Derangements

固定点が一つもない順列の個数を求めます。

[実装](../../combinatorics/derangements.hpp) / [操作できる図解](../../visualizations/math_a/index.html#derangements) / [カテゴリ一覧](README.md) / [補完項目・検証](../math_a/README.md)

## API・使用条件

`derangements(max_n,mod=998244353)` → vector<long long>、D(0)=1、D(1)=0。

N>=0、mod>0。法1・合成数・LLONG_MAXに対応。不正入力はinvalid_argument。

## 原理と計算量

要素nの行き先をn-1通りから選び、その相手と2-cycleにする場合と長いcycleへ挿入する場合へ分けてD(n)=(n-1)(D(n-1)+D(n-2))です。

O(N)回の安全なmod演算/O(N)領域。大きな法の乗算にはO(log mod)かかる場合があります。

## 使用例

```cpp
#include <iostream>
#include <vector>
#include "combinatorics/derangements.hpp"

int main() {
    std::cout << cp::derangements(5)[5] << "\n";
}
```

出力:

```text
44
```

[動く使用例](../../examples/combinatorics/derangements.cpp)。C++17でリポジトリルートをincludeパスに指定します。

## 問題例・検証範囲・参考資料

問題例: [Library Checker Montmort Number](https://judge.yosupo.jp/problem/montmort_number_mod)。

`python3 verify/run_math_a.py` と `--sanitize` で、使用例・単独/重複include・複数翻訳単位・素朴解との比較を実行します。[独立した検証](../../verify/math_a/)では空入力、同値、法1、使用条件内の境界も扱います。オンラインACは未確認です。図解は小さい入力向けの教材で、数値範囲と簡略化は[図の仕様](../../visualizations/math_a/README.md)を参照してください。

参考: [NIST DLMF 順列とcycle](https://dlmf.nist.gov/26.13)。全体の設計判断は[一次資料一覧](../math_a/references.md)に記載しています。
