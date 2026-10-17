---
title: Stirling数（第1種・第2種）
documentation_of: //combinatorics/stirling.hpp
---

# Stirling数（第1種・第2種）

順列のcycle数と集合分割のブロック数を数える三角表を構築します。

[実装](../../combinatorics/stirling.hpp) / [操作できる図解](../../visualizations/math_a/index.html#stirling) / [カテゴリ一覧](README.md) / [補完項目・検証](../math_a/README.md)

## API・使用条件

`stirling_first(max_n,mod=998244353,signed_values=false)`、`stirling_second(max_n,mod=998244353)` → vector<vector<long long>>。第n行は添字0..n。第1種の既定値は符号なしc(n,k)、trueでs(n,k)=(-1)^(n-k)c(n,k)。

N>=0、mod>0。不正値はinvalid_argument。法1・合成数・LLONG_MAXの法を使えます。範囲外の表添字へはアクセスしないこと。0行は{1 mod mod}、n>0の0列は0です。

## 原理と計算量

第1種は新しい要素を独立のcycleにするか既存cycleのn-1箇所に挿入します。第2種は独立のブロックにするか既存のkブロックの一つへ入れます。

O(N^2)回の安全なmod演算/O(N^2)領域。大きい法の乗算はO(log mod)加算になる場合があります。

## 使用例

```cpp
#include <iostream>
#include <vector>
#include "combinatorics/stirling.hpp"

int main() {
    auto first=cp::stirling_first(5); auto second=cp::stirling_second(5);
    std::cout << first[5][2] << " " << second[5][2] << "\n";
}
```

出力:

```text
50 15
```

[動く使用例](../../examples/combinatorics/stirling.cpp)。C++17でリポジトリルートをincludeパスに指定します。

## 問題例・検証範囲・参考資料

問題例: [Library Checker Stirling Number of the First Kind (Small)](https://judge.yosupo.jp/problem/stirling_number_of_the_first_kind_small)、[第2種](https://judge.yosupo.jp/problem/stirling_number_of_the_second_kind_small)。

`python3 verify/run_math_a.py` と `--sanitize` で、使用例・単独/重複include・複数翻訳単位・素朴解との比較を実行します。[独立した検証](../../verify/math_a/)では空入力、同値、法1、使用条件内の境界も扱います。オンラインACは未確認です。図解は小さい入力向けの教材で、数値範囲と簡略化は[図の仕様](../../visualizations/math_a/README.md)を参照してください。

参考: [NIST DLMF 26.8](https://dlmf.nist.gov/26.8)。全体の設計判断は[一次資料一覧](../math_a/references.md)に記載しています。
