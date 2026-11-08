---
title: Floor Sum
documentation_of: //number_theory_basic/floor_sum.hpp
---

# Floor Sum

負の係数にも対応し、一次式を割って床を取った総和を求めます。

[実装](../../number_theory_basic/floor_sum.hpp) / [操作できる図解](../../visualizations/math_a/index.html#floor_sum) / [カテゴリ一覧](README.md) / [補完項目・検証](../math_a/README.md)

## API・使用条件

`floor_sum(n,m,a,b)` → long long、`sum_{i=0}^{n-1} floor((a*i+b)/m)`。

0<=n<2^32、1<=m<2^32、a,bは全long long。負値は数学的な床で正規化します。不正範囲はinvalid_argument。正規化係数qa=floor(a/m), qb=floor(b/m)を使う`n(n-1)/2*qa`、`n*qb`、両者の和、最後の非負部分の加算がそれぞれlong longに収まる必要があります。途中で超える場合は最終的に相殺できてもoverflow_error。ACLのmod 2^64の返却契約とは異なります。n=0は係数にかかわらず0。

## 原理と計算量

a,bを商と非負剰余へ分けて、商の寄与を三角数とn倍で数えます。残った非負格子点を数える部分は軸を入れ替えるEuclid型の反復で縮めます。

O(log m)時間、追加領域O(1)。標準C++17のuint64_tで非負部分の中間積を計算し、__int128には依存しません。

## 使用例

```cpp
#include <iostream>
#include <vector>
#include "number_theory_basic/floor_sum.hpp"

int main() {
    std::cout << cp::floor_sum(4,10,6,3) << "\n";
}
```

出力:

```text
3
```

[動く使用例](../../examples/number_theory_basic/floor_sum.cpp)。C++17でリポジトリルートをincludeパスに指定します。

## 問題例・検証範囲・参考資料

問題例: [AtCoder practice2 C - Floor Sum](https://atcoder.jp/contests/practice2/tasks/practice2_c)。

`python3 verify/run_math_a.py` と `--sanitize` で、使用例・単独/重複include・複数翻訳単位・素朴解との比較を実行します。[独立した検証](../../verify/math_a/)では空入力、同値、法1、使用条件内の境界も扱います。オンラインACは未確認です。図解は小さい入力向けの教材で、数値範囲と簡略化は[図の仕様](../../visualizations/math_a/README.md)を参照してください。

参考: [ACL公式floor_sum仕様](https://atcoder.github.io/ac-library/production/document_ja/math.html)。全体の設計判断は[一次資料一覧](../math_a/references.md)に記載しています。
