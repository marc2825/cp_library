---
title: 整数の分割数
documentation_of: //combinatorics/partition_numbers.hpp
---

# 整数の分割数

正整数の和による整数分割を、和の順序を区別せずに数えます。

[実装](../../combinatorics/partition_numbers.hpp) / [操作できる図解](../../visualizations/math_a/index.html#partition_numbers) / [カテゴリ一覧](README.md) / [補完項目・検証](../math_a/README.md)

## API・使用条件

`partition_numbers(max_n,mod=998244353)` → vector<long long>、p(0)=1、添字0..N。

N>=0、mod>0。法1・合成数・LLONG_MAXに対応。不正入力はinvalid_argument。

## 原理と計算量

Eulerの五角数定理より、一般化五角数k(3k±1)/2だけ前の値を符号+,+,-,-,...で足します。制限付きの集合分割とは異なる整数分割です。

O(N sqrt N)加減算/O(N)領域。

## 使用例

```cpp
#include <iostream>
#include <vector>
#include "combinatorics/partition_numbers.hpp"

int main() {
    std::cout << cp::partition_numbers(10)[10] << "\n";
}
```

出力:

```text
42
```

[動く使用例](../../examples/combinatorics/partition_numbers.cpp)。C++17でリポジトリルートをincludeパスに指定します。

## 問題例・検証範囲・参考資料

問題例: [Library Checker Partition Function](https://judge.yosupo.jp/problem/partition_function)。

`python3 verify/run_math_a.py` と `--sanitize` で、使用例・単独/重複include・複数翻訳単位・素朴解との比較を実行します。[独立した検証](../../verify/math_a/)では空入力、同値、法1、使用条件内の境界も扱います。オンラインACは未確認です。図解は小さい入力向けの教材で、数値範囲と簡略化は[図の仕様](../../visualizations/math_a/README.md)を参照してください。

参考: [NIST DLMF 整数分割](https://dlmf.nist.gov/26.9)、[生成関数](https://dlmf.nist.gov/27.14)。全体の設計判断は[一次資料一覧](../math_a/references.md)に記載しています。
