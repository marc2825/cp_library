---
title: Bell数
documentation_of: //combinatorics/bell.hpp
---

# Bell数

N要素の集合を、順序のない非空ブロックへ分ける方法を数えます。

[実装](../../combinatorics/bell.hpp) / [操作できる図解](../../visualizations/math_a/index.html#bell) / [カテゴリ一覧](README.md) / [補完項目・検証](../math_a/README.md)

## API・使用条件

`bell_numbers(max_n,mod=998244353)` → vector<long long>、添字0..N。B(0)=1。

N>=0、mod>0。法1・合成数・LLONG_MAXも可。不正入力はinvalid_argument。

## 原理と計算量

Bell三角形の次の行の先頭を前の行の末尾にし、左隣と左上の和で行を作ります。各行の先頭がBell数です。集合分割の全列挙と比較します。

O(N^2)加算/O(N)領域。階乗・逆元を使いません。

## 使用例

```cpp
#include <iostream>
#include <vector>
#include "combinatorics/bell.hpp"

int main() {
    std::cout << cp::bell_numbers(5)[5] << "\n";
}
```

出力:

```text
52
```

[動く使用例](../../examples/combinatorics/bell.cpp)。C++17でリポジトリルートをincludeパスに指定します。

## 問題例・検証範囲・参考資料

使用問題: [Stirling第2種](stirling.md)を全ブロック数で合計する集合分割の数え上げ。

`python3 verify/run_math_a.py` と `--sanitize` で、使用例・単独/重複include・複数翻訳単位・素朴解との比較を実行します。[独立した検証](../../verify/math_a/)では空入力、同値、法1、使用条件内の境界も扱います。オンラインACは未確認です。図解は小さい入力向けの教材で、数値範囲と簡略化は[図の仕様](../../visualizations/math_a/README.md)を参照してください。

参考: [NIST DLMF 26.7](https://dlmf.nist.gov/26.7)。全体の設計判断は[一次資料一覧](../math_a/references.md)に記載しています。
