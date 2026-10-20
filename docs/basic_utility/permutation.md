---
title: 順列の合成・逆・サイクル・符号・冪
documentation_of: //basic_utility/permutation.hpp
---

# 順列の合成・逆・サイクル・符号・冪（U-05）

[ユーティリティ索引](README.md) / [今回の14項目](../utility_search/README.md) / [入力を変えられる図解](../../visualizations/utility_search/index.html#permutation)

順列を互いに素なサイクルに分け、冪を各サイクル内の循環移動として計算します。符号はサイクル数cに対し (−1)^(n−c) です。

## 使用例

```cpp
#include <iostream>
#include "basic_utility/permutation.hpp"
int main() {
    std::vector<int> p{2, 0, 1, 4, 3};
    auto power = cp::permutation_power(p, -2);
    for (std::size_t i = 0; i < power.size(); ++i) std::cout << (i ? " " : "") << power[i];
    std::cout << '\n' << cp::permutation_sign(p) << '\n';
}
```

出力:

```text
2 0 1 3 4
-1
```

## API・使用条件

入力は `[0,n)` の置換を表す `vector<int>`。空順列を許し、不正な重複・範囲外値は演算時に `invalid_argument`。

- `is_permutation(p)` はbool、`validate_permutation(p)` は不正なら例外。
- `permutation_compose(p,q)` は **pをqの後に作用**させ、結果 `r[i]=p[q[i]]`。長さ不一致は例外。
- `permutation_inverse(p)` は逆置換。
- `permutation_cycles(p)` は各サイクルを最小要素からpの向きに辿り、サイクルの先頭が昇順。固定点も含みます。
- `permutation_sign(p)` は偶置換なら+1、奇置換なら−1（空は+1）。
- `permutation_power(p, int64_t k)` は負の指数も可。k=0は恒等置換、INT64_MINも直接剰余を取って扱います。

辞書順rank/unrankは既存の [permutation_rank.hpp](permutation_rank.md) を共有します。

## 計算量

各APIはO(n)時間・O(n)記憶。冪も指数の大きさに依存せず、サイクル長で剰余を取ります。

## 使用する問題・検証

操作列の繰り返しによる並べ替え、逆変換、置換の偶奇。n≤8の全順列で、合成による逆の確認、素朴な反復、転倒数の偶奇と比較します。

[共通の比較テスト](../../verify/utility_search/unit_test.cpp)で空入力・端の値・重複・不正入力と独立した素朴解を確認します。`python3 verify/run_utility_search.py`。実行した構成と図解の制約は[検証記録](../utility_search/README.md)を参照してください。オンライン提出ACは主張しません。

## 参考資料

[原理・出典・ライセンス一覧](../utility_search/references.md)。
