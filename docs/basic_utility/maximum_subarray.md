---
title: 最大部分配列和・区間復元
documentation_of: //basic_utility/maximum_subarray.hpp
---

# 最大部分配列和・区間復元（U-02）

[ユーティリティ索引](README.md) / [今回の14項目](../utility_search/README.md) / [入力を変えられる図解](../../visualizations/utility_search/index.html#maximum_subarray)

Kadane法で、各右端に終わる最大和の区間を保持します。現在の和が負なら次の要素から再開し、和0なら左端を保持して同点規則を満たします。

## 使用例

```cpp
#include <iostream>
#include "basic_utility/maximum_subarray.hpp"
int main() {
    auto answer = cp::maximum_subarray(std::vector<long long>{-2, 3, -1, 3, -5});
    std::cout << answer->sum << ' ' << answer->left << ' ' << answer->right << '\n';
    std::cout << cp::maximum_subarray(std::vector<int>{-4, -2}, true)->sum << '\n';
}
```

出力:

```text
5 1 4
0
```

## API・使用条件

`maximum_subarray(vector<T>, allow_empty=false)` → `optional<MaximumSubarrayResult<T>>`。結果は `sum,left,right`。

空入力かつ非空指定だけが `nullopt` です。全負値で非空指定なら最大の要素1個を返します。空区間を許す場合は `[0,0)`・和0を候補に含めます。同じ和なら `(left,right)` の辞書順最小を選びます。

整数型T（128bitも可）は加算を検査し、最大和が表現範囲を超えると `overflow_error`。`BigInt` など厳密な順序付き加法型も利用可能です。浮動小数点型では有限値・有限な中間和が前提で、比較は許容誤差なしです。

## 計算量

O(n)演算・結果を除きO(1)記憶。BigIntでは加算・比較の桁数コストが別にかかります。

## 使用する問題・検証

連続区間の利益最大化。全区間の和を独立に計算し、和だけでなく復元区間と同点規則まで照合します。

[共通の比較テスト](../../verify/utility_search/unit_test.cpp)で空入力・端の値・重複・不正入力と独立した素朴解を確認します。`python3 verify/run_utility_search.py`。実行した構成と図解の制約は[検証記録](../utility_search/README.md)を参照してください。オンライン提出ACは主張しません。

## 参考資料

[原理・出典・ライセンス一覧](../utility_search/references.md)。
