---
title: 半分全列挙・部分和の個数と復元
documentation_of: //basic_utility/meet_in_the_middle.hpp
---

# 半分全列挙・部分和の個数と復元（U-04）

[ユーティリティ索引](README.md) / [今回の14項目](../utility_search/README.md) / [入力を変えられる図解](../../visualizations/utility_search/index.html#meet_in_the_middle)

半分ごとに全ての部分和を整列し、片側を昇順、もう片側の境界を降順に動かします。全体の2^n組を作らずに個数と最適な組を求めます。

## 使用例

```cpp
#include <iostream>
#include "basic_utility/meet_in_the_middle.hpp"
int main() {
    cp::MeetInTheMiddle search({3, -2, 5, 1});
    auto best = search.best_leq(6);
    std::cout << static_cast<long long>(best->sum) << ' ' << best->mask << ' '
              << search.count_equal(6) << '\n';
}
```

出力:

```text
6 7 2
```

## API・使用条件

`sorted_subset_sums(vector<int64_t>)` は23要素までの全 `SubsetSum{sum,mask}` を `(sum,mask)` 昇順で返します。空集合も1件です。重複和を除去しません。

`MeetInTheMiddle(values)` は46要素までを前後に分割。`left_sums()` / `right_sums()` は各半分のソート済みリスト（maskは各半分のローカル添字）へのconst参照です。

- `count_leq(target)`、`count_equal(target)`、`count_range(low,high)` は条件を満たす部分集合数。rangeは `[low,high)`、逆順は `invalid_argument`。
- `best_leq(target)` は上限以下の最大和、同じ和なら最小の元添字bitmaskをoptionalで返します。該当なしは `nullopt`。maskのbit iが元の要素iの採用を示します。
- 和・上限は `__int128_t`、個数・maskはuint64。64bit入力46個の総和は128bitに収まり、最大個数2^46もuint64に収まります。負の値も扱えます。
- 要素数超過は `length_error`。指数メモリが必要で、46要素は環境によってメモリ不足になります。

## 計算量

単体部分和はシフト済みソート列のマージを繰り返し O(2^m)時間・記憶。クラスの構築・各問い合わせは A=2^floor(n/2), B=2^ceil(n/2) としてO(A+B)、記憶O(A+B)。n=46では結果表だけでも通常の128bit整列で約512MiB、構築中の一時表はさらに増えます。

## 使用する問題・検証

要素数約40のSubset Sumの解数、上限以下の最大部分和と採用要素の復元。全bitmask走査と個数・最適和・復元maskを比較し、64bit極値も確認します。

[共通の比較テスト](../../verify/utility_search/unit_test.cpp)で空入力・端の値・重複・不正入力と独立した素朴解を確認します。`python3 verify/run_utility_search.py`。実行した構成と図解の制約は[検証記録](../utility_search/README.md)を参照してください。オンライン提出ACは主張しません。

## 参考資料

[原理・出典・ライセンス一覧](../utility_search/references.md)。
