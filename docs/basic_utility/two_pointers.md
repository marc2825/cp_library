---
title: 尺取り法
documentation_of: //basic_utility/two_pointers.hpp
---

# 尺取り法（U-01）

[ユーティリティ索引](README.md) / [今回の14項目](../utility_search/README.md) / [入力を変えられる図解](../../visualizations/utility_search/index.html#two_pointers)

右端を単調に進め、条件を満たす限り区間を拡張します。左端を除いた区間も有効なので、右端を後退させる必要がありません。右端配列は二次個になり得る区間集合の圧縮表現です。

## 使用例

```cpp
#include <iostream>
#include "basic_utility/two_pointers.hpp"
int main() {
    std::vector<int> a{2, 1, 3, 1, 2};
    int sum = 0;
    auto windows = cp::two_pointers(a.size(),
        [&](std::size_t i) { return sum + a[i] <= 4; },
        [&](std::size_t i) { sum += a[i]; },
        [&](std::size_t i) { sum -= a[i]; });
    auto [l, r] = windows.longest();
    std::cout << l << ' ' << r << ' ' << windows.count() << '\n';
}
```

出力:

```text
0 2 9
```

## API・使用条件

`two_pointers(n, can_add, add, remove)` は左端ごとの最大右端 `right[l]` を返します。全て0始まり、半開区間です。

- 状態は空区間で開始します。`can_add(r)` は現在の区間に添字 `r` を加えてよいかを調べ、状態を変更しません。`add(r)` と `remove(l)` が更新します。正常終了時も状態は空です。
- 空区間は常に有効で、有効な区間の全ての部分区間も有効であることが必要です。非負列の和の上限制約、重複なしの制約など。負数を含む列の和には一般に使えません。callbackは例外を投げないものとし、途中例外時の状態復元は行いません。
- `longest()` は最長の `[l,r)`。同長なら最小の左端、有効な非空区間がなければ `[0,0)`。
- `count()` は非空区間数。uint64を超えれば `overflow_error`。
- `for_each(visit)` は `(l,r)` の辞書順で全有効非空区間を渡します。callbackの `false` で即停止し `false` を返し、完走は `true`。
- `right` は結果の読み取り用です。手動変更後の整合性は検査しません。

## 計算量

構築 O(n) callback呼出し・O(n)記憶。各要素を最大1回追加・削除、条件判定は最大2n回。最長・個数は O(n)、列挙は出力K件に対し O(n+K)。callback自体の費用は別途加算します。

## 使用する問題・検証

非負列の「和が上限以下」の区間数や最長区間、全要素が異なる最長区間。テストでは全区間走査と比較し、追加・削除の対応とcallback回数も数えます。

[共通の比較テスト](../../verify/utility_search/unit_test.cpp)で空入力・端の値・重複・不正入力と独立した素朴解を確認します。`python3 verify/run_utility_search.py`。実行した構成と図解の制約は[検証記録](../utility_search/README.md)を参照してください。オンライン提出ACは主張しません。

## 参考資料

[原理・出典・ライセンス一覧](../utility_search/references.md)。
