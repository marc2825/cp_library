---
title: 重複付き動的MEX
documentation_of: //basic_utility/dynamic_mex.hpp
---

# 重複付き動的MEX（U-03）

[ユーティリティ索引](README.md) / [今回の14項目](../utility_search/README.md) / [入力を変えられる図解](../../visualizations/utility_search/index.html#dynamic_mex)

キーごとの個数を管理し、非負値の個数が0と1の間で変化する時だけ既存のRangeSetを更新します。0を含む被覆区間の右端がMEXです。

## 使用例

```cpp
#include <iostream>
#include "basic_utility/dynamic_mex.hpp"
int main() {
    cp::DynamicMex values;
    values.insert(0); values.insert(1); values.insert(1);
    values.erase(1);
    std::cout << values.mex() << ' ' << values.count(1) << '\n';
    values.insert(3); values.erase(0);
    std::cout << values.mex() << ' ' << values.size() << '\n';
}
```

出力:

```text
2 1
0 2
```

## API・使用条件

`DynamicMex` は初期状態で空です。

- `insert(int64_t x)` は1個追加、`erase(x)` は1個削除。存在しなければ `false`、削除できれば `true`。
- `count(x)` は出現数、`size()` は重複込みの要素数。`clear()` は全削除。
- `mex()` は非負整数の最小欠損値をuint64で返します。負数は保存・削除できますがMEXには影響しません。
- `INT64_MIN`、`INT64_MAX` も入力可能。非負値の被覆区間をuint64で扱うので `INT64_MAX+1` の右端も表現できます。要素数がuint64を超える挿入は `overflow_error`。

事前の値上限・座標圧縮は不要です。静的MEXは既存の [grundy_mex](../../game_theory/grundy.hpp) を使えます。

## 計算量

異なるキー数Dに対し挿入・削除・count・mexは O(log(D+1))、記憶O(D)。sizeはO(1)、clearはO(D)。

## 使用する問題・検証

配列の点更新後のMEXでは、旧値を1個削除し新値を1個挿入します。比較テストは頻度表と0からの走査、境界値、欠損の分割・再結合を使用します。

[共通の比較テスト](../../verify/utility_search/unit_test.cpp)で空入力・端の値・重複・不正入力と独立した素朴解を確認します。`python3 verify/run_utility_search.py`。実行した構成と図解の制約は[検証記録](../utility_search/README.md)を参照してください。オンライン提出ACは主張しません。

## 参考資料

[原理・出典・ライセンス一覧](../utility_search/references.md)。
