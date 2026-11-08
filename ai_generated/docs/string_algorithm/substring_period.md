---
documentation_of: //string_algorithm/substring_period.hpp
---

# 区間の周期・反復回数（S-23）

[カテゴリ](README.md) / [S全11項目](../string_extensions/README.md) / [実装](../../string_algorithm/substring_period.hpp) / [操作できる図解](../../visualizations/string_extensions/index.html#period_queries)

## API・使用条件

`SubstringPeriodIndex(text={})`、is_period(l,r,p)、minimum_repetition_period(l,r)、repetition_count(l,r)。

固定文字列の半開区間。period判定は1≤p≤長さで、長さを割り切らなくてもよい。最小反復周期は長さを割り切るperiodだけから選び、反復回数は長さ/周期。空区間の最小反復周期・回数は0。

C++17、namespace `cp`。入力はbyte列、添字は0始まり・半開区間です。UTF-8の文字単位には変換しません。0を含む入力は長さ付きstring/string_viewで渡してください。入力長はintの範囲内、BWT/LCEはINT_MAX未満、SAMは状態数を含めてintに収まる範囲です。形状・法・予算等の違反はinvalid_argument/out_of_range/length_errorで拒否します。各ヘッダとstring_algorithm/all.hppから利用できます。

## 原理・計算量

SA/LCPの厳密LCEでずらした2区間を比較する。区間長を素因数分解し、周期を各素因数で割っても一致する間だけ縮める。

構築O(n log n)時間・領域。period判定O(1)、最小反復周期・回数O(log n)。

## 使用例

[コンパイル可能なソース](../../examples/string_algorithm/substring_period.cpp)。

```cpp
#include "string_algorithm/substring_period.hpp"
#include <iostream>

template <class T> void print(const std::vector<T> &a) {
    for (std::size_t i = 0; i < a.size(); ++i)
        std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}

int main() {
    cp::SubstringPeriodIndex index("zzabababx");
    std::cout << index.is_period(2, 8, 4) << ' ' << index.minimum_repetition_period(2, 8) << ' '
              << index.repetition_count(2, 8) << '\n';
}
```

出力:

```text
1 2 3
```

## 使用する問題・検証

多数の区間の繰返し判定・primitive root・反復回数。任意periodの全列挙はS-06。

[比較検証一覧](../string_extensions/README.md#検証)。独立した全探索・動的計画法、通常/debug mode/sanitizerのDebug/NDEBUG、全カテゴリ混在・2翻訳単位・使用例を確認します。オンラインACは未確認です。

[一次資料・設計判断](../string_extensions/references.md) / [図解の対応範囲](../../visualizations/string_extensions/README.md)。
