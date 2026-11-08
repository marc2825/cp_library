---
documentation_of: //string_algorithm/border_tree.hpp
---

# Border Tree・全周期列挙（S-06）

[カテゴリ](README.md) / [S全11項目](../string_extensions/README.md) / [実装](../../string_algorithm/border_tree.hpp) / [操作できる図解](../../visualizations/string_extensions/index.html#borders)

## API・使用条件

`BorderTree(text={})`、append(byte/string_view)、parent(length)、is_border(border,prefix)、borders(prefix,include_self=false,include_empty=false)、periods(prefix)、prefix_occurrences()。`string_periods(text)`。

頂点kは長さkのprefix。親は最長の真のborder。bordersとperiodsは昇順。periodは長さを割り切る必要がない。空prefixのperiodsは空。空prefixの出現数はn+1。追記後にも全APIを利用できる。

C++17、namespace `cp`。入力はbyte列、添字は0始まり・半開区間です。UTF-8の文字単位には変換しません。0を含む入力は長さ付きstring/string_viewで渡してください。入力長はintの範囲内、BWT/LCEはINT_MAX未満、SAMは状態数を含めてintに収まる範囲です。形状・法・予算等の違反はinvalid_argument/out_of_range/length_errorで拒否します。各ヘッダとstring_algorithm/all.hppから利用できます。

## 原理・計算量

prefix functionの失敗リンクを木にする。全border bとperiod n−bの対応を使い、子から親へ出現数を集計する。

構築・追記累積O(n log n)、領域O(n log n)。is_border O(log n)、列挙O(出力数)、prefix_occurrences O(n)。実装はint幅の31段ancestor表。

## 使用例

[コンパイル可能なソース](../../examples/string_algorithm/border_tree.cpp)。

```cpp
#include "string_algorithm/border_tree.hpp"
#include <iostream>

template <class T> void print(const std::vector<T> &a) {
    for (std::size_t i = 0; i < a.size(); ++i)
        std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}

int main() {
    cp::BorderTree tree("ababab");
    print(tree.borders(6));
    print(tree.periods(6));
    std::cout << tree.prefix_occurrences()[2] << '\n';
}
```

出力:

```text
2 4
2 4 6
3
```

## 使用する問題・検証

共通する接頭辞・接尾辞の列挙、全period、prefixの出現回数。

[比較検証一覧](../string_extensions/README.md#検証)。独立した全探索・動的計画法、通常/debug mode/sanitizerのDebug/NDEBUG、全カテゴリ混在・2翻訳単位・使用例を確認します。オンラインACは未確認です。

[一次資料・設計判断](../string_extensions/references.md) / [図解の対応範囲](../../visualizations/string_extensions/README.md)。
