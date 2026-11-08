---
documentation_of: //string_algorithm/suffix_automaton_utilities.hpp
---

# SAMのk番目・全出現位置（S-07）

[カテゴリ](README.md) / [S全11項目](../string_extensions/README.md) / [実装](../../string_algorithm/suffix_automaton_utilities.hpp) / [操作できる図解](../../visualizations/string_extensions/index.html#sam_utilities)

## API・使用条件

`SuffixAutomatonUtilities(text={})`、kth_substring(k)、occurrences(pattern)、distinct_substrings()、automaton()。

既存SuffixAutomatonに添える静的索引。kは0始まり、空部分文字列を除きunsigned-byte辞書順。範囲外はnullopt。出現位置は昇順、空patternは全境界。既存のオンラインSAMは変更しない。

C++17、namespace `cp`。入力はbyte列、添字は0始まり・半開区間です。UTF-8の文字単位には変換しません。0を含む入力は長さ付きstring/string_viewで渡してください。入力長はintの範囲内、BWT/LCEはINT_MAX未満、SAMは状態数を含めてintに収まる範囲です。形状・法・予算等の違反はinvalid_argument/out_of_range/length_errorで拒否します。各ヘッダとstring_algorithm/all.hppから利用できます。

## 原理・計算量

遷移DAGの先の経路数で辞書順の枝を飛ばす。suffix link木のEuler区間に含まれる実prefixの終点を取り出す。

構築O(n log n)時間・O(n)領域。k番目はO(256·出力長)。全出現はO(|p|log256+log n+k log k)。

## 使用例

[コンパイル可能なソース](../../examples/string_algorithm/suffix_automaton_utilities.cpp)。

```cpp
#include "string_algorithm/suffix_automaton_utilities.hpp"
#include <iostream>

template <class T> void print(const std::vector<T> &a) {
    for (std::size_t i = 0; i < a.size(); ++i)
        std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}

int main() {
    cp::SuffixAutomatonUtilities index("banana");
    std::cout << *index.kth_substring(2) << '\n';
    print(index.occurrences("ana"));
}
```

出力:

```text
ana
1 3
```

## 使用する問題・検証

[Number of Substrings](https://judge.yosupo.jp/problem/number_of_substrings)、辞書順の部分文字列と重複出現の列挙。

[比較検証一覧](../string_extensions/README.md#検証)。独立した全探索・動的計画法、通常/debug mode/sanitizerのDebug/NDEBUG、全カテゴリ混在・2翻訳単位・使用例を確認します。オンラインACは未確認です。

[一次資料・設計判断](../string_extensions/references.md) / [図解の対応範囲](../../visualizations/string_extensions/README.md)。
