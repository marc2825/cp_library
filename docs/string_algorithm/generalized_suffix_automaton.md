---
documentation_of: //string_algorithm/generalized_suffix_automaton.hpp
---

# Generalized Suffix Automaton（S-22）

[カテゴリ](README.md) / [S全11項目](../string_extensions/README.md) / [実装](../../string_algorithm/generalized_suffix_automaton.hpp) / [操作できる図解](../../visualizations/string_extensions/index.html#generalized_sam)

## API・使用条件

`GeneralizedSuffixAutomaton(documents={})`、add_document、contains/count_occurrences/occurrences/containing_documents、distinct_substrings/kth_substring、document_count/state_count。

文書を追加可能。重複文書は別ID。出現は(document,position)昇順。空patternは各文書の全境界、文書0件なら0。内部の区切り256はbyte queryから見えず、文書をまたぐ部分文字列は数えない。union言語の最小DFAを保証するAPIではない。

C++17、namespace `cp`。入力はbyte列、添字は0始まり・半開区間です。UTF-8の文字単位には変換しません。0を含む入力は長さ付きstring/string_viewで渡してください。入力長はintの範囲内、BWT/LCEはINT_MAX未満、SAMは状態数を含めてintに収まる範囲です。形状・法・予算等の違反はinvalid_argument/out_of_range/length_errorで拒否します。各ヘッダとstring_algorithm/all.hppから利用できます。

## 原理・計算量

私有separatorで文書を区切ったSAMを作り、byte遷移だけのDAGとsuffix link木の索引を共有する。出現位置を文書座標に戻す。

N=総byte長＋文書数。構築O(N log257)、O(N)領域。追記後の最初の索引queryはO(N log N)でcache再構築。countはO(|p|log257+log N)、位置・文書列挙はO(|p|log257+log N+k log k)、k番目O(256·出力長)。

## 使用例

[コンパイル可能なソース](../../examples/string_algorithm/generalized_suffix_automaton.cpp)。

```cpp
#include "string_algorithm/generalized_suffix_automaton.hpp"
#include <iostream>

template <class T> void print(const std::vector<T> &a) {
    for (std::size_t i = 0; i < a.size(); ++i)
        std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}

int main() {
    cp::GeneralizedSuffixAutomaton index(std::vector<std::string>{"banana", "band", "ana"});
    std::cout << index.count_occurrences("ana") << ' ' << index.distinct_substrings() << '\n';
    print(index.containing_documents("ana"));
}
```

出力:

```text
3 19
0 2
```

## 使用する問題・検証

複数文書に含まれる部分文字列の検索、文書別の全出現、unionの相異なる部分文字列数。

[比較検証一覧](../string_extensions/README.md#検証)。独立した全探索・動的計画法、通常/debug mode/sanitizerのDebug/NDEBUG、全カテゴリ混在・2翻訳単位・使用例を確認します。オンラインACは未確認です。

[一次資料・設計判断](../string_extensions/references.md) / [図解の対応範囲](../../visualizations/string_extensions/README.md)。
