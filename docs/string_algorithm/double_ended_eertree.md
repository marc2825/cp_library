---
documentation_of: //string_algorithm/double_ended_eertree.hpp
---

# 両端回文木（S-20）

[カテゴリ](README.md) / [S全11項目](../string_extensions/README.md) / [実装](../../string_algorithm/double_ended_eertree.hpp) / [操作できる図解](../../visualizations/string_extensions/index.html#deque_eertree)

## API・使用条件

`DoubleEndedEertree(text={})`、push_front/back(byte)、pop_front/back()、distinct_count()、total_occurrences()、longest_prefix/suffix()、size/empty/text、allocated_nodes/clear。

空回文は数えない。popは削除したbyteを返し、空dequeではout_of_range。現在文字列の相異なる回文数・出現総数・最長prefix/suffix長を返す。過去の回文nodeは再利用のためclearまで保持し、allocated_nodesは履歴を含む。

C++17、namespace `cp`。入力はbyte列、添字は0始まり・半開区間です。UTF-8の文字単位には変換しません。0を含む入力は長さ付きstring/string_viewで渡してください。入力長はintの範囲内、BWT/LCEはINT_MAX未満、SAMは状態数を含めてintに収まる範囲です。形状・法・予算等の違反はinvalid_argument/out_of_range/length_errorで拒否します。各ヘッダとstring_algorithm/all.hppから利用できます。

## 原理・計算量

最大回文に覆われない出現の端（surface）を記録する。最長回文のsuffix linkと間接出現数で追加・削除を更新し、persistentなbyte辞書でsuffix探索を短縮する。

更新は償却O(log256)、個数・最長prefix/suffixはO(1)。nを現在長、Pを履歴中の相異なる回文数としてO(n+P log256) word。text()はO(n)。vector再確保を含むので最悪O(1)更新とはしない。

## 使用例

[コンパイル可能なソース](../../examples/string_algorithm/double_ended_eertree.cpp)。

```cpp
#include "string_algorithm/double_ended_eertree.hpp"
#include <iostream>

template <class T> void print(const std::vector<T> &a) {
    for (std::size_t i = 0; i < a.size(); ++i)
        std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}

int main() {
    cp::DoubleEndedEertree e;
    e.push_back('a');
    e.push_back('b');
    e.push_front('a');
    e.push_back('a');
    e.pop_front();
    e.push_front('b');
    std::cout << e.distinct_count() << ' ' << e.total_occurrences() << ' ' << e.longest_prefix() << ' '
              << e.longest_suffix() << '\n';
}
```

出力:

```text
4 6 3 3
```

## 使用する問題・検証

両端追加削除・sliding window中の回文数と最長の回文prefix/suffix。

[比較検証一覧](../string_extensions/README.md#検証)。独立した全探索・動的計画法、通常/debug mode/sanitizerのDebug/NDEBUG、全カテゴリ混在・2翻訳単位・使用例を確認します。オンラインACは未確認です。

[一次資料・設計判断](../string_extensions/references.md) / [図解の対応範囲](../../visualizations/string_extensions/README.md)。
