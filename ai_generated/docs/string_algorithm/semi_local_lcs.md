---
documentation_of: //string_algorithm/semi_local_lcs.hpp
---

# Prefix-Substring / Semi-local LCS（S-05）

[カテゴリ](README.md) / [S全11項目](../string_extensions/README.md) / [実装](../../string_algorithm/semi_local_lcs.hpp) / [操作できる図解](../../visualizations/string_extensions/index.html#semi_local)

## API・使用条件

`SemiLocalLCS(a,b)`：string_substring(l,r)、substring_string(l,r)、prefix_suffix(prefix,start)、suffix_prefix(start,prefix)、seaweed()。`PrefixSubstringLCS(a,b,max_cells=4000000).query(prefix,l,r)`。

通常LCSの長さを返す。引数順はA対Bで、空区間も可。SemiLocalは4種類の半局所比較を1つの置換で保持する。PrefixSubstringはAの各prefixについてBの全substringを扱い、(m+1)nセル（空Bではm+1）の予算を検査する。

C++17、namespace `cp`。入力はbyte列、添字は0始まり・半開区間です。UTF-8の文字単位には変換しません。0を含む入力は長さ付きstring/string_viewで渡してください。入力長はintの範囲内、BWT/LCEはINT_MAX未満、SAMは状態数を含めてintに収まる範囲です。形状・法・予算等の違反はinvalid_argument/out_of_range/length_errorで拒否します。各ヘッダとstring_algorithm/all.hppから利用できます。

## 原理・計算量

一致セルまたは左strandの開始labelが上より大きいセルでstrandを交換する。終点順の開始labelを範囲計数し、区間長から横切るstrand数を引く。

SemiLocal構築O(mn+(m+n)log(m+n))、領域O((m+n)log(m+n)) bit＋O(m+n) word、query O(log(m+n))。PrefixSubstring構築O(m+(m+1)n log(n+1))、領域O(m+(m+1)n log(n+1)) wordの上界、query O(log(n+1))。

## 使用例

[コンパイル可能なソース](../../examples/string_algorithm/semi_local_lcs.cpp)。

```cpp
#include "string_algorithm/semi_local_lcs.hpp"
#include <iostream>

template <class T> void print(const std::vector<T> &a) {
    for (std::size_t i = 0; i < a.size(); ++i)
        std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}

int main() {
    cp::SemiLocalLCS s("abac", "baca");
    cp::PrefixSubstringLCS p("abac", "baca");
    std::cout << p.query(3, 1, 4) << ' ' << s.string_substring(1, 4) << ' ' << s.substring_string(1, 4) << ' '
              << s.prefix_suffix(3, 1) << ' ' << s.suffix_prefix(1, 3) << '\n';
}
```

出力:

```text
2 2 3 2 3
```

## 使用する問題・検証

部分列の区間比較、prefix対substringの大量問い合わせ。Longest Common Substringとは異なる。

[比較検証一覧](../string_extensions/README.md#検証)。独立した全探索・動的計画法、通常/debug mode/sanitizerのDebug/NDEBUG、全カテゴリ混在・2翻訳単位・使用例を確認します。オンラインACは未確認です。

[一次資料・設計判断](../string_extensions/references.md) / [図解の対応範囲](../../visualizations/string_extensions/README.md)。
