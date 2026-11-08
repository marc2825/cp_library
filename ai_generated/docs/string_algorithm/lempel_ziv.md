---
documentation_of: //string_algorithm/lempel_ziv.hpp
---

# Lempel–Ziv（LZ77）分解（S-02）

[カテゴリ](README.md) / [S全11項目](../string_extensions/README.md) / [実装](../../string_algorithm/lempel_ziv.hpp) / [操作できる図解](../../visualizations/string_extensions/index.html#lz77)

## API・使用条件

`lempel_ziv_factorization(text) → vector<LZ77Factor>`。`restore_lz77(factors,max_length=100000000)`。

greedyな最長既出因子。sourceは絶対位置、−1ならliteralの1byte。sourceは因子の開始より前で、参照区間の重なりを許す。復元は先頭から1byteずつコピーする。同長のsourceはSA上の前後候補の小さい位置で、全候補中の最左とは限らない。

C++17、namespace `cp`。入力はbyte列、添字は0始まり・半開区間です。UTF-8の文字単位には変換しません。0を含む入力は長さ付きstring/string_viewで渡してください。入力長はintの範囲内、BWT/LCEはINT_MAX未満、SAMは状態数を含めてintに収まる範囲です。形状・法・予算等の違反はinvalid_argument/out_of_range/length_errorで拒否します。各ヘッダとstring_algorithm/all.hppから利用できます。

## 原理・計算量

既出suffixの辞書順の直前・直後だけで最長一致長を求め、greedyに因子へする。

既存SA/LCPのLCE索引と順位集合でO(n log n)時間・領域。復元O(出力長)、明示した出力予算を超えるとlength_error。

## 使用例

[コンパイル可能なソース](../../examples/string_algorithm/lempel_ziv.cpp)。

```cpp
#include "string_algorithm/lempel_ziv.hpp"
#include <iostream>

template <class T> void print(const std::vector<T> &a) {
    for (std::size_t i = 0; i < a.size(); ++i)
        std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}

int main() {
    auto f = cp::lempel_ziv_factorization("aaaaabaaaaab");
    for (auto x : f)
        std::cout << x.source << ' ' << x.length << ' ' << int(x.literal) << '\n';
    std::cout << (cp::restore_lz77(f) == "aaaaabaaaaab") << '\n';
}
```

出力:

```text
-1 1 97
0 4 0
-1 1 98
0 6 0
1
```

## 使用する問題・検証

繰返し文字列の因子化と、自己参照を含むLZ77データの復元。

[比較検証一覧](../string_extensions/README.md#検証)。独立した全探索・動的計画法、通常/debug mode/sanitizerのDebug/NDEBUG、全カテゴリ混在・2翻訳単位・使用例を確認します。オンラインACは未確認です。

[一次資料・設計判断](../string_extensions/references.md) / [図解の対応範囲](../../visualizations/string_extensions/README.md)。
