---
documentation_of: //string_algorithm/fm_index.hpp
---

# FM-index（S-04）

[カテゴリ](README.md) / [S全11項目](../string_extensions/README.md) / [実装](../../string_algorithm/fm_index.hpp) / [操作できる図解](../../visualizations/string_extensions/index.html#fm_index)

## API・使用条件

`FMIndex(text={},sample_rate=32)`、`interval(pattern)`、`count(pattern)`、`locate(pattern)`、size/sample_rate。

終端付きBWTの半開SA区間を返す。locateは重複を含む開始位置の昇順。空patternは全n+1境界。sample_rateは正。構築後は元テキスト・完全なSAを保持しない。9段のpacked rankを使用し、entropy圧縮やrun-length FM-indexは実装しない。

C++17、namespace `cp`。入力はbyte列、添字は0始まり・半開区間です。UTF-8の文字単位には変換しません。0を含む入力は長さ付きstring/string_viewで渡してください。入力長はintの範囲内、BWT/LCEはINT_MAX未満、SAMは状態数を含めてintに収まる範囲です。形状・法・予算等の違反はinvalid_argument/out_of_range/length_errorで拒否します。各ヘッダとstring_algorithm/all.hppから利用できます。

## 原理・計算量

patternを後ろから読み、C[c]+rank_cで候補区間を狭める。位置はLFを辿ってテキスト座標で間引いたSA標本へ戻る。

構築O(n log n)、count/intervalはO(9|pattern|)。k件のlocateはO(9|pattern|+9k·min(sample_rate,n+1)+k log k)。保持領域はO(n log257+(n/sample_rate+1)log n) bitと固定アルファベット表。構築時はO(n) word。

## 使用例

[コンパイル可能なソース](../../examples/string_algorithm/fm_index.cpp)。

```cpp
#include "string_algorithm/fm_index.hpp"
#include <iostream>

template <class T> void print(const std::vector<T> &a) {
    for (std::size_t i = 0; i < a.size(); ++i)
        std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}

int main() {
    cp::FMIndex index("banana", 3);
    std::cout << index.count("ana") << '\n';
    print(index.locate("ana"));
}
```

出力:

```text
2
1 3
```

## 使用する問題・検証

静的な全文検索、出現数と全出現位置の取得。

[比較検証一覧](../string_extensions/README.md#検証)。独立した全探索・動的計画法、通常/debug mode/sanitizerのDebug/NDEBUG、全カテゴリ混在・2翻訳単位・使用例を確認します。オンラインACは未確認です。

[一次資料・設計判断](../string_extensions/references.md) / [図解の対応範囲](../../visualizations/string_extensions/README.md)。
