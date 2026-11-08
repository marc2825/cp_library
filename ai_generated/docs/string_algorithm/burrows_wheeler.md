---
documentation_of: //string_algorithm/burrows_wheeler.hpp
---

# BWT・逆BWT（S-03）

[カテゴリ](README.md) / [S全11項目](../string_extensions/README.md) / [実装](../../string_algorithm/burrows_wheeler.hpp) / [操作できる図解](../../visualizations/string_extensions/index.html#bwt)

## API・使用条件

`burrows_wheeler_transform(text) → BurrowsWheelerTransform{last,primary}`。`inverse_burrows_wheeler(bwt) → string`。

byteを1〜256に写し、仮想終端0を追加する。lastはuint16_tのn+1要素、primaryは元文字列の行。空文字列もlast={0},primary=0。0/FFを含む全byteに対応する。逆変換は終端数・primary・LFの単一巡回を検査する。

C++17、namespace `cp`。入力はbyte列、添字は0始まり・半開区間です。UTF-8の文字単位には変換しません。0を含む入力は長さ付きstring/string_viewで渡してください。入力長はintの範囲内、BWT/LCEはINT_MAX未満、SAMは状態数を含めてintに収まる範囲です。形状・法・予算等の違反はinvalid_argument/out_of_range/length_errorで拒否します。各ヘッダとstring_algorithm/all.hppから利用できます。

## 原理・計算量

終端付きsuffixを並べ、直前の文字を取り出す。逆変換では同じ文字の出現順位を保存するLF対応を辿る。

変換O(n log n)時間・O(n)領域、逆変換O(n+257)時間・O(n)領域。

## 使用例

[コンパイル可能なソース](../../examples/string_algorithm/burrows_wheeler.cpp)。

```cpp
#include "string_algorithm/burrows_wheeler.hpp"
#include <iostream>

template <class T> void print(const std::vector<T> &a) {
    for (std::size_t i = 0; i < a.size(); ++i)
        std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}

int main() {
    auto b = cp::burrows_wheeler_transform("banana");
    std::cout << b.primary << '\n';
    print(b.last);
    std::cout << (cp::inverse_burrows_wheeler(b) == "banana") << '\n';
}
```

出力:

```text
4
98 111 111 99 0 98 98
1
```

## 使用する問題・検証

可逆な文字列変換、圧縮や全文索引の前処理。

[比較検証一覧](../string_extensions/README.md#検証)。独立した全探索・動的計画法、通常/debug mode/sanitizerのDebug/NDEBUG、全カテゴリ混在・2翻訳単位・使用例を確認します。オンラインACは未確認です。

[一次資料・設計判断](../string_extensions/references.md) / [図解の対応範囲](../../visualizations/string_extensions/README.md)。
