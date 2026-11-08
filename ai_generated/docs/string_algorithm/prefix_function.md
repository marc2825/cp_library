---
title: Prefix Function
documentation_of: //string_algorithm/prefix_function.hpp
---

# Prefix Function

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/string_algorithm/index.html#prefix) / [実装](../../string_algorithm/prefix_function.hpp) / [一次資料](references.md)

## usage

```cpp
#include <iostream>
#include "string_algorithm/prefix_function.hpp"
int main() {
    for (int x : cp::prefix_function("ababaca")) std::cout << x << ' ';
    std::cout << '\n';
}
```

[動く使用例](../../examples/string_algorithm/prefix_function.cpp)。リポジトリのルートをincludeパスに追加します。C++17・namespace cp。[共通仕様](README.md#共通仕様)も参照してください。

## API・使用条件

`prefix_function(string_view s) -> vector<int>`、`prefix_function(const vector<T>& s)`。

prefix[i]はs[0..i]の接頭辞かつ接尾辞となる最長の**真の**borderの長さです。文字列自身の長さi+1は含めません。prefix[0]=0、空入力は空です。vector版は==だけを要求します。

全体のborderを列挙するには、非空ならk=prefix.back()からk=prefix[k−1]へ進み、0で終了します。

## 原理・計算量

前の長さj=prefix[i−1]から延長を試み、不一致ならj=prefix[j−1]へ戻します。戻る先は今のborderが持つborderであり、他の候補を個別に試す必要がありません。jの増加は1ずつで、減少の合計も線形なのでO(n)時間・O(n)領域です。

## 使用する問題・検証範囲

[CSES Finding Borders](https://cses.fi/problemset/task/1732)の[ラッパ](../../verify/string_algorithm/prefix_function.test.cpp)。全ての接頭辞と接尾辞を直接比較して最大の真のborderと全borderを確認します。

[検証入口](../../verify/run_string_algorithm.py)は10ヘッダ（内部補助・all含む）、8使用例、8ジャッジ形式・512ケースを確認します。9,841三値文字列・180全バイト列、3,937 KMP組、120 Trie履歴・48,000操作、300組のハッシュ・6,000区間問い合わせ、Python独立検証4,126クエリ（うちTrie 2,880操作）を定義式と照合。0/FF・UTF-8のバイト・空・重複・符号付き整数の極値・base境界・入力拒否、20万文字の同一／交互列・深いTrieと比較回数の線形上限を含みます。

通常・ASan/UBSan/LeakSanitizerのdebug/releaseを実行します。図解は8モード、3,096入力・51,386スナップショット、16拒否条件とブラウザ操作を確認します。オンラインACは未確認です。ハッシュに衝突が存在しないことを主張する検証ではありません。
