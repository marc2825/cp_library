---
title: KMP（全出現位置）
documentation_of: //string_algorithm/kmp.hpp
---

# KMP（全出現位置）

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/string_algorithm/index.html#kmp) / [実装](../../string_algorithm/kmp.hpp) / [一次資料](references.md)

## usage

```cpp
#include <iostream>
#include "string_algorithm/kmp.hpp"
int main() {
    for (int pos : cp::kmp_search("abababa", "aba")) std::cout << pos << ' ';
    std::cout << '\n';
    for (int pos : cp::kmp_search("ab", "")) std::cout << pos << ' ';
    std::cout << '\n';
}
```

[動く使用例](../../examples/string_algorithm/kmp.cpp)。リポジトリのルートをincludeパスに追加します。C++17・namespace cp。[共通仕様](README.md#共通仕様)も参照してください。

## API・使用条件

`kmp_search(string_view text,string_view pattern) -> vector<int>`。同じTのvectorを2つ渡す版もあります。

patternの全出現位置を0始まり・昇順で返します。**重複する一致も含みます**。例えばaaaa中のaaは{0,1,2}。空patternは全ての境界0..text.size()に一致し、両方空なら{0}です。区切り文字を使って連結しないため、全てのバイトを入力に使えます。vector版は==だけを要求します。

## 原理・計算量

patternのPrefix Functionを作り、本文位置iと一致済み長jを保持します。不一致では本文を戻さずjをprefix[j−1]へ戻します。全一致した後も末尾のborderを残すため、重なる一致を見落としません。

本文n、pattern m、一致数kとしてO(n+m+k)時間・O(m+k)追加領域です。空patternではk=n+1となります。

## 使用する問題・検証範囲

[AOJ ALDS1_14_B String Search](https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=ALDS1_14_B)の[ラッパ](../../verify/string_algorithm/kmp.test.cpp)。各本文位置でpatternを直接比較し、空・長さ超過・一致なし・重複を検証します。

[検証入口](../../verify/run_string_algorithm.py)は10ヘッダ（内部補助・all含む）、8使用例、8ジャッジ形式・512ケースを確認します。9,841三値文字列・180全バイト列、3,937 KMP組、120 Trie履歴・48,000操作、300組のハッシュ・6,000区間問い合わせ、Python独立検証4,126クエリ（うちTrie 2,880操作）を定義式と照合。0/FF・UTF-8のバイト・空・重複・符号付き整数の極値・base境界・入力拒否、20万文字の同一／交互列・深いTrieと比較回数の線形上限を含みます。

通常・ASan/UBSan/LeakSanitizerのdebug/releaseを実行します。図解は8モード、3,096入力・51,386スナップショット、16拒否条件とブラウザ操作を確認します。オンラインACは未確認です。ハッシュに衝突が存在しないことを主張する検証ではありません。
