---
title: Manacher（奇数・偶数の回文半径）
documentation_of: //string_algorithm/manacher.hpp
---

# Manacher（奇数・偶数の回文半径）

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/string_algorithm/index.html#manacher) / [実装](../../string_algorithm/manacher.hpp) / [一次資料](references.md)

## usage

```cpp
#include <iostream>
#include "string_algorithm/manacher.hpp"
int main() {
    auto radii = cp::manacher("abbaaba");
    for (int x : radii.odd) std::cout << x << ' ';
    std::cout << '\n';
    for (int x : radii.even) std::cout << x << ' ';
    std::cout << '\n';
    std::cout << radii.is_palindrome(0, 4) << ' ' << radii.is_palindrome(0, 7) << '\n';
}
```

[動く使用例](../../examples/string_algorithm/manacher.cpp)。リポジトリのルートをincludeパスに追加します。C++17・namespace cp。[共通仕様](README.md#共通仕様)も参照してください。

## API・使用条件

`manacher(string_view s) -> PalindromicRadii`、`manacher(const vector<T>& s)`。

結果のodd/evenは共に長さnのvector<int>です。odd[i]は中心i自身を含む半径で、最大の奇数長回文は[i−odd[i]+1,i+odd[i])、長さ2*odd[i]−1です。even[i]は**iの直前のすき間**を中心とする半径で、最大の偶数長回文は[i−even[i],i+even[i])、長さ2*even[i]です。even[0]=0。空入力は両配列とも空。

`result.is_palindrome(l,r)`は[l,r)が回文かをO(1)で返します。空区間はtrue、区間違反はout_of_range。結果の2配列を変更せずに使用してください。vector版は==だけを使います。特殊な番兵バイトを必要としません。

## 原理・計算量

最も右まで届く回文の境界を保持し、その内側では対称な中心の半径を、現在の右端までコピーします。既知の範囲を越える部分だけを比較して広げます。奇数・偶数の各走査がO(n)、全体O(n)時間・O(n)領域です。

全回文の個数はΣodd+Σevenで求められます。半径をコンパクトに保持するアルゴリズムで、O(n²)個あり得る全区間を実際に列挙する費用は含みません。

## 使用する問題・検証範囲

[Library Checker Enumerate Palindromes](https://judge.yosupo.jp/problem/enumerate_palindromes)の[ラッパ](../../verify/string_algorithm/manacher.test.cpp)。奇数の中心と偶数のすき間を交互にした最大長へ変換します。全ての中心の直接拡張と、全区間を反転した回文判定を比較します。

[検証入口](../../verify/run_string_algorithm.py)は10ヘッダ（内部補助・all含む）、8使用例、8ジャッジ形式・512ケースを確認します。9,841三値文字列・180全バイト列、3,937 KMP組、120 Trie履歴・48,000操作、300組のハッシュ・6,000区間問い合わせ、Python独立検証4,126クエリ（うちTrie 2,880操作）を定義式と照合。0/FF・UTF-8のバイト・空・重複・符号付き整数の極値・base境界・入力拒否、20万文字の同一／交互列・深いTrieと比較回数の線形上限を含みます。

通常・ASan/UBSan/LeakSanitizerのdebug/releaseを実行します。図解は8モード、3,096入力・51,386スナップショット、16拒否条件とブラウザ操作を確認します。オンラインACは未確認です。ハッシュに衝突が存在しないことを主張する検証ではありません。
