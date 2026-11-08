---
title: LCP Array
documentation_of: //string_algorithm/lcp_array.hpp
---

# LCP Array

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/string_algorithm/index.html#lcp) / [実装](../../string_algorithm/lcp_array.hpp) / [一次資料](references.md)

## usage

```cpp
#include <iostream>
#include <string>
#include "string_algorithm/suffix_array.hpp"
#include "string_algorithm/lcp_array.hpp"
int main() {
    std::string s = "banana";
    auto sa = cp::suffix_array(s);
    for (int x : cp::lcp_array(s, sa)) std::cout << x << ' ';
    std::cout << '\n';
}
```

[動く使用例](../../examples/string_algorithm/lcp_array.cpp)。リポジトリのルートをincludeパスに追加します。C++17・namespace cp。[共通仕様](README.md#共通仕様)も参照してください。

## API・使用条件

`lcp_array(string_view s,const vector<int>& sa) -> vector<int>`、同じTのvector版もあります。

長さmax(0,n−1)で、結果[i]はs[sa[i]..n)とs[sa[i+1]..n)の最長共通接頭辞長です。**saは正しく辞書順に並んだSuffix Arrayであることが前提**です。

saの長さ、要素の範囲、重複を検査し、違反はinvalid_argument。辞書順の正しさは再計算しません。単なる別の順列を渡した場合の結果・計算量は保証しません。空入力・saとも空なら空、1要素でも空を返します。vector版は==が必要で、Suffix Arrayの順序と同値判定が一致する型を使います。

## 原理・計算量

Kasaiの方法で本文位置iを昇順に走査します。逆順位から辞書順の隣のsuffixを求め、前の位置で確定した長さから1を引いた下限を再利用します。比較で長さを増やす合計は線形です。

正しいsaに対してO(n)時間・O(n)追加領域。Suffix Arrayを作る費用は別です。異なる部分文字列の個数はn(n+1)/2−ΣLCPで求まり、大きいnでは個数を64ビット型にします。任意の2つのsuffixの厳密なLCPが必要なら、この隣接LCPの区間最小値を取れます。

## 使用する問題・検証範囲

[Library Checker Number of Substrings](https://judge.yosupo.jp/problem/number_of_substrings)の[ラッパ](../../verify/string_algorithm/lcp_array.test.cpp)。隣接suffixの直接比較と、全ての部分文字列を集合に入れた個数を独立に確認します。

[検証入口](../../verify/run_string_algorithm.py)は10ヘッダ（内部補助・all含む）、8使用例、8ジャッジ形式・512ケースを確認します。9,841三値文字列・180全バイト列、3,937 KMP組、120 Trie履歴・48,000操作、300組のハッシュ・6,000区間問い合わせ、Python独立検証4,126クエリ（うちTrie 2,880操作）を定義式と照合。0/FF・UTF-8のバイト・空・重複・符号付き整数の極値・base境界・入力拒否、20万文字の同一／交互列・深いTrieと比較回数の線形上限を含みます。

通常・ASan/UBSan/LeakSanitizerのdebug/releaseを実行します。図解は8モード、3,096入力・51,386スナップショット、16拒否条件とブラウザ操作を確認します。オンラインACは未確認です。ハッシュに衝突が存在しないことを主張する検証ではありません。
