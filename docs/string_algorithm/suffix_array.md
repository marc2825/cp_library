---
title: Suffix Array
documentation_of: //string_algorithm/suffix_array.hpp
---

# Suffix Array

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/string_algorithm/index.html#suffix) / [実装](../../string_algorithm/suffix_array.hpp) / [一次資料](references.md)

## usage

```cpp
#include <iostream>
#include "string_algorithm/suffix_array.hpp"
int main() {
    for (int pos : cp::suffix_array("banana")) std::cout << pos << ' ';
    std::cout << '\n';
}
```

[動く使用例](../../examples/string_algorithm/suffix_array.cpp)。リポジトリのルートをincludeパスに追加します。C++17・namespace cp。[共通仕様](README.md#共通仕様)も参照してください。

## API・使用条件

`suffix_array(string_view s) -> vector<int>`、`suffix_array(const vector<T>& s)`。

sの**空でないsuffix**の始点0..n−1を辞書順に並べます。空suffixの位置nは含みません。空入力は空、1要素は{0}です。string版はunsigned byteの0..255の順序を使い、charが符号付きの環境でも同じ順になります。

vector版はTの<による狭義弱順序を使い、要素の座標圧縮を行います。LLONG_MIN/MAXなどでも差を取らずに比較します。LCP等と組み合わせる場合は、<で同値となる要素と==が一致する型を使ってください。

## 原理・計算量

1文字の順位から始め、(rank[i],rank[i+width])の組でsuffixを安定に並べ直して順位を圧縮します。文字列の外の後半は全ての値より小さい番兵順位−1で、入力に番兵バイトは追加しません。C++では後半→前半の計数ソートを使い、各段階O(n+順位数)。widthは1,2,4,…と倍にします。

時間O(n log(n+1))、領域O(n)。vector版は最初の座標圧縮にもO(n log(n+1))の比較を使います。比較・要素操作の費用は別。SA-ISによる線形時間版ではありません。

## 使用する問題・検証範囲

[Library Checker Suffix Array](https://judge.yosupo.jp/problem/suffixarray)の[ラッパ](../../verify/string_algorithm/suffix_array.test.cpp)。suffixそのものを直接辞書順に並べ、完全な順列と比較します。

[検証入口](../../verify/run_string_algorithm.py)は10ヘッダ（内部補助・all含む）、8使用例、8ジャッジ形式・512ケースを確認します。9,841三値文字列・180全バイト列、3,937 KMP組、120 Trie履歴・48,000操作、300組のハッシュ・6,000区間問い合わせ、Python独立検証4,126クエリ（うちTrie 2,880操作）を定義式と照合。0/FF・UTF-8のバイト・空・重複・符号付き整数の極値・base境界・入力拒否、20万文字の同一／交互列・深いTrieと比較回数の線形上限を含みます。

通常・ASan/UBSan/LeakSanitizerのdebug/releaseを実行します。図解は8モード、3,096入力・51,386スナップショット、16拒否条件とブラウザ操作を確認します。オンラインACは未確認です。ハッシュに衝突が存在しないことを主張する検証ではありません。
