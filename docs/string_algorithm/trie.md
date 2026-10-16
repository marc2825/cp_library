---
title: Trie（バイト列の多重集合）
documentation_of: //string_algorithm/trie.hpp
---

# Trie（バイト列の多重集合）

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/string_algorithm/index.html#trie) / [実装](../../string_algorithm/trie.hpp) / [一次資料](references.md)

## usage

```cpp
#include <iostream>
#include "string_algorithm/trie.hpp"
int main() {
    cp::Trie trie;
    trie.insert("app"); trie.insert("apple"); trie.insert("app"); trie.insert("");
    std::cout << trie.size() << ' ' << trie.count("app") << ' ' << trie.count_prefix("app") << '\n';
    trie.erase("app");
    std::cout << trie.count("app") << ' ' << trie.contains("ap") << ' ' << trie.count("") << '\n';
}
```

[動く使用例](../../examples/string_algorithm/trie.cpp)。リポジトリのルートをincludeパスに追加します。C++17・namespace cp。[共通仕様](README.md#共通仕様)も参照してください。

## API・使用条件

`Trie`は任意のunsigned byteを辺に持つ文字列の多重集合です。アルファベット26文字に限定せず、0バイトやUTF-8も登録できます。空文字列はrootで終わる単語として扱います。

| API | 意味 |
|---|---|
| insert(string_view word) -> void | 単語を1個挿入 |
| erase(word) -> bool | 1個だけ削除。存在しなければfalseで個数不変 |
| count(word) -> size_t | 完全一致する単語の個数 |
| contains(word) -> bool | countが0以外か |
| count_prefix(prefix) -> size_t | この接頭辞を持つ単語の個数 |
| size() / empty() | 重複を含む全体個数／空か |
| node_count() | rootと、過去の登録で作られたノードの個数 |
| clear() | 登録単語と過去のノードを全て消し、root1個へ戻す |

空prefixの個数はsize()、count(空文字列)は空の単語だけの個数です。削除で個数0になった辺・ノードは保持し、再挿入時に再利用します。コピーは独立した多重集合。move元は空のroot1個に戻ります。

ノード数はINT_MAXまで。超過は変更前にlength_error、全体個数のsize_t超過は変更前にoverflow_errorです。通常のメモリ確保失敗に対する内部ノード数の巻戻しは保証しません。再帰を使わず、深い単語も扱います。

## 原理・計算量

ノードに、この接頭辞を通る個数passedと、ここで終わる個数terminalを保持します。辺はbyte順の小さなvectorへ疎に格納し、lower_boundで遷移先を探します。256個の子ポインタを各ノードに確保しません。

単語長L、最大出次数σ≤256として、検索・個数・prefixはO(L log(σ+1))。挿入はO(L log(σ+1)+新規辺数*σ)、削除はO(L log(σ+1))時間です。バイトの種類は固定なのでいずれもLに比例する上限を持ちます。挿入・削除は経路を保持するO(L)追加領域、格納全体は過去に作った異なる接頭辞の個数に比例。clearはノード数に比例します。

## 使用する問題・検証範囲

[AOJ ALDS1_4_C Dictionary](https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=ALDS1_4_C)の[ラッパ](../../verify/string_algorithm/trie.test.cpp)。独立な辞書で重複数・prefixを全走査し、削除後の過去のノード数も接頭辞集合から確認します。

[検証入口](../../verify/run_string_algorithm.py)は10ヘッダ（内部補助・all含む）、8使用例、8ジャッジ形式・512ケースを確認します。9,841三値文字列・180全バイト列、3,937 KMP組、120 Trie履歴・48,000操作、300組のハッシュ・6,000区間問い合わせ、Python独立検証4,126クエリ（うちTrie 2,880操作）を定義式と照合。0/FF・UTF-8のバイト・空・重複・符号付き整数の極値・base境界・入力拒否、20万文字の同一／交互列・深いTrieと比較回数の線形上限を含みます。

通常・ASan/UBSan/LeakSanitizerのdebug/releaseを実行します。図解は8モード、3,096入力・51,386スナップショット、16拒否条件とブラウザ操作を確認します。オンラインACは未確認です。ハッシュに衝突が存在しないことを主張する検証ではありません。
