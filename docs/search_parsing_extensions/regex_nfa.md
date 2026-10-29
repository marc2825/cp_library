---
title: 正規表現からNFA・Thompson構成
documentation_of: //parsing/regex_nfa.hpp
---

# X-03 正規表現からNFA・Thompson構成

[一覧](README.md) / [ヘッダ](../../parsing/regex_nfa.hpp) / [実行例](../../examples/search_parsing_extensions/regex_nfa.cpp) / [操作できる図解](../../visualizations/search_parsing_extensions/index.html#regex_nfa) / [出典](references.md)

連接・選択・繰返しをε辺で結び、入力を読むごとに到達可能な状態集合を進める。

## APIと条件

regex_to_nfa(pattern,ParseLimits)、regex_full_match(nfa,text)。

バイト単位の全体一致。literal、.、連接、|、()、*+?、文字集合/範囲/否定、エスケープ\n/\r/\t/\0/\xHH。空式・空選択肢はε。繰返し量指定・後方参照・anchor・Unicode文字単位処理は非対応。

regex_to_nfa(pattern,ParseLimits={})はNFAを返し、regex_full_match(nfa,text)で全体一致を調べます。記号はunsigned byteの0〜255で、UTF-8はバイト列として扱います。`.`は改行・NULを含む全byte。優先度は量指定*+?、連接、選択|です。空のpattern・空の()・空の選択肢はεを受理します。

文字集合は[abc]、[a-z]、[^a-z]。エスケープは\n/\r/\t/\0/\xHHと、次の文字をliteralとして読む形です。\dのようなクラス省略はありません。閉じ角括弧・ハイフン等はescapeしてliteralにできます。逆順range、不完全hex、空集合[]、重複量指定a**、閉じ忘れはParseError（バイト位置付き）。{m,n}、anchor ^/$、後方参照は実装しません。escapeした数字はliteralです。

Thompson構成の状態/ε辺をそのまま公開し、必要ならdeterminize/minimize_dfaを明示的に組み合わせます。NFA simulationはbacktrackingの分岐指数を避けますが、決定化は指数個の状態を作り得ます。ParseLimits::max_nodesはNFAの状態数（初期の未使用状態も含む）、max_depthは括弧再帰の上限です。

## 計算量・用途

正規表現長RでO(256R)構築・辺数（文字集合を列挙）、受理はNFA simulation。状態数と再帰深さを明示上限で制限。

問題例・用途: 小さい正規言語の認識、正規表現条件を満たす文字列のDP。

## 使用例

C++17、namespace cp。並列探索を含むプログラムは-pthreadでリンクします。

```cpp
#include "parsing/regex_nfa.hpp"
#include <iostream>
using namespace cp;
int main() {
    auto nfa = regex_to_nfa("(ab|c)*d?");
    const std::vector<std::string> words{"", "abccabd", "ac", "d", "abc"};
    for (std::size_t i = 0; i < words.size(); ++i)
        std::cout << (i ? " " : "") << regex_full_match(nfa, words[i]);
    std::cout << "\n";
}
```

出力:

```text
1 1 0 1 1
```

## 検証

[共通の検証手順・独立比較](README.md#検証)を使用します。図解の参照値はこのC++使用例の出力と一致させています。オンラインジャッジへの提出結果ではありません。
