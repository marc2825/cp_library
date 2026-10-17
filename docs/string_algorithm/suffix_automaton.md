---
title: Suffix Automaton
documentation_of: //string_algorithm/suffix_automaton.hpp
---

# Suffix Automaton

任意のバイト列に1文字ずつ追加でき、部分文字列の所属・出現回数・初出位置・異なる部分文字列数を求めます。

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/final_a/index.html#suffix_automaton) / [追加APIと検証](../final_a/README.md) / [一次資料](../final_a/references.md)

## API・条件

`SuffixAutomaton(string_view text={})`、`append(unsigned char)` / `append(string_view)`、`clear()`。`size()`は追加済み文字数、`state_count()`はrootとcloneを含む状態数です。文字はunsigned byteとして扱い、NUL・高位byteも使えます。

`contains(pattern)`、`first_occurrence(pattern) -> optional<int>`、`count_occurrences(pattern) -> uint64_t`、`distinct_substrings() -> uint64_t`。空patternは所属し、初出0、出現数n+1。未出現はfalse / nullopt / 0です。重複する出現も数えます。

`states()`はconst vector<State>&。Stateはnext（map<unsigned char,int>）、link、length、first_end、cloneを持ちます。rootのlinkとfirst_endは-1。状態の最短長はlength[link]+1です。参照はappend/clear等で無効になり得ます。

`longest_common_substring(text)`は共通APIのCommonSubstringResultを返します。[別の説明](longest_common_substring.md)へ。追加後の全長は(INT_MAX−1)/2以下で、超過はlength_error。出現数のキャッシュは追加で無効化され、次の成功した非空patternの問い合わせで再構築します。

## Usage

```cpp
#include <iostream>
#include "string_algorithm/suffix_automaton.hpp"
int main() {
    cp::SuffixAutomaton a("abcbc");
    std::cout << a.distinct_substrings() << ' ' << a.count_occurrences("bc") << ' ' << *a.first_occurrence("bc") << '\n';
    a.append("bc");
    std::cout << a.size() << ' ' << a.count_occurrences("bc") << '\n';
}
```

出力:

```text
12 2 1
7 3
```

[コンパイルできる使用例](../../examples/string_algorithm/suffix_automaton.cpp)。

## 原理

各状態は同じendpos集合を持つ部分文字列をまとめます。新しい末尾状態を作り、suffix linkを辿って不足する遷移を追加します。既存遷移の長さが合わないと状態をcloneし、遷移とsuffix linkを付け替えます。出現数は各prefixの末尾状態に1を置き、長さの降順にsuffix linkへ加えます。

## 計算量

n文字・アルファベットσ<=256。構築はO(n log σ)時間、O(n)領域。append1回は最悪O(n log σ)、全体では上記に償却。contains/first_occurrenceはO(m log σ)。count_occurrencesはキャッシュ有効ならO(m log σ)、無効ならO(n+m log σ)。distinct_substringsはO(n)。statesはO(1)で参照を返します。

## 使用する問題と検証

[Library Checker Number of Substrings](https://judge.yosupo.jp/problem/number_of_substrings)。出現回数と初出も別の全探索で確認します。

`python3 verify/run_final_a.py`（`--sanitize`対応）で単独include・複数翻訳単位・このusageと出力・独立した素朴解・境界と例外を確認します。[詳しい検証範囲](../final_a/README.md)。オンライン提出・ACは未確認です。
