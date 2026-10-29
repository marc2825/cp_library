---
title: DFA/NFAの共通操作
documentation_of: //parsing/automata.hpp
---

# X-02 DFA/NFAの共通操作

[一覧](README.md) / [ヘッダ](../../parsing/automata.hpp) / [実行例](../../examples/search_parsing_extensions/automata.cpp) / [操作できる図解](../../visualizations/search_parsing_extensions/index.html#automata) / [出典](references.md)

ε閉包の集合をDFA状態にし、受理と遷移先の同値類で分割を細かくして最小化する。

## APIと条件

DFA・NFA、determinize、minimize_dfa、dfa_product、complement_dfa、dfa_digit_count。

記号は[0,alphabet)。DFAの−1遷移は拒否、NFAの−1記号はε。最小化は到達状態と補完sinkを対象にし、元状態mapを返す。決定化と積は状態数予算超過でlength_error。桁DPは既存digit_dpを共用。

DFA/NFA(states,alphabet,start=0)は1状態以上・alphabet≥0を要求します。add_state/set_start/set_accepting、DFA::set_transition(from,symbol,to)、NFA::add_transition(from,symbol,to)で構築します。DFAの欠けた遷移と状態−1は拒否sink、NFAのsymbol=−1はε辺です。NFA::advance(active,symbol)は前後のε閉包も計算し、昇順・重複なし集合を返します。

determinize(nfa,max_states=1000000)はautomatonとsubsets、dfa_product(a,b,operation,max_states)はautomatonとstates（元状態のpair）を返します。積のoperationはAutomatonBoolean::intersection/unite/difference/symmetric_difference。alphabetは一致が必要。到達可能な状態だけを生成し、上限超過でlength_errorです。空集合も拒否sinkとして明示します。

complete_dfa/complement_dfaは必要ならsinkを足して補完・受理反転。minimize_dfaはMoore分割法で、automaton/original_to_state/blocksを返します。未到達元状態のmapは−1、blocksには追加sinkのID（元の状態数）が含まれる場合があります。HopcroftのO(n log n)ではありません。

NFA受理は長さLでO((L+1)(n+m+n log n))、決定化は最大K≤2^nとしてO(Kσ(n+m+n log n+n log(K+1)))程度です。最小化はO(σn² log(n+1))、積は到達pair数Pに対しO(σP log(P+1))。σ=0でも状態処理の費用があるため式のσはmax(1,σ)と解釈します。

dfa_digit_count<Count>(upper,dfa,canonical=true)はalphabet=10のDFAで0..upper（両端含む）を数えます。既定は先行0を読まず、整数0だけ文字0を1個読みます。falseならupperと同桁数まで0埋めした文字列を判定します。既存digit_dpを共有し、O(10×桁数×状態数)、Countのoverflow/剰余型は利用側で選びます。

## 計算量・用途

DFA受理O(L)、NFA受理は各文字でε閉包。Moore最小化は最悪O(σn² log(n+1))、領域O(σn)。決定化は最大2^n状態、積は最大(n+1)(m+1)状態。詳細は個別説明。

問題例・用途: 禁止パターンの受理条件、言語の積・補集合、桁DPの条件構築。

## 使用例

C++17、namespace cp。並列探索を含むプログラムは-pthreadでリンクします。

```cpp
#include "parsing/automata.hpp"
#include <iostream>
using namespace cp;
int main() {
    NFA nfa(3, 2);
    nfa.set_accepting(2);
    for (int c : {0, 1}) {
        nfa.add_transition(0, c, 0);
        nfa.add_transition(2, c, 2);
    }
    nfa.add_transition(0, 0, 1);
    nfa.add_transition(1, 1, 2);
    auto dfa = determinize(nfa);
    auto minimal = minimize_dfa(dfa.automaton);
    std::cout << dfa.automaton.size() << " " << minimal.automaton.size() << "\n";
    const std::vector<std::vector<int>> words{{}, {0}, {0, 1}, {1, 0, 0}, {1, 1, 0, 1, 1}};
    for (std::size_t i = 0; i < words.size(); ++i)
        std::cout << (i ? " " : "") << minimal.automaton.accepts(words[i]);
    std::cout << "\n";
}
```

出力:

```text
4 3
0 0 1 0 1
```

## 検証

[共通の検証手順・独立比較](README.md#検証)を使用します。図解の参照値はこのC++使用例の出力と一致させています。オンラインジャッジへの提出結果ではありません。
