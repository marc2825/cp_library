---
title: Nim
documentation_of: //game_theory/nim.hpp
---
# Nim

[カテゴリ索引・共通仕様](README.md) / [操作できる図解](../../visualizations/game_theory/index.html#nim) / [一次資料・設計判断](references.md)

一手で一つの山から1個以上を取り、最後の石を取ると勝ちです。`nim_sum(piles)`は全山のXORをuint64_tで返し、`nim_winning(piles)`は手番側が勝てるかを返します。空・全0はXOR=0で負けです。

`nim_winning_move(piles)`は勝てるときoptional<NimMove>を返します。`NimMove {size_t pile; uint64_t remaining,removed;}`は元の山の添字、移動後の石数、取り除く個数。元の配列は変更しません。敗勢ではnulloptです。

XORをsとし、`target = piles[i] ^ s`が元の石数より小さい山を選びます。その山をtargetにすると、新しいXORは`s ^ piles[i] ^ target = 0`。s≠0ならこの条件の山が必ず存在します。添字順で最初の候補を返します。

入力は`vector<Int>`で、boolを除く64ビット以下の標準整数型。符号付きでも全要素が非負なら使え、負値はinvalid_argumentです。結果のremainingは元の値以下なので、元の符号付き型にも収まります。総石数の加算はしないため、uint64_tの最大値を持つ山が複数あってもXORを計算できます。

## usage

```cpp
#include <iostream>
#include <vector>
#include "game_theory/nim.hpp"
int main(){
    std::vector<unsigned long long> piles{3,4,5};
    std::cout<<cp::nim_sum(piles)<<' '<<cp::nim_winning(piles)<<'\n';
    if(auto m=cp::nim_winning_move(piles))
        std::cout<<m->pile<<' '<<m->remaining<<' '<<m->removed<<'\n';
}
```

[コンパイルできる例](../../examples/game_theory/nim.cpp)。

## 条件・計算量

O(n) 時間・O(1) 作業領域。kは列または独立成分の数、nは山数、V/Eは全局面／全合法手の数。標準C++17・namespace cp、外部依存と整数拡張はありません。Graph版だけリポジトリ内の既存Graphを利用します。空・型・例外の規約は[共通仕様](README.md#共通仕様)も参照してください。

## 使用する問題・検証

[CSES Nim Game I](https://cses.fi/problemset/task/1730/)形式のローカル検証を用意しました。勝ち手は合法性と相手の敗勢も確認し、Grundyの独立和は積ゲームの完全探索と比較します。[検証入口](../../verify/run_game_theory.py)／[全検証範囲](README.md#ローカル検証)。オンラインACは未確認です。
