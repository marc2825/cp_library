---
title: Misère Nim
documentation_of: //game_theory/misere_nim.hpp
---
# Misère Nim

[カテゴリ索引・共通仕様](README.md) / [操作できる図解](../../visualizations/game_theory/index.html#misere) / [一次資料・設計判断](references.md)

一手で一つの山から1個以上を取る点はNimと同じですが、**最後の石を取った側が負け**です。`misere_nim_winning(piles)`と`misere_nim_winning_move(piles)`を提供します。型・非負条件・NimMoveの意味は[Nim](nim.md)と共通です。

| 2個以上の山の数 | 勝敗・勝ち手 |
|---|---|
| 0 | 1個の山が偶数個なら勝ち。1個取り、奇数個を残す |
| 1 | 常に勝ち。1個の山が偶数個なら大きい山を1個へ、奇数個なら0個へ減らす |
| 2以上 | Nimと同じくXOR≠0なら勝ち。XOR=0へ移す |

0個の山は数に含めません。2個以上の山が一つだけになった時点では、相手へ奇数個の1を渡す手が作れます。

**空・全0の終局は、手番側の勝ちという規約**です。最後の石を取った前の手番側が負けたと解釈します。この場合winningはtrueでも、合法手がないのでwinning_moveはnulloptです。非終局の勝勢なら必ず勝ち手を返します。1個だけの山では負けとなり、勝ち手はありません。

この判定はMisère Nim専用です。通常ルール用Grundy数のXORを、そのまま一般のmisèreゲームへ適用することはできません。

## usage

```cpp
#include <iostream>
#include <vector>
#include "game_theory/misere_nim.hpp"
int main(){
    std::vector<int> piles{1,1,5};
    std::cout<<cp::misere_nim_winning(piles)<<'\n';
    if(auto m=cp::misere_nim_winning_move(piles))
        std::cout<<m->pile<<' '<<m->remaining<<' '<<m->removed<<'\n';
    const std::vector<int> empty;
    std::cout<<cp::misere_nim_winning(empty)<<' '<<bool(cp::misere_nim_winning_move(empty))<<'\n';
}
```

[コンパイルできる例](../../examples/game_theory/misere_nim.cpp)。

## 条件・計算量

O(n) 時間・O(1) 作業領域。kは列または独立成分の数、nは山数、V/Eは全局面／全合法手の数。標準C++17・namespace cp、外部依存と整数拡張はありません。Graph版だけリポジトリ内の既存Graphを利用します。空・型・例外の規約は[共通仕様](README.md#共通仕様)も参照してください。

## 使用する問題・検証

[HackerRank Misère Nim](https://www.hackerrank.com/challenges/misere-nim-1/problem)形式のローカル検証を用意しました。勝ち手は合法性と相手の敗勢も確認し、Grundyの独立和は積ゲームの完全探索と比較します。[検証入口](../../verify/run_game_theory.py)／[全検証範囲](README.md#ローカル検証)。オンラインACは未確認です。
