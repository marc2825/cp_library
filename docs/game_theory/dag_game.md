---
title: DAGゲーム・独立和と勝ち手
documentation_of: //game_theory/dag_game.hpp
---
# DAGゲーム・独立和と勝ち手

[カテゴリ索引・共通仕様](README.md) / [操作できる図解](../../visualizations/game_theory/index.html#dag) / [一次資料・設計判断](references.md)

`DAGGame(moves)`または`DAGGame(directed_graph)`は、移動先とGrundy数を保持して問い合わせに答えます。入力検査は[Grundy計算](grundy.md)と同じです。構築後は変更せず、別のグラフを解くときは作り直します。`size()`、`transitions()`、`grundy()`、`topological_order()`の配列取得はconst参照です。

| API | 意味・返り値 | 時間 |
|---|---|---|
| grundy(v) / winning(v) | intの値／値≠0なら勝ち | O(1) |
| move_to_grundy(v,target) | targetの値を持つ最初の移動先、なければnullopt | O(outdegree(v)) |
| winning_move(v) | 値0へ移る最初の辺、なければnullopt | O(outdegree(v)) |
| nim_sum(positions) / winning(positions) | 各独立ゲームのXOR／その勝敗 | O(k) |
| winning_move(positions) | optional<DAGGameMove> | O(k+選んだ頂点のoutdegree) |

`positions`の各要素は独立したゲームの現在頂点です。一手で**ちょうど一成分だけ**を辺に沿って動かします。同じ頂点が複数回現れても別の成分であり、相互の衝突・占有制限はありません。値0の成分も残してよく、空の独立和は負けです。

`DAGGameMove {size_t component; int from,to;}`はpositionsの添字と元・先の頂点です。XORをsとすると、`target = grundy(from) ^ s < grundy(from)`となる最初の成分を選び、そのtargetへ移せる辺を探します。mexの定義により小さい値は全て選択肢にあり、勝勢なら必ずXOR=0へ移る合法手があります。

有効な頂点は[0,size)。範囲外の単体・成分問い合わせはout_of_range、負のtargetはinvalid_argumentです。全成分を検査するので、前半の値が0でも後半の不正頂点を見逃しません。重複した選択肢では登録順の最初を返します。

通常ルール専用で、DAGなので引き分けはありません。Misère Nimの終局規約はこのクラスへ持ち込みません。

## usage

```cpp
#include <iostream>
#include <vector>
#include "game_theory/dag_game.hpp"
int main(){
    cp::Graph<int> graph(4,cp::GraphKind::Directed);
    graph.add_edge(1,0);graph.add_edge(2,0);graph.add_edge(2,1);
    graph.add_edge(3,1);graph.add_edge(3,2);
    cp::DAGGame game(graph);
    std::vector<int> tokens{1,2}; // Independent tokens; one moves each turn.
    std::cout<<game.nim_sum(tokens)<<' '<<game.winning(tokens)<<'\n';
    if(auto m=game.winning_move(tokens))
        std::cout<<m->component<<' '<<m->from<<' '<<m->to<<'\n';
}
```

[コンパイルできる例](../../examples/game_theory/dag_game.cpp)。

## 条件・計算量

構築 O(V+E) 時間・O(V+E) 保持領域。kは列または独立成分の数、nは山数、V/Eは全局面／全合法手の数。標準C++17・namespace cp、外部依存と整数拡張はありません。Graph版だけリポジトリ内の既存Graphを利用します。空・型・例外の規約は[共通仕様](README.md#共通仕様)も参照してください。

## 使用する問題・検証

[AtCoder Educational DP K - Stones](https://atcoder.jp/contests/dp/tasks/dp_k)（石数0..Kを局面としてDAGを構築）形式のローカル検証を用意しました。勝ち手は合法性と相手の敗勢も確認し、Grundyの独立和は積ゲームの完全探索と比較します。[検証入口](../../verify/run_game_theory.py)／[全検証範囲](README.md#ローカル検証)。オンラインACは未確認です。
