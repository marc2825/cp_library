---
title: Grundy数計算
documentation_of: //game_theory/grundy.hpp
---
# Grundy数計算

[カテゴリ索引・共通仕様](README.md) / [操作できる図解](../../visualizations/game_theory/index.html#grundy) / [一次資料・設計判断](references.md)

`grundy_mex(values)`は非負int列に含まれない最小の非負整数を返します。空のmexは0、重複は無視します。値がINT_MAXでも、その値の大きさの配列は確保せず、要素数kまでの印だけを持ちます。負値はinvalid_argument、長さがINT_MAXを超えればlength_errorです。

`calculate_grundy(moves)`は`vector<vector<int>>`の隣接リストから、`GrundyResult {vector<int> values,topological_order}`を返します。頂点は局面、有向辺v→uは一手でvからuへ移る意味です。`values[v]=mex({values[u] | v→u})`。終端は0。topological_orderは移動が前から後へ向かう順で、計算自体は逆順に行います。

入力は有限・完全情報・二人の合法手が同じ**通常ルール（手がない側の負け）**のゲームです。自己ループや全成分の閉路をinvalid_argumentで拒否し、範囲外の移動先はout_of_rangeです。開始局面から到達しない成分も検査します。空、孤立点、重複辺、非連結DAGを許します。入力長V≤INT_MAXで、超過はlength_error。

`calculate_grundy(const Graph<Cost>&)`は既存の有向Graphにも対応し、辺の費用を無視して移動先だけを使います。無向Graphはinvalid_argumentです。既存Graphへの辺登録自体は、そのAPIのassert条件を守ります。

各頂点の値は異なる移動先の数以下です。全頂点ごとにV要素を初期化せず、頂点番号を印として使うことで、mex走査の合計もO(V+E)に抑えます。再帰を使わず深いDAGにも対応します。

一手で複数の独立ゲームに分かれる場合は、各選択肢の子ゲームのGrundy数をXORしてから、その選択肢の列へgrundy_mexを適用できます。calculate_grundyの各辺は単一の次局面であり、ゲーム分割の状態列挙を自動化する関数ではありません。

## usage

```cpp
#include <iostream>
#include <vector>
#include "game_theory/grundy.hpp"
int main(){
    std::cout<<cp::grundy_mex({0,2,2,1000000000})<<'\n';
    const std::vector<std::vector<int>> moves{{},{0},{0,1},{1,2}};
    auto r=cp::calculate_grundy(moves);
    for(auto g:r.values)std::cout<<g<<' ';
    std::cout<<'\n';
}
```

[コンパイルできる例](../../examples/game_theory/grundy.cpp)。

## 条件・計算量

mex: O(k) 時間・O(k) 領域、DAG: O(V+E) 時間・O(V) 作業領域。kは列または独立成分の数、nは山数、V/Eは全局面／全合法手の数。標準C++17・namespace cp、外部依存と整数拡張はありません。Graph版だけリポジトリ内の既存Graphを利用します。空・型・例外の規約は[共通仕様](README.md#共通仕様)も参照してください。

## 使用する問題・検証

[CSES Nim Game II](https://cses.fi/problemset/task/1098/)（1..3個の除去ではg(x)=x mod 4。小さいDAGの値を周期で参照）形式のローカル検証を用意しました。勝ち手は合法性と相手の敗勢も確認し、Grundyの独立和は積ゲームの完全探索と比較します。[検証入口](../../verify/run_game_theory.py)／[全検証範囲](README.md#ローカル検証)。オンラインACは未確認です。
