---
title: Assignment Problem
documentation_of: //combinatorial_optimization/assignment_problem.hpp
---
# Assignment Problem

[カテゴリ・使用条件](README.md) / [操作できる図解](../../visualizations/combinatorial_optimization/index.html#assignment) / [参考資料](references.md)

`AssignmentProblem(rows,columns)`は全セルを禁止として作ります。`set_cost(i,j,long long)`で許可・上書き、`forbid(i,j)`で禁止へ戻し、`cost(i,j)`でoptionalの値を取得します。`rows()`と`columns()`は次元です。禁止辺付き行列を受けるコンストラクタもあります。

`solve(AssignmentObjective::minimize)`は最小費用、`solve(AssignmentObjective::maximize)`は最大重みのAssignmentResultをoptionalで返します。既定はminimize。呼ぶたびに現在の入力から再計算し、以前の結果は独立した値です。負の次元・不明なenumはinvalid_argument、セル範囲外はout_of_rangeです。

**両目的とも全行の割当が必須**です。最大化で全重みが負でも、空の選択にはしません。仕事の数が作業者より多ければ解なし。未割当を許すには、その意味に合う費用を持つダミー列を利用者が追加します。最大化は内部の128ビットで符号反転するのでLLONG_MIN入力も扱います。

最小化の双対はHungarianと同じです。最大化では返すpotentialも元の符号へ戻し、u[i]+v[j]≥weight[i][j]、v[j]≥0、選択辺が等号、未使用列v=0、Σu+Σv=総重みとなります。

割当は容量1の「source→行→列→sink」の最小費用流にも帰着できます。二部マッチングは組数、割当は全行を埋めたときの費用を最適化するため、目的を区別してください。

AssignmentResultはcost、row_to_column、column_to_row（未使用列−1）、row_potential、column_potentialを持ちます。costはlong long、potentialはOptimizationWideです。potentialの表示はoptimization_to_string(value)。入力と返した結果を共有しません。

## usage

```cpp
#include <iostream>
#include <vector>
#include "combinatorial_optimization/assignment_problem.hpp"
int main(){
    cp::AssignmentProblem problem(2,3); // Initially all cells are forbidden.
    problem.set_cost(0,0,5);problem.set_cost(0,1,8);
    problem.set_cost(1,0,7);problem.set_cost(1,2,4);
    std::cout<<problem.solve()->cost<<'\n';
    std::cout<<problem.solve(cp::AssignmentObjective::maximize)->cost<<'\n';
    problem.forbid(0,0);problem.forbid(0,1);
    std::cout<<bool(problem.solve())<<'\n';
}
```

[実行できる使用例](../../examples/combinatorial_optimization/assignment_problem.cpp)。

## 計算量・数値の条件

構築 O(nm)・set/forbid/cost O(1)・solve O(n²m+n+m) 時間。nは行数、mは列数（全行を割り当てるときn≤m）、Vは頂点数、Eは登録辺数です。費用のある新実装はGCC/Clangの検査付き128ビット演算を使用します。返す総費用はlong longまたはCostに収まる必要があり、超過はoverflow_errorです。[共通仕様](README.md#共通仕様)を参照してください。

## 問題・検証

[Library Checker Assignment](https://judge.yosupo.jp/problem/assignment)形式のローカル検証コードを用意しました。独立した全割当・頂点部分集合DP・全整数フローと比較し、境界、数値極値と復元も確認します。[検証入口](../../verify/run_combinatorial_optimization.py)と[全検証範囲](README.md#ローカル検証)を参照。オンラインACは未確認です。
