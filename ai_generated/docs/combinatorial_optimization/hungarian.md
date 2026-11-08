---
title: Hungarian Algorithm
documentation_of: //combinatorial_optimization/hungarian.hpp
---
# Hungarian Algorithm

[カテゴリ・使用条件](README.md) / [操作できる図解](../../visualizations/combinatorial_optimization/index.html#hungarian) / [参考資料](references.md)

`hungarian(costs)`はn行m列から、全ての行へ異なる列を一つずつ割り当て、総費用を最小化します。`vector<vector<long long>>`と`vector<vector<optional<long long>>>`を受け、後者のnulloptは**禁止辺**です。有限な巨大コストを禁止辺の代わりには使いません。

解が存在すればoptional<AssignmentResult>、n>mやHall条件を満たさない場合はnulloptです。行の長さが異なればinvalid_argument。0行のvectorは0×0として扱い、費用0・空の結果になります。n×0（n>0）は不可能。0×mを区別する場合はAssignmentProblem(0,m)を使えます。

行・列potentialをu,vとすると、全ての許可辺でu[i]+v[j]≤cost[i][j]を保ち、選んだ辺は等号になります。未使用の列はv=0、それ以外の列もv≤0です。総費用とΣu+Σvが一致するので最適性を確認できます。負費用には行の最小値からpotentialを初期化します。

未訪問列へ向かうreduced costの最小値slackを保持し、最小slackだけpotentialを更新して探索範囲を広げます。禁止辺を飛ばしても、以前の行から得たslackは候補に残します。

AssignmentResultはcost、row_to_column、column_to_row（未使用列−1）、row_potential、column_potentialを持ちます。costはlong long、potentialはOptimizationWideです。potentialの表示はoptimization_to_string(value)。入力と返した結果を共有しません。

## usage

```cpp
#include <iostream>
#include <vector>
#include "combinatorial_optimization/hungarian.hpp"
int main(){
    std::vector<std::vector<long long>> cost{{4,1,3},{2,0,5},{3,2,2}};
    auto r=cp::hungarian(cost);
    if(r){std::cout<<r->cost<<'\n';for(auto j:r->row_to_column)std::cout<<j<<' ';std::cout<<'\n';}
}
```

[実行できる使用例](../../examples/combinatorial_optimization/hungarian.cpp)。

## 計算量・数値の条件

O(n²m+n+m) 時間・O(n+m) 作業領域（入力・出力を除く）。nは行数、mは列数（全行を割り当てるときn≤m）、Vは頂点数、Eは登録辺数です。費用のある新実装はGCC/Clangの検査付き128ビット演算を使用します。返す総費用はlong longまたはCostに収まる必要があり、超過はoverflow_errorです。[共通仕様](README.md#共通仕様)を参照してください。

## 問題・検証

[Library Checker Assignment](https://judge.yosupo.jp/problem/assignment)形式のローカル検証コードを用意しました。独立した全割当・頂点部分集合DP・全整数フローと比較し、境界、数値極値と復元も確認します。[検証入口](../../verify/run_combinatorial_optimization.py)と[全検証範囲](README.md#ローカル検証)を参照。オンラインACは未確認です。
