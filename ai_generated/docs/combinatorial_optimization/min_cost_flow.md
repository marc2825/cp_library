---
title: 最小費用流
documentation_of: //combinatorial_optimization/min_cost_flow.hpp
---
# 最小費用流

[カテゴリ・使用条件](README.md) / [操作できる図解](../../visualizations/combinatorial_optimization/index.html#flow) / [参考資料](references.md)

`MinCostFlow<Cap=long long,Cost=long long>(n)`へ`add_edge(from,to,capacity,cost)`で有向辺を追加すると、登録順のint IDが返ります。容量は非負、費用は符号付き。Capはbool以外の64ビット以下の整数（符号なしも可）、Costは符号付き64ビット以下の整数です。自己ループ・平行辺・逆向きの元辺・容量0に対応します。

`flow(s,t,limit=Capの最大値)`は上限まで、または到達不能まで送流し、`Result {Cap flow; Cost cost;}`に**今回追加した流量と費用**を返します。希望量を満たしたかはresult.flow==limitで判定します。最大限送った中で費用を最小化するので、正の費用を避けて早く止まることはありません。

`slope(s,t,limit)`は今回の(0,0)からの追加流量と追加費用の折れ点vector<Result>。同じ単位費用の区間はまとめます。流量は増加し、単位費用は狭義増加します。**負費用を許すので総費用は単調増加とは限りません。** 各区間を直線補間すれば整数流量ごとの最小費用になります。

`get_edge(id)`は`Edge {from,to,capacity,flow,cost}`、`edges()`は登録順の全元辺を値で返します。逆辺は自動生成し、元辺の逆向き登録とは区別します。`reset_flow()`は全送流を0にし、辺・ID・容量・費用を保持します。

最初のflow/slope呼出し（limit=0も含む）からresetまでs,tと辺集合を固定します。同じs,tへの追加送流は可能。別s,tや辺追記はlogic_errorです。reset後には追記できます。ゼロ上限では探索せず(0,0)だけを返し、負閉路も検査しません。

負費用がある初回正流量では、仮想始点から全頂点へ到達するBellman–Fordでpotentialを計算します。**正容量辺に負閉路がある入力は、非連結成分も含めdomain_errorで拒否**します。負閉路を使って循環流を改善するアルゴリズムは含みません。拒否時点まで送流を変更しません。初回に負費用の残余辺がなければこの処理はO(V+E)です。

以後はreduced cost `cost+h[from]−h[to]`を非負に保ち、Dijkstraで増加路を選びます。逆辺を通って以前の選択を取り消せます。s,tが有効で異なること、容量・limitの非負を検査します。負のサイズ・容量・limitと同一端点はinvalid_argument、頂点・辺IDの範囲外はout_of_rangeです。

Costの範囲を超える追加費用、または内部128ビットの超過はoverflow_error。**各増加後の今回の費用がCostに収まること**が条件で、最終費用だけが収まっても途中超過は拒否します。現在の増加路は算術検査後に反映しますが、同じ呼出しの前の成功した増加は巻き戻しません。Capの上限は今回の追加流量に適用されます。

到達不能を確かめる最後の探索もあるため、探索回数は最大A+1です。初期化のO(V+VE)は初回正送流時だけ必要です。Aは一回のrunの増加回数で、整数容量ではA≤今回流量Fです。容量の大きさに依存するため、強多項式時間の保証はありません。flowは折れ点列を保存せず、slopeは追加でO(A)の結果領域を使います。

## usage

```cpp
#include <iostream>
#include <vector>
#include "combinatorial_optimization/min_cost_flow.hpp"
int main(){
    cp::MinCostFlow<> network(4);
    network.add_edge(0,1,2,-1);network.add_edge(1,3,2,3);
    network.add_edge(0,2,1,1);network.add_edge(2,3,1,4);
    auto first=network.flow(0,3,1);
    auto rest=network.flow(0,3);
    std::cout<<first.flow<<' '<<first.cost<<'\n'<<rest.flow<<' '<<rest.cost<<'\n';
    network.reset_flow();
    for(auto p:network.slope(0,3))std::cout<<p.flow<<':'<<p.cost<<' ';
    std::cout<<'\n';
}
```

[実行できる使用例](../../examples/combinatorial_optimization/min_cost_flow.cpp)。

## 計算量・数値の条件

初期化 O(V+VE)、探索 O((A+1)(V+E) log(V+E+1))、保持・作業 O(V+E)。nは行数、mは列数（全行を割り当てるときn≤m）、Vは頂点数、Eは登録辺数です。費用のある新実装はGCC/Clangの検査付き128ビット演算を使用します。返す総費用はlong longまたはCostに収まる必要があり、超過はoverflow_errorです。[共通仕様](README.md#共通仕様)を参照してください。

## 問題・検証

[AOJ GRL_6_B](https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=GRL_6_B)形式のローカル検証コードを用意しました。独立した全割当・頂点部分集合DP・全整数フローと比較し、境界、数値極値と復元も確認します。[検証入口](../../verify/run_combinatorial_optimization.py)と[全検証範囲](README.md#ローカル検証)を参照。オンラインACは未確認です。
