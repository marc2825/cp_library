---
title: 動的木DP（Rerooting対応）
documentation_of: //tree/dynamic_rerooting.hpp
---

# 動的木DP（Rerooting対応）

[カテゴリ索引](README.md) / [B項目一覧](../b_7_11/README.md) / [操作できる図解](../../visualizations/b_7_11/index.html#dynamic_rerooting) / [実装](../../tree/dynamic_rerooting.hpp) / [一次資料と設計](../b_7_11/references.md)

Static Top Tree上の更新と、式の外側Contextの伝播を組み合わせ、更新後の任意根DPを求めます。距離和のSpecも同梱しています。

## usage

```cpp
#include <iostream>
#include "tree/dynamic_rerooting.hpp"
int main() {
    cp::Graph<long long> g(3); g.add_edge(0,1,2); g.add_edge(1,2,3);
    cp::DynamicTreeDistanceSum<> dp(g,{1,1,1});
    std::cout << dp.query(0) << ' '; dp.set_vertex(2,2); std::cout << dp.query(1) << '\n';
}
```

出力:

```text
7 8
```

## API・使用条件

`DynamicRerootingTreeDP<Spec,Weight>`はStaticTopTreeDPの更新APIにquery(v)を追加。SpecにはContextとroot_context/compress_left_context/compress_right_context/rake_context/vertex_point_context/edge_context/finishが必要。`DynamicTreeDistanceSum<T>`は頂点質量と辺長からΣ mass[u] dist(v,u)を返す具体版。

頂点・辺IDは0始まり。グラフを受ける関数は構築時点の入力に対する計算です。重みや値の型は特記した制約に従います。

## 仕組みと境界

topologyは固定。Contextはクラスタ外からtop/bottom境界へ入る情報。query時に根から頂点tokenまで式を下り、各兄弟の寄与を外側へ加えます。提供SpecのPoint={mass,distance}、Path={length,mass,top_sum,bottom_sum}。質量・辺長が負でも距離和の代数は有効ですが、中間加減乗算をTに収める必要があります。一般Specの各context関数は同じ根付きDPの再結合を表すよう利用者が定義します。

## Contextを定義する場合

StaticTopTreeDPのSpec契約に次を加えます。Contextは、現在見ているクラスタ以外の情報を境界へ投影した値です。演算は全てconstオブジェクトから呼び出せる必要があります。

| 演算 | 返すContextまたは答え |
|---|---|
| `root_context()` | 式の根の外側。通常は空。 |
| `compress_left_context(c,right,e)` | compressの左のPathを見るとき、右全体と接続辺をcに取り込む。 |
| `compress_right_context(c,left,e)` | compressの右のPathを見るとき、左全体と接続辺をcに取り込む。 |
| `rake_context(c,sibling)` | Pointの兄弟枝を外側に加える。 |
| `vertex_point_context(c,value,v)` | Vertex式のlight-Pointへ下りる際、頂点自身の値を外側に加える。 |
| `edge_context(c,e)` | 親境界のPointから、light辺eの先の子Pathへ外側を渡す。eは親→子。 |
| `finish(vertex_path,c)` | query対象のVertex式と外側から最終Answerを返す。 |

提供するTreeDistanceSumSpecではContext={top,bottom}で、それぞれのPointが境界までの質量と距離和を持ちます。edge_contextは質量Mに対し距離和へM×辺長を加えます。compress側では兄弟Pathの長さを使って外側の距離を平行移動します。rakeは質量と距離和の加算です。これによりqueryの度に全方位DP全体を作り直さず、式の高さぶんのContext伝播だけで答えます。

## 計算量

構築O(V)、領域O(V)、点・辺更新と任意根query O(log²(V+1))。 Vは頂点数、Eは論理辺数。テンプレートの比較・加減乗算・Specの各演算はO(1)として数えています。

## 使用する問題・検証

想定問題: Static Top Tree上の更新と、式の外側Contextの伝播を組み合わせ、更新後の任意根DPを求めます。距離和のSpecも同梱しています。 [実行できる小問題](../../examples/tree/dynamic_rerooting_b.cpp)。

動的木DP（Rerooting対応）を必要とする3.11の最適化・構造判定・更新クエリで利用します。上の使用例は実際にコンパイル・実行し、文書中のコードとの一致も検査します。

`python3 verify/run_b_7_11.py` / `--sanitize`。[検証コード](../../verify/b_7_11/)には定義からの全列挙、独立した素朴解、乱択比較と復元の整合性検査を置いています。[比較方法](../b_7_11/README.md#ローカル検証)を参照してください。オンライン提出・ACは未確認です。図解は小入力の教材モデルで、C++との差異は[図解の説明](../../visualizations/b_7_11/README.md)へ明記しています。
