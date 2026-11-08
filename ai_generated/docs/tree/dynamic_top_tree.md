---
title: Dynamic Top Tree
documentation_of: //tree/dynamic_top_tree.hpp
---

# Dynamic Top Tree（T-20）

辺のlink/cut、頂点の値変更、任意根の成分・部分木DPをオンラインで扱います。preferred pathをcompress用splay、lightな枝の集約をrake用splayで維持します。固定トポロジーの `StaticTopTreeDP` / `DynamicRerootingTreeDP` は既存のまま利用できます。

## API・条件

`DynamicTopTree<Spec>(vector<Spec::Info>)` は独立したn頂点の森を作ります。頂点番号は `[0,n)`。空も構築でき、範囲外の番号は `out_of_range`。

| 操作 | 戻り値・意味 |
|---|---|
| `link(u,v)` | 辺追加。同一成分・自己ループはfalse |
| `cut(u,v)` | その直接の辺を削除。辺がなければfalse |
| `connected(u,v)` | 同一成分か |
| `make_root(v)` | 内部で使う根を変更 |
| `set_vertex(v,x)` / `get_vertex(v)` | 頂点情報の代入 / コピー取得 |
| `component_cluster(root)` | rootを境界にする成分全体のPath |
| `subtree_cluster(root,v)` | 指定根でv以下のPath。非連結はnullopt |
| `path_cluster(u,v)` | spineをu→vにしたPath。脇の枝も含む。非連結はnullopt |
| `size()` / `allocated_nodes()` | 頂点数 / 確保した頂点・rakeノード数（再利用待ちも含む） |

問い合わせも内部構造を変更します。root引数は各問い合わせで明示するため、直前の根の状態に依存しません。失敗したlink/cutは辺集合・値を変更しませんが、内部の根やsplay形状は変わり得ます。`Path` を値で返すので後の操作で参照が失効しません。コピーは禁止、moveは可能。移動元は破棄または再代入してください。

## Spec（頂点型rake/compress）

`Info` は頂点のデータ、`Path` は向きのあるspineとそこに付く枝、`Point` は仮の親へ接続する枝の集約です。各型はコピー可能にし、次のstatic関数を定義します。

| 関数 | 意味 |
|---|---|
| `vertex(info) -> Path` | 頂点単独 |
| `add_vertex(point,info) -> Path` | light枝の集約を頂点に取り付ける |
| `add_edge(path) -> Point` | pathの上端を仮の親に接続する |
| `rake(point,point) -> Point` | 同じ親に付く枝を集約 |
| `compress(upper,lower) -> Path` | 隣接するspineを上から下の順に連結 |

rakeは結合的・可換で、逆元や単位元は不要です。compressは結合的で、非可換でも構いません。さらに合法なpreferred-path分解をどれに変えても、同じ根付き木のDPを表さなければなりません。単に2つの任意のモノイドを渡せばよいわけではありません。頂点単体のspine反転は同じ値を表し、長いspineの反転は逆順のcompressを使います。隣接順を意味に含むDP、境界方向によって頂点単体の値が変わるSpecは対象外です。

辺はコストなしです。辺のデータが必要なら独立した頂点として細分する設計が可能ですが、その場合のSpecや距離の扱いは利用者が定義します。新規頂点の追加はなく、初期値の配列で全頂点を確保します。例外を投げない演算を前提とし、利用者のSpecやメモリ確保が途中で例外を投げた後の継続利用は保証しません。

## 計算量・用途

構築 O(n)、各操作はならし O(log n)、空間 O(n)。Specの値コピー・各演算がO(1)の場合です。単独の操作に最悪O(log n)を保証するものではありません。頂点・rakeノードは所有権付きで管理し、削除したrakeノードを再利用するので、更新回数に比例してメモリが増えません。

根付き木DP、動的ネットワークの距離和などに使います。純粋なpath productなら既存 `LinkCutTree` が使えます。今回の `path_cluster` はoff-spineも含むので、Specで必要な集計を分けてください。

## 検証・出典

方向付き文字列によるcompressの非可換検査、全列挙の独立集合、根依存の距離和を別解と比較します。共通coreのlazy版では15万操作と10万頂点の鎖・星型、move・破棄・メモリ再利用も検査します。

coreはLuzhiledのUnlicense実装を版固定で取り込み、所有権・再利用・祖先からのpush・公開APIを変更しました。詳細とライセンスは[一次資料](../tree_extensions/references.md)、[出典台帳](../../verification/provenance.json)を参照してください。

## 使用例

```cpp
#include <iostream>
#include "../../tree/dynamic_top_tree.hpp"
#include "../../tree/fully_dynamic_tree_dp.hpp"
int main() {
    cp::DynamicTopTree<cp::DistanceSumTreeDP<long long>> tree({1, 2, 3, 4});
    tree.link(0, 1); tree.link(1, 2); tree.link(1, 3);
    std::cout << tree.component_cluster(0).top << '\n';
    tree.cut(1, 3); tree.link(2, 3);
    std::cout << tree.component_cluster(0).top << '\n';
    std::cout << tree.subtree_cluster(0, 2)->mass << '\n';
}
```

出力:

```text
16
20
7
```

[操作できる図解](../../visualizations/tree_extensions/index.html#dynamic_top_tree) / [検証範囲・コマンド](../tree_extensions/README.md) / [一次資料と設計判断](../tree_extensions/references.md) / [木カテゴリ](README.md)
