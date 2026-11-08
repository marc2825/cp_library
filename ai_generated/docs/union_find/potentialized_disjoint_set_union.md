---
title: 重み付き Union-Find（Potentialized DSU）
documentation_of: //union_find/potentialized_disjoint_set_union.hpp
---

# 重み付き Union-Find

`potential[v] - potential[u] = w` という差分制約を追加し、2頂点間の差分や連結性を問い合わせます。通常版と同じように成分サイズ・成分数も取得できます。

[インタラクティブな解説](../../visualizations/union_find/index.html#potential) / [実装](../../union_find/potentialized_disjoint_set_union.hpp) / [参考資料と設計方針](references.md)

## 使用例

```cpp
#include "union_find/potentialized_disjoint_set_union.hpp"

cp::PotentializedDisjointSetUnion<long long> dsu(4);
dsu.merge(0, 1, 5);  // p[1] - p[0] = 5
dsu.merge(1, 2, -2); // p[2] - p[1] = -2
auto difference = dsu.diff(0, 2); // optional<long long>(3)
auto unknown = dsu.diff(0, 3);    // nullopt
auto redundant = dsu.merge(0, 2, 3); // AlreadyConsistent
auto rejected = dsu.merge(0, 2, 4);  // Contradiction
// 矛盾する制約は採用しないため、diff(0,2) は引き続き3。
int count = dsu.component_count(); // 2
int size = dsu.size(0);            // 3
```

動く例は [potential_constraints.cpp](../../examples/union_find/potential_constraints.cpp)。

## API・計算量

頂点は `[0,n)`。構築・メモリは O(n)。演算 `T` の加減算・比較・コピーを O(1) とした計算量です。

| API | 意味 | 時間 |
|---|---|---|
| `PotentializedDisjointSetUnion<T>(n = 0)` | n 個の単独成分 | O(n) |
| `leader(v)` | 差分も更新しながら経路圧縮して代表元を返す | 償却 O(α(n)) |
| `merge(u,v,w)` | p[v] - p[u] = w を要求し、結果を3種類で返す | 償却 O(α(n)) |
| `diff(u,v)` | p[v] - p[u]。別成分なら nullopt | 償却 O(α(n)) |
| `potential(v)` | 現在の代表元を0とした v の相対値 | 償却 O(α(n)) |
| `same(u,v)` / `size(v)` | 連結判定／成分サイズ | 償却 O(α(n)) |
| `component_count()` / `vertex_count()` | 成分数／頂点数 | O(1) |
| `groups()` | 通常版と同じ順序の成分一覧 | O(n α(n)) |
| `reset(n)` | すべての制約を捨てて再初期化 | O(n) |

`merge` の返り値は `cp::PotentialMergeResult`（クラス内では `MergeResult` でも参照可能）です。

| 値 | 意味 | 成分数 |
|---|---|---|
| `Merged` | 異なる成分に制約を追加して併合した | 1減る |
| `AlreadyConsistent` | 既存の関係だけで同じ制約が成立している | 変わらない |
| `Contradiction` | 既存の関係と矛盾するため追加を拒否した | 変わらない |

`enum class` なので bool には暗黙変換しません。「制約が有効か」は `result != cp::PotentialMergeResult::Contradiction`、「新たに併合したか」は `result == cp::PotentialMergeResult::Merged` です。

拒否時も内部の経路圧縮は行いますが、採用済みの差分関係は変えません。成分を「矛盾状態」にする設計ではなく、矛盾した制約を受け付けない設計です。

## 原理と符号

親への差分 `weight[v] = p[v] - p[parent[v]]` を持ちます。根の差分は0です。経路圧縮では親までの差分を加算して、根までの差分に直します。

u, v の根を a, b、根からの相対値を U = p[u] - p[a]、V = p[v] - p[b] とすると、b を a の下につなぐための差分は

```text
p[b] - p[a] = U + w - V
```

です。サイズを見て接続方向を反転するときは、この値の符号も反転します。同じ成分なら `V - U == w` かどうかで整合・矛盾を判断します。

[操作できる図](../../visualizations/union_find/index.html#potential) では、この計算、接続方向の反転、経路圧縮、矛盾の拒否を1ステップずつ確認できます。

## 型・境界条件

- `T(0)` が単位元であり、`+`、二項 `-`、`==` が使える可換群を指定します。通常は `long long`。modint や座標ベクトルにも利用できます。
- XOR を `+` と `-` として定義した型なら XOR 制約も扱えます。整数型をそのまま渡した場合は加法の差分です。
- 非可換な群には対応しません。浮動小数点の近似比較も内蔵せず、整合判定は厳密な `==` を使います。
- 整数では加減算の途中も含めて T に収まる範囲を使用します。オーバーフロー検査は追加していません。
- 絶対的な p[v] は決まりません。`potential(v)` は代表元の変更で変わりますが、同一成分の `diff(u,v)` は不変です。
- `diff(v,v)` は0。`merge(v,v,0)` は整合、0以外なら矛盾です。空の DSU は頂点数・成分数とも0です。
- 経路圧縮に再帰を使いますが、union by size により圧縮前でも深さは O(log n) です。

## 使う問題

- [AOJ DSL_1_B](https://onlinejudge.u-aizu.ac.jp/problems/DSL_1_B)：p[y] - p[x] の向きで、`merge(x,y,z)`、`diff(x,y)` をそのまま使えます。[検証プログラム](../../verify/union_find/weighted_unionfind_aoj.test.cpp)。
- [Library Checker: Unionfind with Potential](https://judge.yosupo.jp/problem/unionfind_with_potential)：998244353 を法とする差分制約と矛盾判定。問題は a[u] - a[v] の向きなので `merge(v,u,x)`、`diff(v,u)` に対応させます。[検証プログラム](../../verify/union_find/unionfind_with_potential.test.cpp)。
- 差分制約の追加と問い合わせ、相対座標の整合判定、mod 上の関係管理。

## 検証

`python3 verify/run_union_find.py --sanitize`。小規模な制約履歴の全探索、BFS で差分を求める独立した愚直実装との比較、整数・mod・XOR、重複／逆向き／自己制約、矛盾拒否、接続方向反転、再初期化、20万要素を検証します。問題形式の入出力も別の Python オラクルと比較します。

ローカル検証済みですが、外部ジャッジへの提出はまだ行っていません。
