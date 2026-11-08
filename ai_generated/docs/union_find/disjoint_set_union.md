---
title: Union-Find・連結成分サイズと個数
documentation_of: //union_find/disjoint_set_union.hpp
---

# 通常の Union-Find

無向グラフへの辺追加に対して、連結判定、成分サイズ、成分数を管理します。3.2「Union-Find・連結性管理」の S ランクの実装です。

[インタラクティブな解説](../../visualizations/union_find/index.html#ordinary) / [実装](../../union_find/disjoint_set_union.hpp) / [参考資料と設計方針](references.md)

## 使用例

```cpp
#include "union_find/disjoint_set_union.hpp"

cp::DisjointSetUnion dsu(5);
bool added = dsu.merge(0, 1); // true: 異なる成分を併合
dsu.merge(1, 2);
bool connected = dsu.same(0, 2); // true
int size = dsu.size(1);          // 3
int count = dsu.component_count(); // 3
auto groups = dsu.groups();     // {{0,1,2},{3},{4}}
```

コンパイルして動かせる例は [connectivity.cpp](../../examples/union_find/connectivity.cpp)。

## API・計算量

頂点は `[0,n)`。`n >= 0`、各問い合わせの頂点番号は有効である必要があります。構築・保持メモリは O(n)。

| API | 意味 | 時間 |
|---|---|---|
| `DisjointSetUnion(n = 0)` | n 個の単独成分を作る | O(n) |
| `leader(v)` | 代表元を返し、経路を圧縮する | 償却 O(α(n)) |
| `merge(u,v)` | 新たに併合したら true、既に同じ成分なら false | 償却 O(α(n)) |
| `same(u,v)` | 同じ成分か | 償却 O(α(n)) |
| `size(v)` | v を含む成分の頂点数 | 償却 O(α(n)) |
| `component_count()` | 現在の成分数 | O(1) |
| `vertex_count()` | 頂点数 | O(1) |
| `groups()` | 成分ごとの頂点列を返す | O(n α(n)) |
| `reset(n)` | 全情報を捨て、n 個の単独成分に戻す | O(n) |

`groups()` は各成分内を昇順、成分同士を最小頂点番号の昇順で返します。空の DSU は頂点数・成分数が0、成分一覧が空です。自己ループや同じ辺の再追加では成分数を減らしません。

## 原理

各成分を親へのポインタからなる木で表します。代表元同士だけを接続し、小さい成分を大きい成分の下へ付けます。探索した頂点を代表元へ直接つなぎ直す経路圧縮も使います。

サイズは根に保持し、異なる成分の併合が成功したときだけ成分数を1減らします。代表元は最小頂点とは限らず、併合で変わる場合があります。代表元を固定した番号として保存せず、必要なときに `leader(v)` を呼びます。

図の親ポインタは DSU 内部の木であり、元の入力グラフの辺とは限りません。[操作できる図](../../visualizations/union_find/index.html#ordinary) では併合前後・経路圧縮・成分サイズ／個数を確認できます。

## 使う問題

- [Library Checker: Unionfind](https://judge.yosupo.jp/problem/unionfind)：辺追加と連結判定。
- [AtCoder Library Practice Contest A](https://atcoder.jp/contests/practice2/tasks/practice2_a)：通常の Union-Find。
- Kruskal 法による最小全域森、辺追加のみの連結性管理、成分サイズを使う組合せ計算。

削除・Undo・永続化・成分の頂点重み和はこの S ランクの実装の対象外です。

## 互換性

以前の `data_structure/disjoint_set_union.hpp` と `data_structure/UnionFind.hpp` は同じ実装を読み込みます。`root`、`issame`、`unite`、旧名 `unionfind` も残しています。

`merge` の戻り値は従来どおり bool です。ACL の代表元を返す `merge` とは違うので、代表元は `leader` で取得してください。互換用の公開配列 `par` / `siz` は直接変更しません。変更すると成分数などの整合性が失われます。

## 検証

`python3 verify/run_union_find.py`。小規模な併合履歴の全探索、ラベル全置換の愚直実装との比較、再初期化、空入力、重複併合、20万要素、旧 API と Kruskal への組み込みを検証します。

[ジャッジ形式のプログラム](../../verify/union_find/unionfind.test.cpp) も用意しています。ローカルでの検証と外部ジャッジの AC は区別し、外部への提出はまだ行っていません。

2026-10-24にmerge(u,v,callback)を追加しました。[呼出し順・例外・付随情報の契約](rollback_disjoint_set_union_with_data.md)。旧2引数mergeは保持します。
