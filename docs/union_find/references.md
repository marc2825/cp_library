# 3.2 Union-Find・連結性管理：参考資料と設計

2026-10-15 に確認した一次資料です。汎用アルゴリズムと API の設計を参考に、このライブラリの既存 API・カテゴリ構成に合わせて実装しています。

| 資料 | 確認した観点 |
|---|---|
| [ACL DSU の仕様](https://atcoder.github.io/ac-library/production/document_en/dsu.html)・[実装](https://github.com/atcoder/ac-library/blob/master/atcoder/dsu.hpp) | 経路圧縮、サイズによる併合、代表元・成分サイズ・成分一覧 |
| [Luzhiled: Union Find](https://ei1333.github.io/library/structure/union-find/union-find.hpp) | bool の併合結果と、find/same/size/groups の構成 |
| [Nyaan: Union Find](https://nyaannyaan.github.io/library/data-structure/union-find.hpp) | 短い通常版と成分サイズの保持 |
| [maspypy: Potentialized UnionFind](https://maspypy.github.io/library/ds/unionfind/potentialized_unionfind.hpp) | 根間の相対値、接続方向反転、成分数の更新。群全般の汎用版は今回の可換加法版より広い対象 |
| [hitonanode: Potentialized UnionFind](https://hitonanode.github.io/cplib-cpp/unionfind/potentialized_unionfind.hpp) | p[t] = p[s] + diff の符号、既存制約との矛盾判定、整数以外の型 |
| [noshi91: Potentialized Union Find](https://noshi91.github.io/Library/data_structure/potentialized_union_find.cpp) | 経路圧縮と相対値の更新、代表元・差分・サイズの API |
| [KACTL: UnionFind](https://github.com/kth-competitive-programming/kactl/blob/main/content/data-structures/UnionFind.h) | 短い union by size + path compression の構成 |
| [CP-Algorithms: DSU](https://cp-algorithms.com/data_structures/disjoint_set_union.html) | 計算量、成分情報の保持、根への距離の更新 |
| [Library Checker: Unionfind](https://judge.yosupo.jp/problem/unionfind)・[Potential の問題仕様](https://github.com/yosupo06/library-checker-problems/blob/master/data_structure/unionfind_with_potential/task.md) | 入出力形式、未知の差分、矛盾する情報を拒否する仕様、差分の向き |

## この実装での選択

- カテゴリは `union_find/` に独立させ、長い説明は `docs/union_find/`、操作できる図は `visualizations/union_find/` に分離。
- 通常版の `merge` は既存コードに合わせて bool。ACL の代表元を返す API には変更しない。
- 通常版の既存パスと旧メソッド名を互換入口として保持。重複実装は作らない。
- 成分数は両方のクラスに保持し、実際に異なる成分を併合した時だけ減らす。
- 重み付き版は可換な加法の差分に絞り、整数・modint・XOR を表現する型を扱えるようにする。
- 重み付き版の戻り値を3種類に分け、未連結の差分は optional にする。
- 矛盾する制約は拒否し、以前の正しい制約を保持する。矛盾フラグを成分に蓄積する方式とは区別する。
- 成分一覧を最小頂点の昇順に揃え、説明・可視化・テストを読みやすくする。

Suisen・sotanishy・kmyk の公開索引も確認しましたが、一部の個別ページを取得できなかったため、実装の直接の根拠は上記の取得できた一次資料に限定しています。
