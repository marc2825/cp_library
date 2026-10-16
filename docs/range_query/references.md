# 参考資料・設計判断

調査日: 2026-10-15。実際に確認した一次資料を以下に記載します。

| 資料 | 参考にした点 |
|---|---|
| [ACL Fenwick Tree](https://atcoder.github.io/ac-library/production/document_ja/fenwicktree.html) | 一点加算と半開区間和の API |
| [ACL Segtree](https://atcoder.github.io/ac-library/production/document_ja/segtree.html) | モノイド、順序保存、左右の境界探索 |
| [ACL Lazy Segtree](https://github.com/atcoder/ac-library/blob/master/document_ja/lazysegtree.md) | 更新の作用、合成方向、遅延伝播の API |
| [Luzhiled Dual Segment Tree](https://ei1333.github.io/library/structure/segment-tree/dual-segment-tree.hpp.html) | 区間更新と一点取得、非可換な更新の境界伝播 |
| [KACTL FenwickTree](https://github.com/kth-competitive-programming/kactl/blob/main/content/data-structures/FenwickTree.h) | 累積和を木上で探索する方法 |
| [CP-Algorithms Fenwick Tree](https://cp-algorithms.com/data_structures/fenwick.html) | 線形構築、差分と 2 本の BIT の原理 |
| [Library Checker 問題定義](https://github.com/yosupo06/library-checker-problems/tree/master/data_structure) | 関数合成の向き、アフィン更新と入出力仕様 |
| [AOJ DSL_2_G](https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=DSL_2_G) | 区間加算・区間和の検証仕様 |

## 実装の出典

`segtree.hpp` と `lazy_segtree.hpp` は AtCoder Library の CC0 実装を改変しています。ローカルで参照した版は `864245a00b00dd008d1abfdc239618fdb7d139da` です。標準ヘッダ以外の依存を除き、C++17 の形に絞り、`cp` 名前空間・型名・`size()`・長さの検査を追加しました。`ll` 等の外部別名は不要です。出典は各ヘッダにも残しています。[ACL の CC0 ライセンス](ACL-LICENSE.txt)。

Fenwick、2-BIT、Dual は各 API の仕様に合わせて実装しています。参考ライブラリの説明文やコードを一括で転載していません。Dual の合成方向は ACL に揃え、更新同士の等値比較を要求しません。KACTL と異なり、Fenwick の非正目標の戻り値は旧 BIT と同じ 0 です。

## 検証の意味

`verify/run_range_query.py` はデータ構造内部の値を期待値に使わず、独立した素朴配列・関数の逐次適用と比較します。ジャッジ形式も Python で期待値を生成します。C++ の assert が無効な release でもテスト自身の検査は有効です。

図解のモデルは別の JavaScript 実装なので、そのモデルも素朴配列と照合します。HTML には簡略化した再帰的な区間分解を使う箇所があり、C++ の反復実装の命令位置と 1 対 1 に対応するものではありません。遅延・区間分解・更新順序の原理は同じです。
