# 参考資料・設計判断

調査日: 2026-10-15。以下の一次資料の本文・実装・問題定義を確認しました。

| 資料 | 参考にした点 |
|---|---|
| [Luzhiled Sparse Table](https://ei1333.github.io/library/structure/others/sparse-table.hpp.html) | 冪等性、2 区間による O(1) の集約 |
| [CP-Algorithms Sparse Table](https://cp-algorithms.com/data_structures/sparse-table.html) | 2 の冪の前計算、重複を許す条件 |
| [CP-Algorithms Minimum Queue](https://cp-algorithms.com/data_structures/stack_queue_modification.html) | 添字付き候補列、論理 FIFO の削除、固定長窓の処理 |
| [Nyaan Slide Window Aggregation](https://nyaannyaan.github.io/library/data-structure/slide-window-aggregation.hpp.html) | 2 スタックの移し替えと左右の集約順序 |
| [Library Checker Static RMQ の問題定義](https://github.com/yosupo06/library-checker-problems/blob/master/data_structure/staticrmq/task.md) | RMQ の半開区間と入出力 |
| [Library Checker Queue Composite の問題定義](https://github.com/yosupo06/library-checker-problems/blob/master/data_structure/queue_operate_all_composite/task.md) | 関数を列の左から順に適用する向き、空列の恒等関数 |
| [AOJ DSL_3_D](https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=DSL_3_D) / [DPL_3_C](https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=DPL_3_C) | 窓最小値、最近傍境界を利用するヒストグラム問題 |

Sotanishy の索引も確認しましたが、個別ページは取得できなかったため実装の参照先としては扱っていません。

## このカテゴリでの選択

コードはこの API 向けに記述しています。参考ライブラリのコード・説明文を一括転載していません。

- Sparse Table はモノイド形式に揃えて空区間を e() にし、空配列の構築・全再構築にも対応します。和を誤って O(1) で求める用途には使いません。
- Monotonic Stack は最近傍を求める**候補スタック**です。CP-Algorithms の「各要素に最小値を付けた通常のスタック」とは用途・契約が異なります。
- 比較器は less/greater を差し替える形式。等値の扱いを文書化し、比較器以外の T の演算子を要求しません。
- 単調キューは論理長と候補数を分け、値を知らずに pop できます。同値は左優先とし、argmin を返します。
- 固定長窓は値と添字の 2 関数を提供し、余分な長さ n の中間配列を作りません。
- SWAG は Segtree と同じ `S,op,e` を用い、非可換演算の順序を保持します。単一操作の最悪 O(n) と、ならし O(1) を区別します。

## 検証の意味

期待値は独立した全区間走査・線形の最近傍走査・素朴な deque・関数の逐次適用で生成しています。ランダムデータだけでなく、重複値のある短い配列の全探索を含みます。assert を無効にした C++ release でも、テスト自体の CHECK は有効です。

図解モデルも別の JavaScript 実装として、素朴解と比較します。可視化のスナップショットは途中の候補削除や移し替えを含みます。次の操作は一つの操作の開始／完了位置から行い、途中なら完了まで進めてから続けます。
