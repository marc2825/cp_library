# 参考資料・設計判断

調査日: 2026-10-15。標準コンテナの契約は C++17 草案の本文、問題仕様はジャッジの定義を確認しています。

| 一次資料 | 確認した内容 |
|---|---|
| [C++17 N4659・連想コンテナ](https://timsong-cpp.github.io/cppwp/n4659/associative.reqmts) | 比較器の同値、lower/upper_bound、erase の各形式、iterator の有効性と計算量 |
| [C++17 N4659・map 要素アクセス](https://timsong-cpp.github.io/cppwp/n4659/map.access) | operator[] の挿入、at の不在時の例外 |
| [C++17 N4659・二分探索アルゴリズム](https://timsong-cpp.github.io/cppwp/n4659/alg.binary.search) | 非ランダムアクセス iterator での線形移動とメンバ検索との違い |
| [KACTL FenwickTree](https://github.com/kth-competitive-programming/kactl/blob/main/content/data-structures/FenwickTree.h) | 累積和を 2 の冪幅で探索する方法 |
| [ACL Fenwick Tree](https://atcoder.github.io/ac-library/production/document_ja/fenwicktree.html) | 一点加算と半開区間の和 |
| [Library Checker Ordered Set の問題定義](https://github.com/yosupo06/library-checker-problems/blob/master/data_structure/ordered_set/task.md) | 重複しない集合、1 始まりの k、以下の個数、前後検索 |
| [Library Checker Predecessor Problem の問題定義](https://github.com/yosupo06/library-checker-problems/blob/master/data_structure/predecessor_problem/task.md) | 集合の更新・存在・以上／以下の検索 |
| [AtCoder ARC033 C](https://atcoder.jp/contests/arc033/tasks/arc033_3) | k 番目を答えて削除する用途 |

## 設計

標準コンテナは独自クラスで覆わず、C++17 の insert/find/try_emplace/insert_or_assign を直接使います。補助するのは境界確認を伴う前後検索、erase_one、先頭／末尾の取り出しです。順序はコンテナの比較器に従います。

座標圧縮と順位管理は別ヘッダです。既存の `range_query/fenwick_tree.hpp` を再利用し、BIT 実装を複製しません。コードはこの API 向けに記述しており、外部ライブラリの実装や説明を一括転載していません。

未登録の挿入を黙って近隣の座標へ割り当てたり、途中で座標を増やして添字を変えたりしません。すべての挿入候補を先に集める契約です。問い合わせの閾値は lower/upper_bound で位置に変換できるため、登録不要です。

頻度を非負に保ち、総数のオーバーフローも更新前に検査します。kth の外れ値は optional とし、整数の -1 を格納できる仕様を保ちます。標準コンテナの検索はコピー不要な iterator、圧縮版は扱いやすい optional<T> です。

## 検証

全探索では各座標の頻度を 0/1/2 とし、順位・k 番目・区間・前後の期待値を独立した線形走査で作ります。ランダム更新もソートした素朴配列と比較します。標準コンテナの補助関数は std::lower_bound 等を期待値に使わず、列の走査と照合します。

ジャッジ形式の期待値は Python の集合とソートで生成します。assert 無効の release でも C++ のテスト自身の CHECK は有効です。図解モデルも独立した列・辞書と比較します。

図解は sorted な並び、重複、map の対応、圧縮後の頻度と BIT を示します。標準コンテナの内部木の形は実装依存なので、実際の赤黒木の状態を再現したものとはしません。

3.1〜3.6のA補完の[一次資料・問題例・設計判断](../a_priority/references.md)。各APIの条件と検証はカテゴリ索引から辿れます。
