# 参考資料・設計判断

2026-10-15 に次の一次資料・著者自身の実装解説を確認しました。一般的なアルゴリズムの考え方と API を参考にし、このカテゴリのコード・説明は本ライブラリの契約に合わせて記述しています。外部ライブラリのコードを転載したものではありません。

| 参照先 | 参考にした点 | この実装での扱い |
|---|---|---|
| [Luzhiled: Mo](https://ei1333.github.io/library/other/mo.hpp.html) | 左右 4 処理・共通 2 処理、元クエリ ID、ブロック順 | `Mo(n)`＋登録、空区間可、処理順を取得可、snake 順、幅を任意指定 |
| [CP-Algorithms: Mo](https://cp-algorithms.com/data_structures/sqrt_decomposition.html#mos-algorithm) | 区間の移動とブロック幅の考え方 | 自動幅を N/√Q にして、ソート・回答処理も計算量へ記載 |
| [robert1003: Parallel Binary Search](https://robert1003.github.io/2020/02/05/parallel-binary-search.html) | 単調な複数探索で更新処理を共有する応用例 | 著者の再帰方式に対し、reset/apply/check のラウンド・バケツ方式。答えは 0..M と M+1 |
| [robert1003: CDQ](https://robert1003.github.io/2020/01/31/cdq-divide-and-conquer.html) | 左の寄与を右に加える枠組み、Mokia の時間＋二次元和、BIT の巻き戻し | 半開長方形、厳密比較、座標圧縮、共有添字バッファ。境界 −1 を使わない |
| [LC: Static Range Inversions Query の仕様](https://github.com/yosupo06/library-checker-problems/blob/master/data_structure/static_range_inversions_query/task.md) | 区間の転倒数 | 左右別 Mo のジャッジ形式、独立 Python 解で比較 |
| [LC: Rectangle Sum の仕様](https://github.com/yosupo06/library-checker-problems/blob/master/data_structure/rectangle_sum/task.md) | 静的重み付き点と半開長方形 | Sweep Line のジャッジ形式。問題 ID は rectangle_sum |
| [LC: Point Add Rectangle Sum の仕様](https://github.com/yosupo06/library-checker-problems/blob/master/data_structure/point_add_rectangle_sum/task.md) | 点追加と半開長方形の入力形式 | CDQ のジャッジ形式。読み終えて一括 solve |
| [AtCoder: AGC002 D](https://atcoder.jp/contests/agc002/tasks/agc002_d) | 追加済み辺による二成分の和集合サイズ | 並列二分探索＋既存 Union-Find。Python BFS で比較 |

座標圧縮は [3.5](../ordered_structure/coordinate_compression.md)、Fenwick Tree は [3.3](../range_query/fenwick_tree.md) の本リポジトリ実装を利用します。各カテゴリに元の参考資料・ライセンス情報を記載しています。

汎用エンジンだけでは使う際の条件が見えにくいため、Mo／並列二分探索にはコールバックを、Sweep Line／CDQ には競技問題にそのまま適用できる長方形和を用意しました。高度な CDQ DP、長方形面積、オンライン二次元データ構造は別の機能として追加できます。図解はアルゴリズムの主要ステップを再現し、C++ の各命令と 1 対 1 の対応を主張しません。

ジャッジ URL を付けたプログラムはローカル検証用です。オンライン AC と書く際には、実際の提出結果を別途確認してください。[検証方法](README.md)。
