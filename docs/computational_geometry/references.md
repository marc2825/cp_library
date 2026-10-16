# 一次資料・設計判断

[カテゴリ索引](README.md) / [図解](../../visualizations/computational_geometry/index.html)

2026-10-17に確認。アルゴリズムの説明とジャッジ仕様を参考に、実装はこのリポジトリ向けに記述しました。外部コードの実行依存はありません。

| 一次資料 | 参考にした点 |
|---|---|
| [CP-Algorithms: Convex hull](https://cp-algorithms.com/geometry/convex-hull.html) | 単調鎖、共線点を残す場合、全共線の処理 |
| [CP-Algorithms: Point in convex polygon](https://cp-algorithms.com/geometry/point-in-convex-polygon.html) | 先頭からの扇形二分探索、辺上判定 |
| [CP-Algorithms: Nearest points](https://cp-algorithms.com/geometry/nearest_points.html) | y順マージと帯の候補を用いる分割統治 |
| [AOJ CGL_1_C](https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=CGL_1_C) | CCWの5分類と文字列出力 |
| [AOJ CGL_4_A](https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=CGL_4_A) | 凸包の境界点保持、最小(y,x)の開始点 |
| [AOJ CGL_4_C](https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=CGL_4_C) | 有向直線の左半平面を残す切断 |
| [Library Checker: Sort Points by Argument仕様](https://github.com/yosupo06/library-checker-problems/blob/master/geo/sort_points_by_argument/task.md) | (−π,π]の出力順と原点 |
| [Library Checker: Closest Pair仕様](https://github.com/yosupo06/library-checker-problems/blob/master/geo/closest_pair/task.md) | 複数テスト、元の異なる添字を返す |
| [Library Checker: Furthest Pair仕様](https://github.com/yosupo06/library-checker-problems/blob/master/geo/furthest_pair/task.md) | 最遠点対の元添字、重複座標 |

## 設計判断

整数・浮動小数点は別のPointとnamespaceにし、暗黙変換を設けません。整数版にEPSはなく、非整数の座標をRationalPointで保存します。固定128ビットの中間値を検査し、範囲外の誤った符号・距離を黙って返さない方針です。任意精度が必要な問題では別の数値型の実装が必要です。

浮動小数点のEPSは判定に限定し、std::sortの比較器へ近似等値を入れません。凸多角形包含は構築とクエリを分け、O(log n)を維持します。直径も凸包の計算を内包せずO(h)とし、最近点対とは返す添字の対象を区別しました。

検証では同じ単調鎖を再実装する代わりにgift wrapping、windingの代わりに有理数の射線交差、回転キャリパー／最近点対の代わりに全点対を使います。交点・射影・切断はPython Fractionの任意精度結果とも比較します。ジャッジ用コードはローカルの形式検証で、オンラインACの記録ではありません。
