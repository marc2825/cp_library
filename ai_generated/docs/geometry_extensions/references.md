# H追加項目の一次資料・設計判断

2026-10-28確認。数学的定義・退化条件を参照し、既存の共通部品を用いて独立に実装しています。今回、外部実装のソースを新たに移入していません。

| 対象 | 一次資料 | 採用した範囲 |
|---|---|---|
| 厳密predicate | [Shewchuk原論文・解説](https://www.cs.cmu.edu/~quake/robust.html) | 行列式の丸めで位相判定が壊れる問題。今回はexpansion arithmeticの移植ではなく、外向きinterval＋BigRationalのdyadic fallback |
| Delaunay / Voronoi | [CGALの公式仕様](https://doc.cgal.org/latest/Triangulation_2/) | 空円条件、共円での非一意性、点挿入とflip、双対。CGALの実装や複雑度をそのまま転用しない |
| 平面埋め込み | [CGAL 2D Arrangements](https://cgal.geometryfactory.com/CGAL/doc/main/Arrangement_on_surface_2/index.html) | 唯一の外面、外側CCB、穴・孤立点を含む面。直線辺に限定して厳密な包含で境界をまとめる |
| 全頂点の最遠点 | [Aggarwalほか原論文](https://hcsoso.github.io/teaching/topology-f21/papers/SMAWK.pdf)、[CGAL公式仕様](https://doc.cgal.org/Manual/latest/doc_html/cgal_manual/Polytope_distance_d_ref/Function_all_furthest_neighbors_2.html) | 行最大値の単調性を使う。今回はSMAWKではなくO(n log n)の分割統治 |
| 矩形和集合の問題 | [Library Checker公式](https://judge.yosupo.jp/problem/area_of_union_of_rectangles) | N≤500,000、各矩形l d r u、0≤座標≤10^9。独立格子比較を伴うローカル入出力検証。オンラインACは未確認 |
| 3D凸包 | [CGAL公式仕様](https://cgal.github.io/9465/doc/Convex_hull_3/index.html) | 空・点・線分・共面・立体の次元を区別。今回は任意精度の増分可視面法 |

円の接線・三角形の中心・面積・重心は旧geometry/2D_geometry_template.hppの機能と数式を照合し、新Circle/Toleranceと明示的な退化結果へ整理しました。旧Orthocenterの未検証式や円・多角形交差の改善注記をそのまま移していません。外接円は既存MECのthrough_threeを共有しています。

矩形の被覆長と連結区間数、円弧のGreen積分、凸領域のfloor-sum、耳切断は定義に基づいて実装し、独立な格子・積分・面積・三角形条件で比較します。任意精度演算のbit費用と幾何操作回数を区別します。
