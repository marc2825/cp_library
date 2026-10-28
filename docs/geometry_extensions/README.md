# 計算幾何・平面グラフ H全22項目

[統合一覧](../remaining_work.md) / [実装計画](../implementation_plan.md) / [一次資料](references.md) / [22モードの図解](../../visualizations/geometry_extensions/index.html)

H-01〜H-17・H-20〜H-24の22項目を19公開ヘッダ・22使用例・22図解で整備しています。H-18/H-19の行はありません。H-01/H-02はrectangle_union、H-06/H-07/H-08はcircle_areaを共有します。

| ID | API・説明 | ヘッダ | 図解 |
|---|---|---|---|
| H-01 | [長方形の和集合面積](../computational_geometry/rectangle_area.md) | [rectangle_union.hpp](../../computational_geometry/rectangle_union.hpp) | [図解](../../visualizations/geometry_extensions/index.html#rectangle_area) |
| H-02 | [長方形の和集合周長](../computational_geometry/rectangle_perimeter.md) | [rectangle_union.hpp](../../computational_geometry/rectangle_union.hpp) | [図解](../../visualizations/geometry_extensions/index.html#rectangle_perimeter) |
| H-03 | [円の接線](../computational_geometry/circle_tangents.md) | [circle_tangents.hpp](../../computational_geometry/circle_tangents.hpp) | [図解](../../visualizations/geometry_extensions/index.html#circle_tangents) |
| H-04 | [三角形の外接円・内接円・垂心](../computational_geometry/triangle_centers.md) | [triangle_centers.hpp](../../computational_geometry/triangle_centers.hpp) | [図解](../../visualizations/geometry_extensions/index.html#triangle_centers) |
| H-05 | [多角形の重心](../computational_geometry/polygon_centroid.md) | [polygon_centroid.hpp](../../computational_geometry/polygon_centroid.hpp) | [図解](../../visualizations/geometry_extensions/index.html#polygon_centroid) |
| H-06 | [円と多角形の共通部分面積](../computational_geometry/circle_polygon_area.md) | [circle_area.hpp](../../computational_geometry/circle_area.hpp) | [図解](../../visualizations/geometry_extensions/index.html#circle_polygon_area) |
| H-07 | [2円の共通部分面積](../computational_geometry/circle_intersection_area.md) | [circle_area.hpp](../../computational_geometry/circle_area.hpp) | [図解](../../visualizations/geometry_extensions/index.html#circle_intersection_area) |
| H-08 | [円の和集合面積](../computational_geometry/circle_union_area.md) | [circle_area.hpp](../../computational_geometry/circle_area.hpp) | [図解](../../visualizations/geometry_extensions/index.html#circle_union_area) |
| H-09 | [平面埋め込みの面列挙・双対](../computational_geometry/planar_embedding.md) | [planar_embedding.hpp](../../computational_geometry/planar_embedding.hpp) | [図解](../../visualizations/geometry_extensions/index.html#planar_embedding) |
| H-10 | [有理凸領域の格子点・整数凸包](../computational_geometry/rational_lattice.md) | [rational_lattice.hpp](../../computational_geometry/rational_lattice.hpp) | [図解](../../visualizations/geometry_extensions/index.html#rational_lattice) |
| H-11 | [原点共有矩形の増分和集合](../computational_geometry/origin_rectangle_union.md) | [origin_rectangle_union.hpp](../../computational_geometry/origin_rectangle_union.hpp) | [図解](../../visualizations/geometry_extensions/index.html#origin_rectangle_union) |
| H-12 | [線分sweep・交点列挙・Arrangement](../computational_geometry/segment_arrangement.md) | [segment_arrangement.hpp](../../computational_geometry/segment_arrangement.hpp) | [図解](../../visualizations/geometry_extensions/index.html#segment_arrangement) |
| H-13 | [凸多角形の共通部分](../computational_geometry/convex_polygon_intersection.md) | [convex_polygon_intersection.hpp](../../computational_geometry/convex_polygon_intersection.hpp) | [図解](../../visualizations/geometry_extensions/index.html#convex_polygon_intersection) |
| H-14 | [適応精度・厳密幾何判定](../computational_geometry/robust_predicates.md) | [robust_predicates.hpp](../../computational_geometry/robust_predicates.hpp) | [図解](../../visualizations/geometry_extensions/index.html#robust_predicates) |
| H-15 | [三角形内の点数クエリ](../computational_geometry/triangle_point_count.md) | [triangle_point_count.hpp](../../computational_geometry/triangle_point_count.hpp) | [図解](../../visualizations/geometry_extensions/index.html#triangle_point_count) |
| H-16 | [凸多角形の全頂点の最遠点](../computational_geometry/convex_farthest_neighbors.md) | [convex_farthest_neighbors.hpp](../../computational_geometry/convex_farthest_neighbors.hpp) | [図解](../../visualizations/geometry_extensions/index.html#convex_farthest_neighbors) |
| H-17 | [一般・凸多角形の和集合面積](../computational_geometry/polygon_union_area.md) | [polygon_union_area.hpp](../../computational_geometry/polygon_union_area.hpp) | [図解](../../visualizations/geometry_extensions/index.html#polygon_union_area) |
| H-20 | [Delaunay三角形分割](../computational_geometry/delaunay.md) | [delaunay.hpp](../../computational_geometry/delaunay.hpp) | [図解](../../visualizations/geometry_extensions/index.html#delaunay) |
| H-21 | [Voronoi図](../computational_geometry/voronoi.md) | [voronoi.hpp](../../computational_geometry/voronoi.hpp) | [図解](../../visualizations/geometry_extensions/index.html#voronoi) |
| H-22 | [動的凸包](../computational_geometry/dynamic_convex_hull.md) | [dynamic_convex_hull.hpp](../../computational_geometry/dynamic_convex_hull.hpp) | [図解](../../visualizations/geometry_extensions/index.html#dynamic_convex_hull) |
| H-23 | [多角形の三角形分割](../computational_geometry/polygon_triangulation.md) | [polygon_triangulation.hpp](../../computational_geometry/polygon_triangulation.hpp) | [図解](../../visualizations/geometry_extensions/index.html#polygon_triangulation) |
| H-24 | [三次元幾何](../computational_geometry/geometry3d.md) | [geometry3d.hpp](../../computational_geometry/geometry3d.hpp) | [図解](../../visualizations/geometry_extensions/index.html#geometry3d) |

既存BigInt/BigRational、幾何の凸包共通処理、Circle/Tolerance、MECの外接円計算を共有しています。旧geometry/2D_geometry_template.hppと128bit exact APIは保持しています。新しいrational kernelは全演算を任意精度で行い、交点・平面グラフ・多角形・Delaunay/Voronoiから共用します。

H-10は有理凸領域の格子点数と整数凸包、H-24は3Dの点・直線・平面・球、距離と交差、凸包・体積としてユーザー承認済みです。H-10の計数は列挙を避けますが、整数凸包は整数x列の予算を指定します。線分sweepはbounding-box候補列挙、Delaunayは二次時間の挿入法、動的凸包はブロック方式、三角形計数はKD-treeの最悪線形queryです。最先端の計算量を一律に主張せず、各APIの上界を明記しています。

## 検証

```sh
make verify-geometry-extensions
python3 -m tools.cp_tools.suite --runners geometry_extensions
python3 -m tools.cp_tools.suite --stdlib-debug --runners geometry_extensions
python3 -m tools.cp_tools.suite --sanitize --runners geometry_extensions
node verify/visualizations/geometry_extensions_model_test.js
node verify/visualizations/geometry_extensions_browser_test.mjs --browser /path/to/chromium
```

- [面積・円](../../verify/geometry_extensions/measure_test.cpp)：300矩形集合を単位格子と比較、5,000増分追加、int64極値、接線の接点・法線、三角形の中心、独立な縦切片の数値積分、大きな平行移動と円全周の桁落ち対策。
- [位相・有理領域](../../verify/geometry_extensions/topology_test.cpp)：150線分集合の交点分割とEuler検査、入れ子の面・穴・橋・孤立点、100有理凸領域の格子点全列挙、巨大座標のfloor-sum、和集合・共通部分・耳分割。
- [索引・三角形分割](../../verify/geometry_extensions/index_test.cpp)：200凸包の全最遠点、6,000三角形問い合わせ、6,000動的更新と512点の連続削除・再配分、150 Delaunay/Voronoiの空円・面積・nearest条件、共円・共線・極小determinant、int64全域と浮動の極端な指数。
- [3D](../../verify/geometry_extensions/spatial_test.cpp)：凸包の全支持面・辺の接続数・体積、80点集合と平行移動、共面・共線、直線・平面・球の交差。
- 公開ヘッダの単独・重複include、全カテゴリ混在2翻訳単位、22使用例とC++/図解参照値、Python任意精度416ケースの独立比較と矩形和集合のジャッジ形式48入力を検証します。

実行記録は共通suiteから[検証台帳](../../verification/evidence/)へ保存します。オンライン提出AC・GitHub Actionsでの実行は未確認です。

## 今回の実行結果（2026-10-28）

通常・libstdc++ debug mode・ASan/UBSan/LeakSanitizerのDebug/NDEBUGが成功しました。動的凸包の連続削除・再配分と、円の平行移動の修正を含む最終ソースで検証しています。既存の計算幾何と3.21〜3.25 B群の通常回帰も成功。

| runner | 構成 | 時間 | 実行記録 |
|---|---|---:|---|
| geometry_extensions | standard | 121.0秒 | [記録](../../verification/evidence/geometry_extensions/standard.json) |
| geometry_extensions | stdlib-debug | 169.5秒 | [記録](../../verification/evidence/geometry_extensions/stdlib-debug.json) |
| geometry_extensions | sanitize | 226.4秒 | [記録](../../verification/evidence/geometry_extensions/sanitize.json) |
| computational_geometry | standard | 87.6秒 | [記録](../../verification/evidence/computational_geometry/standard.json) |
| b_21_25 | standard | 100.3秒 | [記録](../../verification/evidence/b_21_25/standard.json) |

5記録のfingerprintは `821493cfb791949d61a347765f78bd063daa472128e8d9f79d76cd30cec28623`（1939ファイル）に一致します。実装変更中の記録は無効となるため、最終変更後に取り直しました。

図解は2,327入力・16,283状態・5,014比較、実C++の22参照値、Chromium全22モード・22変更入力・390px表示が成功。Delaunay・円弧・3Dとモバイルの画像を確認しました。関連ローカルリンク、245行の進捗集計、workflow構文、diffも確認しています。ChromiumとLeakSanitizerは既知のsocket/ptrace制限に従い承認レビュー付きで実行しました。オンライン提出AC・GitHub Actions実行は未確認です。
