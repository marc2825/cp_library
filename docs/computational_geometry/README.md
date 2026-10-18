# 3.22 計算幾何

[ライブラリ一覧](../../README.md) / [全図解](../../visualizations/index.html) / [一次資料・設計判断](references.md)

入口は[all.hpp](../../computational_geometry/all.hpp)。整数の厳密計算は`cp::geometry::exact`、浮動小数点計算は`cp::geometry::floating`に分離しています。旧`geometry/2D_geometry_template.hpp`は保持しています。

| 優先度 | API・usage | 計算量 | 図解 |
|---|---|---|---|
| S | [Point / Vector・内積・外積・CCW](point_vector.md) | O(1) | [操作](../../visualizations/computational_geometry/index.html#vectors) |
| S | [線分交差判定](segment_intersection.md) | O(1) | [操作](../../visualizations/computational_geometry/index.html#segments) |
| S | [直線交点](line_intersection.md) | O(1) | [操作](../../visualizations/computational_geometry/index.html#lines) |
| S | [射影・距離](projection_distance.md) | O(1) | [操作](../../visualizations/computational_geometry/index.html#projection) |
| S | [Convex Hull](convex_hull.md) | O(n log n) 時間・O(n) 追加領域 | [操作](../../visualizations/computational_geometry/index.html#hull) |
| A | [点の偏角ソート](angular_sort.md) | O(n log n) 時間・O(n) 追加領域 | [操作](../../visualizations/computational_geometry/index.html#angle) |
| A | [Polygon Area](polygon_area.md) | O(n) 時間・O(1) 追加領域 | [操作](../../visualizations/computational_geometry/index.html#area) |
| A | [Point in Polygon](point_in_polygon.md) | O(n) 時間・O(1) 追加領域 | [操作](../../visualizations/computational_geometry/index.html#inside) |
| A | [Convex Polygon Contains](convex_polygon_contains.md) | 構築 O(n log n)・問い合わせ O(log h)・保持 O(h) | [操作](../../visualizations/computational_geometry/index.html#convex) |
| A | [Convex Polygon Cut](convex_polygon_cut.md) | O(h) 時間・O(h) 出力領域 | [操作](../../visualizations/computational_geometry/index.html#cut) |
| A | [Rotating Calipers・凸多角形の直径](rotating_calipers.md) | O(h) 時間・O(1) 追加領域 | [操作](../../visualizations/computational_geometry/index.html#calipers) |
| A | [最近点対](closest_pair.md) | O(n log n) 時間・O(n) 追加領域 | [操作](../../visualizations/computational_geometry/index.html#closest) |

## 数値と境界の契約

- C++17。整数版はGCC/Clangの`__int128`とoverflow検出組込み関数を利用します。外部ライブラリは不要ですが、この拡張がないコンパイラは対象外です。
- 整数Pointの座標はint64_t、外積・内積・距離の二乗は符号付き128ビットWide。有理数Rationalの分子・正の分母もWideです。**任意精度ではありません。すべての途中演算が型に収まることが条件**で、超過はoverflow_errorです。最終結果が収まっても、途中の積や約分前の和・分母が超えれば拒否します。int64_tの全座標組合せが使えるという保証ではありません。
- Rationalは約分して保持し、積では交差約分、比較では連分数を使って不要な積を避けます。分母0・0除算はdomain_error。Pointの加減乗の結果座標はint64_tの範囲を検査します。失敗時の戻り値を返すAPIではありません。
- 交点・射影・切断はRationalPointで返し、整数へ丸めません。整数版の距離は二乗のみ。浮動小数点への移行は`conversion.hpp`の`to_floating`や`Rational::approximate()`で明示します。
- 浮動小数点はlong doubleで、入力と公開フィールドは有限値が前提です。非有限入力はinvalid_argument、演算で非有限になるとoverflow_error。丸めとアンダーフローは検出しきれず、差・積・距離が数値的に区別できる尺度で使ってください。
- `floating::Tolerance(epsilon=1e-10L)`は有限・非負の**絶対誤差**です。外積、内積、距離二乗、座標の各判定へ同じ値を使うので、相対誤差や座標尺度に不変な保証はありません。0も指定できます。EPSを変えても演算自体が厳密になるわけではありません。
- ソートの`<`、重複除去の`==`は浮動小数点でもEPSを使いません。近似等値は別関数。偏角の浮動小数点版はatan2の数値順です。最近点対と直径の計量にもEPSを入れません。
- 線分は閉集合で点に退化してもよく、直線は異なる二点が必要です。直線交点は一意・平行・同一直線を区別。`Location`はoutside=0、boundary=1、inside=2です。
- 凸包は既定でstrict・CCW・最小(x,y)から開始し、閉点の再掲なし。freeの凸包含・直径はその前提を問い合わせごとには検査しません。切断は有向直線の左側を境界込みで残します。
- 添字はsize_t・0始まり。最近点対は元入力の添字、直径は凸包の添字です。空・一点・全共線・同一点の規約は各説明に記載しています。

## ローカル検証

```sh
python3 verify/run_computational_geometry.py
python3 verify/run_computational_geometry.py --sanitize
node verify/visualizations/computational_geometry_model_test.js
node verify/visualizations/computational_geometry_browser_test.mjs --browser /path/to/chromium
```

`--compiler clang++`で切替可能。17ヘッダの単独・重複includeをdebug/releaseで確認し、混在includeと複数翻訳単位、12使用例、12ジャッジ形式・768ケースを実行します。Python Fractionの任意精度演算2,244ケースで四則・直線交点・射影・距離・凸包・偏角・切断・包含・最近／最遠距離を比較します。オンラインACは未確認です。

C++では18,496組の有理数、729組のCCW、6,561組の線分、512点集合の凸包と12,800包含クエリ、300切断、600点群の全点対を確認。128ビットの上下限と拒否条件、EPSを使わないソート・最近距離も含みます。20万点の最近点対と20,001頂点のキャリパーは演算回数の上限も確認します。通常・ASan/UBSanのdebug/releaseが対象です。

図解は12モード。整数版はBigInt有理数、浮動小数点版はJavaScript Numberを使います。C++の固定128ビットやlong doubleの再現ではなく、小さな入力で境界・途中状態を追う教材です。点・多角形・問い合わせとEPSを変更できます。

図解モデルは5,016入力・38,475スナップショット、13拒否条件を独立解で検証します。

## 3.21〜3.25 B項目

- [Minimum Enclosing Circle](minimum_enclosing_circle.md) / [操作できる図解](../../visualizations/b_21_25/index.html#minimum_enclosing_circle)
- [Circle-Line / Circle-Circle Intersection](circle_intersection.md) / [操作できる図解](../../visualizations/b_21_25/index.html#circle_intersection)
- [Convex Layers](convex_layers.md) / [操作できる図解](../../visualizations/b_21_25/index.html#convex_layers)
- [Minkowski Sum](minkowski_sum.md) / [操作できる図解](../../visualizations/b_21_25/index.html#minkowski_sum)
- [Farthest Pair](farthest_pair.md) / [操作できる図解](../../visualizations/b_21_25/index.html#farthest_pair)
- [Manhattan MST（共通入口）](manhattan_mst.md) / [操作できる図解](../../visualizations/b_21_25/index.html#manhattan_mst)
- [格子点数（Pickの定理）](pick.md) / [操作できる図解](../../visualizations/b_21_25/index.html#pick)
- [半平面交差（明示的な境界矩形内）](half_plane_intersection.md) / [操作できる図解](../../visualizations/b_21_25/index.html#half_plane_intersection)

[全28項目のAPI・条件・比較検証](../b_21_25/README.md)。`python3 verify/run_b_21_25.py` / `--sanitize` / `make verify-b-21-25`。
