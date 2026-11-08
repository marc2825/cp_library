# 共通基盤 F-08〜F-10の図解

[図を開く](index.html) / [API・使用例・検証](../../docs/foundation_structures/README.md) / [図解一覧](../index.html)

JSONを編集し、先頭・戻る・次へ・最後・自動再生で内部状態を追えます。各モードにサンプル復元と説明へのリンクがあります。

| モード | 表示する状態 | 図解の範囲 |
|---|---|---|
| seeded_hash | seed、各要素のhash、結合途中の状態 | 符号付き64bit整数、tuple、sequence。C++の文字列・128bit対応は対象外。 |
| hash_map | 空・占有・削除済みslot、探索先、再構築 | 整数キー／値の挿入・更新・検索・削除。constantHashで全キーを衝突させる。 |
| csr_graph | 次数、offsets、arc配列、辺IDとrev | 有向／無向、自己ループ、多重辺。辺コストはJavaScriptの安全な整数。 |
| monoid_catalog | 左からの結合と中間状態 | min/max＋個数、非空最大部分配列、2×2行列、affine、一次分数、hash。 |
| polynomial_add_sum | 絶対添字、各冪のmoment、更新量、区間和 | 1回の多項式加算。次数0〜5、20要素以内。 |

Monoidの行列・関数・一次分数・hashは法101、hashのbaseは7です。`kind`を`min`・`max`・`subarray`・`matrix`・`affine`・`fractional`・`hash`に変えて比較できます。行列／一次分数の要素は`[a,b,c,d]`、affineは`[a,b]`、ほかは整数文字列です。関数の評価点は`x`、一次分数の無限遠点は`null`で指定します。行列は左×右、関数は左を適用してから右という順序を表示します。

このJavaScriptモデルはC++をブラウザ上で実行するものではありません。Hash Mapのreserve・コピー・例外保証はC++側で検証します。多項式加算はBigIntを使い、C++の固定幅整数のoverflowを再現しません。木の遅延伝播は省略し、集約値と作用の関係を示します。

## 検証

```sh
node verify/visualizations/foundation_structures_model_test.js
node verify/visualizations/foundation_structures_browser_test.mjs --browser /path/to/chromium
python3 verify/run_foundation_structures.py
```

5モードのサンプルをC++の出力と照合します。モデルはMap・隣接リスト・素朴な配列更新・部分配列の全探索による1,200比較、7種類のMonoid、無効入力を検証します。ブラウザでは全モード、入力変更・エラー・復元、再生・移動、ローカルリンク、390px表示を検証します。画像の保存は`--screenshot /tmp/foundation-structures.png`で指定できます。
