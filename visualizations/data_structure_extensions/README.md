# D-02〜D-10 の操作できる解説

[図解を開く](index.html) / [API・検証一覧](../../docs/data_structure_extensions/README.md) / [すべての図解](../index.html)

9つのモードを選び、JSON の値・区間・操作列を変更できます。先頭/前/次/最後、再生、サンプルの復元に対応します。配列は32要素、操作は40個、時間軸は16時刻・16頂点まで。整数は原則 ±100万までの JavaScript の安全整数です。

- 両端優先度キューは比較順の論理集合を表示します。std::multiset の赤黒木内部を描くものではありません。
- MEX は prefix ごとの最終出現位置と二分木での探索、Majority は候補と相殺票・実頻度を表示します。
- 点更新頻度は更新経路と標準区間の頻度表を表示します。map の平衡化は省略します。
- 永続 heap は最小優先（C++ の greater に相当）。ノード ID・左右の子・rank・新旧 root を扱い、コピーを追います。
- 動的セグ木は永続版の論理的な点集合を表示します。座標保持と重複点の加算を扱い、shared_ptr の参照数・ノード確保自体は再現しません。入力操作の各 set/merge は新しい版、split は2つの新しい版を末尾に追加します。
- 標準ノード分解は完全二分木の番号と採用区間を表示します。
- 時間軸は有効辺を標準ノードへ登録し、入場/訪問/rollback を追います。汎用 C++ API の具体例として DSU を使います。
- 付随情報の rollback は各根の和を callback で更新する例です。JavaScript の保存点は状態のコピーで表し、C++ の履歴領域や token 検証の再現はしません。

C++ と JavaScript の9つのサンプル結果を共通 fixture で照合します。モデル検証では別の素朴解とのランダム比較・境界・不正入力も確認します。

```sh
node verify/visualizations/data_structure_extensions_model_test.js
node verify/visualizations/data_structure_extensions_browser_test.mjs --browser /path/to/chromium --screenshot /tmp/data-structures.png
```

ブラウザ検証は9モード、5つの変更入力、移動・再生・エラー復帰、ローカルリンク、390px 幅を確認します。C++ の検証は `make verify-data-structure-extensions`。
