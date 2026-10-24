# 木の追加項目の図解

[index.html](index.html) は外部通信・依存なしで開ける7モードです。入力JSONを編集し、前後・先頭・最後・再生で操作を追えます。SVGは森と対象頂点、太線はspine、表はrake/compress・作用範囲・木DPの集計を示します。

動的木は意味を確認する小規模モデルです。各操作時にpreferred-pathの分解を再構築し、C++のsplay回転列・実際のlazy tag配置は再現しません。更新前後・作用順・spineとoff-spineの集合・link/cut後の木DPを比較できます。状態は操作ごとのスナップショットであり、再生中に入力を実行し直すと先頭から作り直します。

nは16以下（中心のみ20以下）、操作は40以下。整数の値、`path_apply(u,v,a,b)` / `subtree_apply(root,v,a,b)` は `x -> a*x+b` を表します。DPの独立集合はBigIntで計算します。Prüferはnを明示し、中心は非負整数重み、根変更区間はHLD順を表示します。n=0はC++では一部対応しますが図解は1以上に制限します。

```sh
node verify/visualizations/tree_extensions_model_test.js
node verify/visualizations/tree_extensions_browser_test.mjs --browser /path/to/chromium
```

[公開API・検証](../../docs/tree_extensions/README.md) / [一次資料](../../docs/tree_extensions/references.md) / [全図解](../index.html)。
