# E/Xの操作できる図解

[開く](index.html) / [API・検証](../../docs/search_parsing_extensions/README.md) / [図解一覧](../index.html)

E全8項目・X全3項目の11モードです。JSONを編集し、履歴・差分・候補・queue・重み・式・受理条件の変化を追います。前後移動、先頭/最後、自動再生、エラーからの復帰を使えます。表には履歴、頂点、候補順位、ASTの子とspan、NFA辺・部分集合・同値類を表示します。線図は頂点位置と辺を示し、遷移記号と向きは表で確認します。

Undoは配列要素の変更、TSPは16頂点の全距離再計算、Beam/Chokudaiは整数状態への加算と二乗距離評価です。C++は一般のState/Move/callbackを受けます。並列図解はrun別の固定seedを順に進める説明モデルで、JavaScript上でthreadを起動しません。C++の実装とテストはstd::threadで同時実行します。

受理表は1〜12bit、整数Aliasの重みは100万以下・32個まで。distributionはalias/without_replacement/normal/exponential/truncatedを指定できます。seedは整数または10進文字列です。SplitMix64はBigIntで同じbit演算を行います。連続変換はNumberを使い、C++との末尾bitの一致は保証しません。

式はASCII256文字、指数0〜128、値4096桁までのBigIntです。C++のparserはより大きい明示予算を選べます。NFA図解は16状態・8記号・100辺、決定化は128状態まで。regexは96文字、NFA256状態、括弧32段、word64文字まで。UTF-8をbyte列に変換して受理を調べます。X-02の図解は決定化・Moore最小化と受理を扱い、積/補集合/桁DPはC++の検証で確認します。

探索は深さ8・幅16・move4種、Chokudaiは展開1000回・生成2000個、全体のtraceは12000手順まで。数値配列は絶対値100万までです。表示限界はC++ APIの上限と異なります。

[サンプル参照値](../../verify/search_parsing_extensions/visualization_expected.json)はC++例と共用します。モデルは素朴な集合・巡回路・全探索・式・正規表現・NFA経路探索と比較し、ブラウザは全モードのサンプルと変更入力・操作・390px表示を検証します。

```sh
node verify/visualizations/search_parsing_extensions_model_test.js
node verify/visualizations/search_parsing_extensions_browser_test.mjs --browser /path/to/chromium --screenshot /tmp/search-parsing.png
```
