# 3.1〜3.6 A項目の補完

未実装・一部対応32掲載箇所を補完しました。既存実装を重複させず、33の公開ヘッダ（共通入口1本を含む）を各カテゴリに追加しています。[実装予定リスト](../implementation_plan.md) / [引継書](../../HANDOFF.md)。

- [3.1 基本ユーティリティ](../basic_utility/README.md)
- [3.2 Union-Find](../union_find/README.md)
- [3.3 セグメント木・Fenwick](../range_query/README.md)
- [3.4 静的クエリ・整数集合](../static_query/README.md)
- [3.5 平衡木・永続構造](../ordered_structure/README.md)
- [3.6 オフライン・二次元](../offline_query/README.md)
- [32モードの操作できる図解](../../visualizations/a_priority/index.html) / [一次資料・問題例・設計判断](references.md)

```sh
python3 verify/run_a_priority.py
python3 verify/run_a_priority.py --sanitize
node verify/visualizations/a_priority_model_test.js
node verify/visualizations/a_priority_browser_test.mjs --browser /path/to/chromium
```

C++17・警告をエラーとして、33ヘッダの単独/重複include（debug/release）、逆順/6カテゴリ混在include、複数翻訳単位のリンク、33使用例とMarkdown usageの一致、6ジャッジ形式×24バッチ×300操作を確認します。5本の差分検証をdebug/releaseで実行します。`--sanitize`はASan/UBSan付きで同じ実行検証を行います。

- 基本: n<=8の全順列、n<=8の全(S,T⊆S)、小範囲の商/倍数、全1024区間集合、循環添字、64bit境界。
- DSU: 60履歴で過去時刻/分岐の全頂点対・サイズを素朴ラベル配列と比較。成分集約、履歴木、二部性のBFS、20万頂点。
- 区間: n=0..65の全DST区間、文字列順、アフィン合成と素朴配列、LLONG_MAX長の疎な木、100格子と二次元3方式、座標極値/不正登録。
- 順序: Treap/RBST/OrderedSetとstd::multiset/set、BinaryTrieとソート列、64/4096境界のFastSet、1万回の列操作、非可換な反転と作用、4万永続分岐、10万要素Queue。
- オフライン: 3種類のブロック幅でRMQとRollback Mo、1万RangeSet編集、3万木パスを独立BFSと比較、森/空入力/20万頂点のパス。

オンラインACは未確認です。既存カテゴリのランナーは従来のS項目と先行追加分を検証し、今回の追加分は上記の専用ランナーでまとめて検証します。旧2D SegtreeのWIPは変更せず、新しい公開APIを説明から案内します。

今回の確認結果：通常・ASan/UBSan/LeakSanitizerの全検証、既存5カテゴリの回帰、全26カテゴリall.hppの混在コンパイルが成功。図解モデルは2,586入力・87,734状態、ブラウザは全32モードと390px表示が成功。関連索引を含むローカルリンク1,162件も確認しました。
