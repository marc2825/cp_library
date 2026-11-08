# D-20〜D-35の操作できる図解

[図解を開く](index.html) / [API・検証一覧](../../docs/data_structure_advanced/README.md) / [全図解](../index.html)。HTMLを直接開けます。通信・ビルドは不要です。

モードを選び、JSONを変更して「実行」します。「次へ」「戻る」「再生」で制約の伝播、成分、線形値の交差、列と集約、区間の頻度、bit段の安定分割、retroactiveのbridge、boxと距離を追えます。範囲は0-based・半開区間です。ブラウザ用に列60要素・操作200件・安全な整数の範囲へ制限しています。

- D-20は3要素の置換群を扱い、根からの積の順序を表示します。
- D-21は全辺探索による成分比較、D-22は線形値と最初のトーナメントcertificateを表示します。
- D-23/D-32は非可換列・過去版・prefix集約、D-24は商または差分が一様になる条件を表示します。RBSTの乱数形状は再現しません。
- D-25は頻度表、D-26はLISの意味をtails法、D-27は転倒ペアで確認します。C++ではそれぞれブロック分割・Seaweed・Moを使います。
- D-28はbit段ごとの安定分割、D-29は0/1のprefix個数を表示します。JavaScriptのbit幅は1〜20、C++の1〜64全域はC++テストで検証します。
- D-30はdigit/middleの概念配置と版の保持を表示します。C++で選ばれる具体的な形とは異なります。
- D-31は履歴を再生して生存集合を求め、bridge重みを表示します。C++は全再生せずO(log m)で編集します。
- D-33/D-35は半開boxと第1軸Fenwick経路、D-34は移動・削除後の全候補距離を表示します。C++のKD枝刈りとは異なります。

[モデル検証](../../verify/visualizations/data_structure_advanced_model_test.js)は16使用例のC++出力と同じ参照値、全列挙LIS・転倒ペア・配列作用などの独立計算で3,016比較を行います。[ブラウザ検証](../../verify/visualizations/data_structure_advanced_browser_test.mjs)は全16モード・5変更入力・再生・エラー・リンク・390px表示を確認します。
