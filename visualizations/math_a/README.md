# 3.12〜3.15 A項目の図解

[index.html](index.html)をブラウザで開きます。外部通信・追加パッケージは不要です。18モードでJSON入力を変更し、最初/戻る/次へ/再生/最後を操作できます。

[実装・使用例](../../docs/math_a/README.md) / [一次資料](../../docs/math_a/references.md) / [共通一覧](../index.html)

D&Cの候補範囲、Knuthのopt、直線に変形したDP、Li Chaoのノード、Slope Trickの折れ点と関数、WQSの罰則/区間数、φ/μのSPF、CRTのlcm、Garnerの係数、floor_sumのEuclid、原始根の位数、Stirling/Bell三角形、五角数、Montmort、Burnsideの固定点、Pólyaのcycleを追います。

## 数値・教材上の範囲

- DPは14要素、D&Cは16段まで、値0..100。二乗距離は高さ-100..100、定数-100..100。単調最適化とWQSの前提を満たす非負区間和の例です。
- CHTモードは追加直線を直接評価し下包絡を可視化します。C++のmultiset内部を再現しません。Li Chaoは登録座標・ノードへの直線挿入を再現します。
- Slope Trickは35操作、折れ点は教材用のソート配列でheapの先頭と移動を表します。グラフは61点のサンプリングで、折れ点の厳密な位置は表で確認します。mergeの公開APIはC++の格子比較で検証します。
- φ/μの篩は100まで。教材用のSPF構築は単純な篩ですが、値の更新式はC++と同じです。単一値の64bit分解は[既存図解](../advanced/index.html#rho)を参照します。
- CRT/Garnerは10法まで。floor_sumはC++と同じn,mの32bit範囲。これらとBurnsideはBigIntを使い、結果を文字列で表示します。64bit整数はJSONの安全な数値か10進文字列で入力します。CRTのprefix lcmとfloor_sumの中間値はC++と同じ範囲を検査します。
- 原始根は素数2000までで試し割りします。C++は既存Miller–Rabin/Pollard–Rhoを使います。
- 数列は35まで、法は1..10^9。Pólyaは10頂点の輪の回転/反転群、色重み5種類まで（-10..10）。C++は任意の全置換群とcycle長ごとの任意の代入を受けます。群の位数が法に対して非可逆ならエラーです。
- Burnsideは固定点数の正確な整数平均。図解は群作用自体を自動検証しません。

```sh
node verify/visualizations/math_a_model_test.js
node verify/visualizations/math_a_browser_test.mjs --browser /path/to/chromium
```

モデルは小さい全探索・任意精度整数・全彩色の軌道代表元などの独立した素朴解と比較します。ブラウザは全18モード、入力編集、エラー復旧、履歴、再生、ローカルリンク、390px表示を確認します。C++の検証とは別に実行します。
