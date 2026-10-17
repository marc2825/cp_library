# 3.21〜3.25 A項目の操作できる図解

[図解を開く](index.html) / [全図解](../index.html) / [API・検証一覧](../../docs/final_a/README.md) / [一次資料](../../docs/final_a/references.md)

10モードでJSON入力を変え、最初・前・次・最後・再生を操作できます。SVGの状態遷移・suffix link・係数表・期限・スコアを表示し、各手順の理由と内部状態を確認できます。外部通信・追加パッケージは不要です。

| モード | 追跡する状態 |
|---|---|
| Suffix Automaton | 新しい末尾状態、遷移、clone、suffix link、出現数の集約 |
| Lyndon | i/j/kの比較位置、確定した因子と長さ |
| Longest Common Substring | secondの走査、遷移不能時のsuffix link、一致長と最良区間 |
| 2D Rolling Hash | 二方向のprefix、上・左の除去、左上の追加、二つの法の値 |
| Nim Product | 小さい値のmex、二次拡大の分割と組立て |
| 周期Grundy | mex列、逆順Z値、反復候補、同じM窓の周期証明、巨大添字 |
| Zobrist | 位置×値の乱数表、oldの取消し、newの追加と再現するhash |
| Tabu | 訪問keyの期限、候補の許可・aspiration、悪化手と全体最良 |
| Beam重複除去 | 全候補の安定ソート、keyごとの代表、幅の制限 |
| 差分スコア | moveのdelta、apply後の一時状態、採用またはundo |

## C++との違い

- 教材用の小入力です。文字列は16byte、patternは8byte、グリッドは4行×6列、グラフは8局面、近傍12個、Tabuは12走査、Beamは12段まで。文字列JSONはUTF-8へ変換し、byte配列なら0〜255を直接指定できます。
- Nim値・乱数・hashの64bit演算はBigIntです。安全なJSON整数または10進文字列を受け取ります。Nimの説明モデルは4bitのmex表を使い、最上位3段までの再帰を描きます。C++は8bit表を使います。結果は同じnim積です。
- 2DハッシュはBigIntで二つの素数法を正確に計算します。SVGには長い値を小さく表示し、表で全桁を確認できます。C++の大きい配列・容量制限の例外は再現しません。
- 周期検出は64項まで。入力を`kind:"observed", values:[...], repetitions:3`へ変えると候補検出になります。教材ではZ値を直接比較で作ります。引き算ゲームで証明できない未観測値はvalue:nullと表示します（C++のvalue関数はout_of_range）。
- Zobristのoldを取り消した直後は、一時的に盤面全体のhashではありません。次のnewを加えるまで同じ変更を続けます。
- 探索のscoreは小さい整数、keyは整数の同値類です。Tabuのsize_t期限の飽和とstopコールバックはC++で検証します。Beamは同じ深さだけを統合します。
- 差分スコアは二乗誤差を例にし、rule:"improve"またはmovesと同じ長さのaccept:[true,false,...]で採否を変えます。差分SAの温度・乱数は[既存SAの図解](../heuristic_search/index.html#annealing)とC++の再現検証を参照してください。

## 検証

```sh
node verify/visualizations/final_a_model_test.js
node verify/visualizations/final_a_browser_test.mjs --browser /path/to/chromium
```

3,078入力・32,128状態・14拒否条件を独立解と比較します。各prefixのendposと状態長、全Lyndon分割、共通区間の総当たり、二つの法の直接矩形多項式、nimの単項基底、mex列と窓一致、全盤面のhash再計算、別のkey集約順のBeam、訪問時刻からのTabu許可、apply直後とundo後の全スコアを確認します。スナップショットの独立性も検査します。

ブラウザは10モードの入力変更・手順移動・再生・エラー復帰・相互リンク・390px表示を検証します。`--screenshot /tmp/final-a.png`で画像を保存できます。
