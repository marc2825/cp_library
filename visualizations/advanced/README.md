# 高度なアルゴリズムのインタラクティブ解説

[開く](index.html) / [実装と検証一覧](../../docs/advanced/README.md) / [全図解](../index.html)

21モード。アルゴリズムを選び、JSON入力を変更して「入力を反映」で履歴を再構築します。最初・前後・最後・再生に対応し、各段階の候補・集約・親・履歴・rankを表と配列／グラフの図で表示します。入力変更は常に新しいバッチとして扱い、更新途中の状態から別操作を始めません。外部通信・追加パッケージは不要です。

学習のため次の範囲に簡略化しています。

- 配列16要素・木16頂点・操作40個まで。親配列は根を−1、親番号が子番号未満の森を指定。これは図の入力制限で、C++ APIの制限ではありません。
- Beatsは極値条件による分岐を実行し、表示用配列には遅延を展開。Waveletはrankを配列走査で求め、C++のbit辞書は再現しません。
- 永続木はコピーしたノードIDと共有参照を表示。区間k番目はprefix頻度差とbitごとの絞り込みを表示します。
- Treapは論理列のsplit／mergeと反転を表示し、乱数優先度の木を描きません。Link-Cutは表現する森と露出したパスを示し、補助splay木の回転を再現しません。
- 動的連結性は時間セグメント木とRollback DSUを実行。MoはL/Rブロックと時間で整列しますが、C++の交互方向ソートによる移動回数の改善は省略。
- Virtual Treeは隣接LCAと祖先スタック、DSU on Treeは重い子の保持と軽い子の再追加を実行。小さい図のDFSには再帰を使います。
- Li Chaoは全直線の挿入を実行。C++の線分追加は個別説明と検証プログラムを参照。
- Monotone Minima／SMAWKの行列は `(r−c)²+offset[c]` に限定し、Monge性を保証。Min-Plusはbが凸か検査します。
- Miller–RabinはBigIntで同じ7基底を実行。Rhoは10^9以下の1回の因数分割をFloydの2速度で表示し、C++のBrent・バッチGCDとは異なります。再試行上限に達すれば未発見と表示。分割した因数が素数とは限りません。
- Subset Convolutionはrank別ゼータ・rank積・メビウスを実行。係数の絶対値は1000以下に制限しNumberで正確に保持。CRTはBigInt、剰余の生成は直接積で行いNTTは再現しません。
- ACとEertreeはJavaScriptの文字単位です。C++のAPIはunsigned byteなので、非ASCII文字の位置・ノード数は一致しない場合があります。ACの図の報告はfailure鎖をたどり、C++の出力link最適化は省略。

モデルはNode.jsでも動きます。

```sh
node verify/visualizations/advanced_model_test.js
node verify/visualizations/advanced_browser_test.mjs --browser /path/to/chromium
```

モデルの最終値は全submask列挙・直接文字列比較・BFS・素朴配列・試し割り・二重ループと照合します。C++側の検証は `python3 verify/run_advanced.py` です。
