# 共通基盤 F-12〜F-17の図解

[図を開く](index.html) / [API・使用例・検証](../../docs/foundation_tools/README.md) / [図解一覧](../index.html)

6モードのJSON入力を編集し、先頭・戻る・次へ・最後・再生を操作できます。各図からAPI説明へ移動できます。

| モード | 内部状態の表示 | 図解の範囲 |
|---|---|---|
| big_rational | 正規化、分母のGCD、交差する項の約分 | 整数は80桁以内、四則演算1回 |
| arena | ノードID、block、slot、共通の前ノード | BlockSize1〜8、追加／clear40操作、整数値の共有リスト |
| bit_operations | mask、2進数、最下位bitと繰り上げ | 0≤k≤n≤64、最大128maskで打ち切り。moreで続きの有無を表示 |
| grid | 入力と変換後の表、範囲内近傍 | 8×8以内のASCII盤面、回転／転置、4／8近傍。origin=nullは近傍取得を省略 |
| rollback_array | 値、履歴番号、添字、旧値、snapshot | 16要素、40操作、save/set/rollback/clear_history |
| radix_sort | 変換キーと各byte整列後の元ID・値 | 8/16/32/64/128bit、32要素以内、符号付き／なし |

JavaScript BigIntを数値モデルに使用します。Arenaは仮想的なノード配置であり、実メモリアドレス、アラインメント、デストラクタ、割当例外はC++で検証します。Rollback図は整数だけを扱い、C++側のgeneric Tやコピー例外は再現しません。GridはC++のbyte表現に合わせASCIIのみ受け付けます。ブラウザ用の個数制限をC++ APIの上限とみなさないでください。

## 検証

```sh
node verify/visualizations/foundation_tools_model_test.js
node verify/visualizations/foundation_tools_browser_test.mjs --browser /path/to/chromium
python3 verify/run_foundation_tools.py
```

C++の6参照値を照合します。モデルは300ランダムバッチ、55個の固定popcountクラスの全探索、64bit境界・古いsnapshot・不正入力・スナップショット独立性を確認します。ブラウザは6モード・6編集入力、再生と移動、エラー表示、ローカルリンクと390px表示を検証します。画像保存は`--screenshot /tmp/foundation-tools.png`で指定できます。
