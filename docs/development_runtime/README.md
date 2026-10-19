# 実行・観察ツール（V-05〜V-07）

前回の[開発・検証基盤 V-01〜V-04](../development/README.md)に、デバッグ出力・対話ジャッジ・複数seedの設定比較を加えます。

| ID | 機能 | 実装 | 説明 |
|---|---|---|---|
| V-05 | C++デバッグ出力と提出時無効化 | [debug.hpp](../../basic_utility/debug.hpp) | [API・使用例](../basic_utility/debug.md) |
| V-06 | 双方向パイプ、timeout、通信ログ | [interactive.py](../../tools/cp_tools/interactive.py) | [設定・実行](interactive.md) |
| V-07 | 複数seed並列実行、外部scorer、保存/replay | [benchmark.py](../../tools/cp_tools/benchmark.py) | [設定・集計規約](benchmark.md) |

[3モードの図解](../../visualizations/development_runtime/index.html) / [参考資料](references.md) / [文書一覧](../README.md)

プロセス起動・終了処理を [process.py](../../tools/cp_tools/process.py) に共有し、既存stress APIからも使います。PythonツールはPython 3.10以上、対話パイプはPOSIX専用です。C++はC++17、cp名前空間です。

## 検証

```sh
make verify-development-runtime
python3 verify/run_development_runtime.py --sanitize
node verify/visualizations/development_runtime_model_test.js
node verify/visualizations/development_runtime_browser_test.mjs --browser /path/to/chromium
```

Pythonの15ケース、共有処理を使う既存stressの5ケース、C++の4マクロ設定×Debug/Release、単独/混在/2翻訳単位、使用例、500個の独立整形比較を検査します。C++対話solverの9セッションと、2設定×12 seedを4並列で実行し、外部scorerの集計を保存入力から独立計算します。対話は40万byteのbinary転送、EOF、flush忘れ、wall/idle timeout、出力量上限、終了失敗を含みます。AHCは実際の並行動作、同一入力、直列/並列一致、スコア欠落/重複/非有限値、巨大値の平均、失敗除外、replayを確認します。

使用例はローカルの数字当てと数列からの要素選択です。オンラインACやAHCコンテストの公式スコアを確認するものではありません。
