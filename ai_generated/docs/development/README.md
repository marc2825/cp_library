# 開発・検証基盤（V-01〜V-04）

Python 3.10以上のローカル開発ツールです。C++ライブラリのPython移植とは別です。

[競プロでの使い方](../usage.md)に、任意の作業フォルダから使える `tools/cplib.py` を追加しました。V-04バンドラを共用し、ヘッダ検索・雛形・依存込み抽出・VS Codeのinclude補完を提供します。

続く[V-05〜V-07](../development_runtime/README.md)としてC++デバッグ出力、対話ローカルジャッジ、AHC複数seed並列実行を追加しました。プロセス起動・終了は共通の [process.py](../../tools/cp_tools/process.py) を使います。

| ID | 機能 | 実装・設定 | 説明 |
|---|---|---|---|
| V-01 | 固定版competitive-verifier、CI、文書生成 | [runner](../../tools/verify_ci.py)・[workflow](../../../.github/workflows/verify.yml) | [構成・対象・実行方法](ci.md) |
| V-02 | seed付き差分実行、失敗保存、timeout、replay | [stress.py](../../tools/cp_tools/stress.py) | [API・使用例](stress.md) |
| V-03 | 木・森・グラフ・DAG・単純多角形 | [random_data.py](../../tools/cp_tools/random_data.py) | [API・生成分布・計算量](random_data.md) |
| V-04 | C++ローカルinclude展開・重複排除 | [bundle.py](../../tools/cp_tools/bundle.py) | [対応構文・検証](bundle.md) |

[4モードの図解](../../visualizations/development/index.html) / [参考資料](references.md) / [文書一覧](../README.md)

[V-08〜V-10の出典・検証台帳と共通debug mode](../development_records/README.md)では、既存の全runnerを共通入口から選び、現在のソースと一致する実行記録を集計できます。

## 検証

カテゴリごとの実行、コンパイラ変更、サニタイザ、共通suiteの使い方は[ローカル検証ガイド](testing.md)を参照してください。

```sh
make verify-development
python3 verify/run_development.py --sanitize
node verify/visualizations/development_model_test.js
node verify/visualizations/development_browser_test.mjs --browser /path/to/chromium
```

通常検証は標準Python・g++・Boostヘッダで動きます。バンドラの元ファイルと展開後ファイルをDebug/Releaseでコンパイルし、同一入力の出力を比較します。5組の実プログラム、80 seedのC++ DSU対Python探索、生成器の連結性・閉路・幾何条件、異常終了・失敗保存・再実行・timeout・POSIX子プロセス終了を確認します。サニタイザはC++実行にASan/UBSanを付けます。

固定版verifier自体の接続確認は、[依存](../../.competitive-verifier/requirements.txt)を入れた仮想環境で `python verify/run_development.py --with-verifier` を実行します。未コミットの追加も検査できるよう、一時ディレクトリに必要なファイルだけをGit snapshotとしてコピーします。実リポジトリのindexは変更しません。standaloneの実行・文書生成・3オンライン問題の設定と依存解決までを検査し、テストケースのダウンロードは行いません。

GitHub上でのworkflow実行、オンライン問題の全ケース実行、公開サイトの配信は別途行う操作です。この追加だけでオンラインACとは扱いません。
