# 出典・検証台帳と共通検証モード（V-08〜V-10）

Python 3.10以上。外部Pythonパッケージは不要です。リポジトリのルートから実行します。

| ID | 機能 | 説明・実装 |
|---|---|---|
| V-08 | 移植元・ライセンス・取り込み版・変更内容の追跡 | [出典台帳](provenance.md) / [JSON](../../verification/provenance.json) / [validator](../../tools/cp_tools/provenance.py) |
| V-09 | 全ヘッダと検証ソース・問題・実行記録の対応集計 | [検証台帳](verification.md) / [collector](../../tools/cp_tools/catalog.py) |
| V-10 | 全runnerで選べるlibstdc++ debug mode | [共通検証モード](stdlib_debug.md) / [launcher](../../tools/cp_tools/suite.py) |

```sh
python3 -m tools.cp_tools.provenance
python3 -m tools.cp_tools.suite --stdlib-debug --runners union_find range_query
python3 -m tools.cp_tools.catalog
```

最後のコマンドは `build/catalog/catalog.json` と、検索・状態絞り込みができる `build/catalog/index.html` を生成します。HTMLは直接開けます。生成結果を手編集せず、出典のJSONや実行記録を更新して再生成してください。出典台帳をGitで管理し、生成物・ログ・`verification/evidence/` の実行記録はGit管理外に置きます。

既存の [V-01〜V-04](../development/README.md)、[V-05〜V-07](../development_runtime/README.md)も利用できます。[一次資料・設計判断](references.md)。

## 検証

```sh
make verify-development-records
make verify-stdlib-debug
node verify/development_records/browser_test.mjs --browser /path/to/chromium
# 対象を追加する場合
python3 -m tools.cp_tools.suite --stdlib-debug --runners foundation_tools development development_runtime
```

14件のテストで、小規模リポジトリを使った出典・ライセンスの変更、未登録の帰属、循環include、manifest、成功・失敗・タイムアウト、ソース変更による失効、提出ACの証拠、HTMLエスケープを確認します。C++の無効イテレータがDebug/Release双方で検出されること、2翻訳単位間でコンテナを受け渡せることも実コンパイル・実行します。既存の全runnerについて共通オプションの存在を確認します。

全runnerを全モードで実行済みという意味ではありません。実際に実行したrunnerとモードは生成台帳の `attempts` を参照してください。現在のソースに一致する記録がないものは未verifyと表示します。
