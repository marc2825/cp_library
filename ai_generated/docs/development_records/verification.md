# 機能・検証問題・検証状況の集計

[一覧](README.md) / [実装](../../tools/cp_tools/catalog.py) / [提出ACの台帳](../../verification/accepted.json)

```sh
python3 -m tools.cp_tools.catalog --output build/catalog
```

全 `.hpp` / `.h` をファイル単位の機能として列挙します。`all.hpp` はaggregate、detail配下はinternalとして区別します。既存の `verify/*/manifest.json`、C++検証ソースのリテラルinclude、`PROBLEM` 定義を集め、説明・検証runner・問題URL・出典を結び付けます。新規ヘッダは自動的に列挙されるので、未verifyのものも隠れません。

| 状態 | 必要な記録 |
|---|---|
| `unverified` | 現在のソースに一致する成功記録なし。問題リンクやテストソースだけでは昇格しない |
| `local-only` | 対応runnerが共通launcherから成功し、ソース全体のfingerprintが一致 |
| `online-ac` | 実際の提出ACを確認した記録があり、提出プログラムと到達するローカル依存のハッシュが一致 |

レポートには各ヘッダのtests/runners/problems/manifests/doc/provenance、全テスト一覧、全runner一覧、実行の成否・モード・鮮度、提出ACの鮮度を保存します。HTMLでは検索語と状態を同時に絞り込めます。外部スクリプトやサーバーは不要です。

## ローカル実行の証拠

```sh
python3 -m tools.cp_tools.suite --runners union_find range_query
python3 -m tools.cp_tools.suite --stdlib-debug --runners union_find range_query
python3 -m tools.cp_tools.catalog
```

launcherは `verification/evidence/<runner>/<profile>.json` にモードごとの最新結果を保存します。開始時に前の成功をinterruptedへ置き換え、終了後にpassed/failedを記録します。失敗や中断で以前の成功を維持しません。異なるモードの成功が残る場合は、台帳のattemptsからそれぞれ確認できます。同じrunner・モードを同時実行しないでください。

ソース全体のfingerprintにはヘッダ、C++使用例・テスト、Pythonツール・runner、JSON/TOML設定を含めます。ファイル追加・削除も検知し、コード変更は保守的に全ローカル記録を失効させます。証拠ファイル自身・build生成物は含めません。実行中にソースが変化した場合も成功扱いにしません。日時・コマンド・終了コード・経過時間を保存し、stdout/stderrは `build/suites/` に置きます。

旧文書の「ローカル検証済み」や旧ヘッダの提出リンクは、機械検証できる現在版の証拠として移し替えていません。既存runnerをlauncherから再実行すれば記録できます。runnerとヘッダの対応はmanifestやinclude到達関係によるもので、関数単位のテスト網羅率や、全ケースを検査した保証ではありません。

## 提出ACの証拠

`verification/accepted.json` のsubmissionsは初期状態では空です。実際の提出結果と提出したコードを確認した後、次を登録します。

- `test`：対応する `verify/.../*.test.cpp`。
- `problem`：そのファイルのPROBLEMと完全一致するURL。
- `submission_url` / `verdict` / `reviewed_at`：実際の提出のURL、`AC`、確認日。
- `sources`：提出元テストと、リテラルincludeで到達する全ローカルファイルのSHA-256辞書。

ハッシュの対象は [repository.closure](../../tools/cp_tools/repository.py) のinclude到達集合です。登録は人が提出とソースの一致を確認した記録であり、台帳自体が外部サービスの判定を取得・認証するものではありません。competitive-verifierによるダウンロード済みケース実行も実際の提出とは異なるため、提出ACの代用にしません。

## 集計の限界

マクロincludeやコンパイラ固有のinclude探索は評価しません。条件付きincludeは両方の枝を含め、外部システムヘッダは列挙しません。`.git`、build、隠しディレクトリ等は探索対象外です。巨大なリポジトリではinclude到達集合とJSON出力が大きくなります。検証台帳は信頼できる作業者が作る記録であり、改ざん耐性を提供する署名システムではありません。
