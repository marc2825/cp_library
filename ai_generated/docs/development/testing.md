# ローカル検証の使い方

[ライブラリ一覧](../../README.md) / [カテゴリ別の説明・検証範囲](../README.md#カテゴリ索引) / [CIと文書生成](ci.md) / [検証結果の集計](../development_records/verification.md)

以下は `ai_generated/` をカレントディレクトリとして実行します。
親の `cp_library/` からも `make verify-graph-tree` などの検証ターゲットを呼べます。

## 変更したカテゴリを検証する

```sh
make verify-graph-tree
make verify-union-find
make verify-range-query
```

各カテゴリの `docs/<カテゴリ>/README.md` に、使用例・検証コマンド・境界条件・独立比較の範囲を記載しています。
図解モデルとブラウザ操作の検証方法は [図解の説明](../../visualizations/README.md) を参照してください。

## コンパイラ・サニタイザ・大きな入力

```sh
python3 verify/run_graph_tree.py --compiler clang++
python3 verify/run_graph_tree.py --sanitize --stress
```

`--sanitize` はASan/UBSan等を有効にします。`run_graph_tree.py` の `--stress` は20万頂点のパスを使います。
単独・重複include、include順序、複数翻訳単位、使用例、ランダム差分テストと、残している旧APIの互換入口を検証します。
利用できるオプションは各runnerの `--help` で確認できます。

## 複数カテゴリと検証モードをまとめて選ぶ

```sh
python3 -m tools.cp_tools.suite --runners union_find range_query
python3 -m tools.cp_tools.suite --stdlib-debug --runners union_find range_query
```

実行記録とログはGit管理外に保存されます。ソース変更後の古い成功記録は、現在の検証結果として扱いません。
対応付けと実行結果を確認するには `make verification-report` を使います。

## 競技用ツールを検証する

```sh
make verify-contest-usage
make verify-development
make verify-provenance
```

`cplib` の検索・雛形生成・依存込み抽出・提出用展開は、ライブラリ外の作業フォルダでも確認します。
ローカル検証の成功は、オンラインジャッジへの提出・ACやGitHub Actionsの実行結果とは別です。
