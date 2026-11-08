[![AtCoder Algorithm Rating](https://badgen.org/img/atcoder/marc2825/rating/algorithm)](https://atcoder.jp/users/marc2825)
[![AtCoder Heuristic Rating](https://badgen.org/img/atcoder/marc2825/rating/heuristic)](https://atcoder.jp/users/marc2825?contestType=heuristic)
[![Codeforces Rating](https://codeforces-readme-stats.vercel.app/api/badge?username=marc2825)](https://codeforces.com/profile/marc2825)

# cp_library

旧ライブラリ、AI生成ライブラリ、完成版を次のフォルダで管理します。

| フォルダ | 内容 |
|---|---|
| [depricated/](depricated/) | 整理前の `main` にあった旧ライブラリ。内容をそのまま保存しています。 |
| [ai_generated/](ai_generated/) | `ai/generated-libraries` ブランチのライブラリ・説明・使用例・検証・開発ツール一式。 |
| `completed/` | 完成版を置くための空フォルダ。 |

README、Makefile、テンプレート、`user_snippet.json`、エディタ設定はルートに置いています。
`depricated` は指定されたフォルダ名です。

Git は空フォルダを追跡しないため、`completed/` にファイルは追加していません。
新しく clone した環境では `mkdir -p completed` で作成してください。

## ライブラリの利用

AI生成版は [使い方](ai_generated/docs/usage.md) と [ライブラリ一覧](ai_generated/README.md) を参照してください。
利用するライブラリのフォルダを include パスに指定します。

```sh
# AI生成ライブラリ
g++ -std=c++17 -O2 -I /path/to/cp_library/ai_generated main.cpp

# 旧ライブラリ
g++ -std=c++17 -O2 -I /path/to/cp_library/depricated main.cpp

# AI生成ライブラリの検索・提出用ファイルへの展開
python3 ai_generated/tools/cplib.py find dsu
python3 ai_generated/tools/cplib.py bundle /path/to/main.cpp -o /path/to/submission.cpp --check
```

## 検証

ルートの Makefile から AI生成版の検証を実行できます。

```sh
make verify-development
make verify-graph-tree
make verify-provenance
make verification-report
```

`make -C ai_generated <ターゲット>` でも実行できます。
