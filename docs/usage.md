# 競プロで使う：include・コピペ・提出用コード

[ライブラリ一覧](../README.md) / [API索引](README.md) / [cplibの実装](../tools/cp_tools/contest.py) / [バンドラの詳しい仕様](development/bundle.md)

普段は必要なヘッダを `#include` し、提出前に依存ヘッダを1ファイルへ展開する方法が便利です。新しい `cplib` コマンドは、問題用の作業フォルダから使えます。インストールや外部Pythonパッケージは不要です。

| 使いたい方法 | 手順 |
|---|---|
| ローカルでライブラリを読み込む | `#include "カテゴリ/ヘッダ.hpp"` とコンパイラの `-I` |
| ジャッジへ1ファイルで提出する | `cplib bundle main.cpp -o submission.cpp --check` |
| ライブラリ本体をコピペする | `cplib snippet dsu --check -o library.hpp` で依存込みのコードを生成 |
| 新しい問題の雛形を作る | `cplib init main.cpp --use dsu --use modint` |
| ヘッダ名・日本語の説明から探す | `cplib find UnionFind` / `cplib find 最小費用流` |
| VS Codeでincludeを補完する | 問題フォルダで `cplib snippets`、エディタで `cp/dsu` を補完 |

このリポジトリはC++17のヘッダライブラリです。C++での読み込みは `#include` で、C++20 Modulesの `import` 用モジュールは提供していません。Python版のアルゴリズムはまだ未実装で、`import cp_library` では使えません。`tools.cp_tools` はコード生成・検証用のPythonツールです。

## 最初に1回：短いコマンドを用意する

Python 3.10以上と、C++17を使えるコンパイラを用意します。この環境では次をBashに貼れば使えます。別の場所にcloneした場合はパスを変えてください。

```sh
export CP_LIBRARY="/home/marc2825/cp_library"
cplib() { python3 "$CP_LIBRARY/tools/cplib.py" "$@"; }
```

次回のターミナルでも使うなら、この2行を自分の `~/.bashrc` に保存します。ツールはシェル設定を自動変更しません。関数を登録しなくても、例えば `python3 /home/marc2825/cp_library/tools/cplib.py find dsu` と実行できます。

```sh
cplib --help
cplib root
cplib find dsu
cplib find suffix array
```

`find` はカテゴリの `all.hpp` があるディレクトリから公開ヘッダを列挙し、パス・説明タイトル・短縮名で検索します。複数語はAND条件、大文字小文字や通常の区切り記号を吸収します。`detail/` は検索と補完から除外します。結果にはヘッダパスと説明の絶対パスが表示されます。検証済みかどうかの判定は[台帳](development_records/verification.md)を参照してください。

## 通常の使い方：includeして、最後にbundleする

問題用フォルダで作業します。

```sh
mkdir -p ~/contests/practice
cd ~/contests/practice
cplib init main.cpp --use dsu
```

`init` は選んだinclude・入出力設定・空のmainを作ります。既存ファイルは上書きしません。次の完成例は [contest_connectivity.cpp](../examples/development/contest_connectivity.cpp) にも保存しています。

```cpp
#include <iostream>
#include "union_find/disjoint_set_union.hpp"

int main() {
    std::ios::sync_with_stdio(false);
    std::cin.tie(nullptr);
    int n, q;
    if (!(std::cin >> n >> q)) return 0;
    cp::DisjointSetUnion dsu(n);
    while (q--) {
        int type, u, v;
        std::cin >> type >> u >> v;
        if (type == 0) dsu.merge(u, v);
        else std::cout << dsu.same(u, v) << '\n';
    }
}
```

コンパイルと実行:

```sh
g++ -std=c++17 -O2 -Wall -Wextra -I "$(cplib root)" main.cpp -o main
./main < input.txt
```

`-I` はヘッダを探すディレクトリです。ライブラリ直下を指定します。ヘッダ内のローカル依存はC++のincludeで読み込まれるため、自分で依存順を並べる必要はありません。新しいAPIは `cp::DisjointSetUnion` のように `cp::` を付けて使います。

入力例:

```text
4 5
1 0 1
0 0 1
1 0 1
0 2 3
1 1 3
```

出力は順に `0`、`1`、`0` です。提出する前に1ファイルへ展開します。

```sh
cplib bundle main.cpp -o submission.cpp --check
g++ -std=c++17 -O2 submission.cpp -o submission
./submission < input.txt
```

提出欄には **生成された `submission.cpp` の全内容** を貼ります。問題用の `main.cpp` はinclude形式のまま編集し、変更したら再びbundleします。`--check` は展開後のソースを一時ディレクトリでコンパイル・リンクし、成功してから出力します。オンラインジャッジへの提出は行いません。`bundle -o` は既存の提出用出力を更新しますが、入力ソース・その依存ヘッダへの上書きは拒否します。

`bundle` は既存V-04バンドラと同じ実装で、リポジトリのincludeパスを自動追加した入口です。出力先を省くと標準出力に展開します。

## コピペで使う場合

ヘッダ1個をそのまま貼るだけでは、内部の `#include "../..."` がジャッジで見つからない場合があります。`snippet` で必要な依存もまとめてください。

```sh
cplib snippet dsu --check -o library.hpp
cplib snippet fenwick modint --check -o library_two.hpp
```

生成ファイルの **全内容** を、解答のmainや自作namespaceの外側に貼ります。元ヘッダの `namespace cp` を含むので、貼り付け後も同じ `cp::` APIで使えます。複数のヘッダを一度に指定すると共有依存をまとめます。`--check` はmainなしで構文検査するため、クラスだけの抽出でも使えます。

`snippet` はmainを付けません。上の `library.hpp` をローカルで `#include "library.hpp"` して使い、最後に `bundle` することもできます。`snippet -o` は既存ファイルを保護します。生成物を更新するときは `--force` を付けます。`--check` 失敗時は既存出力を保持し、`--force` でも入力依存そのものへの上書きは拒否します。

出力先なしの `cplib snippet dsu` はコードを標準出力へ出します。ターミナルからコピーするか、自分の環境のクリップボードコマンドへ渡せます。ツール自身はクリップボードを変更しません。

## 名前の指定とカテゴリ単位のinclude

`init --use` と `snippet` は次を受け付けます。

| 指定 | 意味 |
|---|---|
| `union_find/disjoint_set_union.hpp` | リポジトリ相対の正確なヘッダパス |
| `range_query/fenwick_tree` | 拡張子を省いたカテゴリ付きパス |
| `disjoint_set_union` | 公開ヘッダの中で一意なファイル名 |
| `dsu` | `union_find/disjoint_set_union.hpp` |
| `fenwick` | `range_query/fenwick_tree.hpp` |
| `segtree` | `range_query/segtree.hpp` |
| `modint` | `basic_utility/static_modint.hpp` |

名前が曖昧なら候補を出して停止します。例えば `all` だけは複数カテゴリに一致するため、`union_find/all.hpp` のように指定します。正確なリポジトリ内の `.hpp` / `.h` パスなら旧ヘッダも明示指定できますが、古いグローバルAPI・問題専用コードには各ファイルの条件が適用されます。

```cpp
#include "union_find/all.hpp"
#include "range_query/all.hpp"
```

カテゴリの `all.hpp` は探索・試作時に便利です。提出コードのサイズやコンパイル時間を抑えるなら、実際に使う個別ヘッダへ絞ります。bundleは未使用関数やクラスを削るツールではなく、指定したincludeを展開します。

## VS Codeでincludeを補完する

VS Codeで開いている **問題用フォルダ** のターミナルで実行します。

```sh
cplib snippets
```

`.vscode/cp-library.code-snippets` にC++向けinclude補完を生成します。この時点の727ヘッダを実ファイルから列挙するため、後でライブラリを更新したら `cplib snippets --force` で再生成できます。ほかのsnippet・settings・tasksは変更しません。

C++ファイルの行頭で `cp/dsu` または `cp/union_find/disjoint_set_union` を入力し、補完候補を選ぶとinclude行が挿入されます。短いprefixは一意な名前にだけ付けます。候補が出ない場合は `Ctrl+Space`、またはコマンドパレットの `Insert Snippet` で選べます。[VS Code公式のsnippet仕様](https://code.visualstudio.com/docs/editing/userdefinedsnippets)に沿った、workspace用・scope=`cpp`のJSONです。

Microsoft C/C++拡張を使う場合、補完用のヘッダ検索パスも設定できます。既存の `.vscode/settings.json` へ必要な項目をマージしてください。パスは自分の環境に合わせます。

```json
{
  "C_Cpp.default.includePath": [
    "${workspaceFolder}",
    "/home/marc2825/cp_library"
  ],
  "C_Cpp.default.compilerPath": "/usr/bin/g++",
  "C_Cpp.default.cppStandard": "c++17"
}
```

この設定はエディタのIntelliSense用です。実際のコンパイルにも前述の `-I` が必要です。[公式のC++設定リファレンス](https://code.visualstudio.com/docs/cpp/customize-cpp-settings)を参照してください。

## 外部依存・コンパイル条件

- `<iostream>` 等の標準ヘッダはbundle後も残ります。通常のC++コンパイラ環境で使います。
- BigInt・BigRational等はBoostヘッダに依存します。bundleはBoost自体を同梱しないため、提出先のコンパイラにもBoostが必要です。各APIの説明で確認してください。
- `parallel_search.hpp` 等のthreadを使うコードは `-pthread` を付けます。検査も `cplib bundle main.cpp -o submission.cpp --check --flag=-pthread` と合わせます。
- 128bit整数やoverflow builtin等を使う機能があるため、このライブラリの検証環境はGCC/Clang系です。MSVCだけでの全機能互換は確認していません。
- マクロで作るinclude名や `#include_next` はバンドラの対象外です。条件分岐内のローカルヘッダも存在する必要があります。`__FILE__` / `__LINE__` は展開後の位置になります。[対応構文と制限](development/bundle.md#対応構文と重複)を参照してください。

追加include・コンパイラ・マクロ条件も指定できます。

```sh
cplib bundle main.cpp -I ./extra -o submission.cpp --check --compiler g++ --flag=-DMY_FLAG=1
cplib bundle main.cpp -o submission.cpp --dependencies dependencies.json
cplib snippet dsu --check --compiler g++ --flag=-Wall
```

`--compiler` は実行ファイル名/パス、追加オプションは `--flag=...` を繰り返します。`--check` は展開後のコンパイルを確認するだけで、解法の正しさやジャッジの環境差を保証しません。

## Pythonからツールをimportする場合

競技用アルゴリズムのPython移植とは別に、コード生成をPythonから呼べます。リポジトリ直下で実行するか、リポジトリをPythonのモジュール検索パスへ追加します。

```python
from pathlib import Path
from tools.cp_tools.bundle import bundle

root = Path("/home/marc2825/cp_library")
text = bundle(Path("main.cpp"), [root])
Path("submission.cpp").write_text(text, encoding="utf-8")
```

上のコードを問題フォルダの `bundle_script.py` に保存する場合は、`PYTHONPATH="$CP_LIBRARY" python3 bundle_script.py` と実行できます。通常はcwdやPython検索パスの設定が不要な `cplib` の入口を使えば足ります。

## 検証

`make verify-contest-usage`、および既存の `make verify-development` で確認できます。リポジトリ外・空白を含む作業パスで実コマンドを実行し、雛形/個別include/展開後/コピペ用抽出をC++17でコンパイルして同じ出力になることを確認します。Debug/NDEBUG、複数ヘッダの共有依存、未知/曖昧な名前、既存出力や依存ファイルの保護、コンパイル失敗時の保持、VS Codeの全prefixの一意性も検証します。VS CodeのJSON生成は検証対象ですが、エディタ本体でのキー操作は自動検証していません。

2026-10-29の最終検証：development通常21.7秒・libstdc++ debug24.2秒が成功しました。新CLIの42コマンド、727ヘッダと補完、ガイドの完成例、既存の14ツールテスト・5組の元/展開後プログラム（Debug/NDEBUG）・80 seed比較を含みます。[通常記録](../verification/evidence/development/standard.json) / [debug記録](../verification/evidence/development/stdlib-debug.json)。
