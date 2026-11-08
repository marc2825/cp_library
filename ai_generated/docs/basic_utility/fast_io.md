---
title: 共通FastIO
documentation_of: //basic_utility/fast_io.hpp
---

# 共通FastIO

先行include・型別名・グローバルなscanner/printerに依存しない、FILE*ベースの共通入出力です。整数をバッファから直接読み書きし、EOFとエラーを区別します。

## 使用例

```cpp
#include <cstdio>
#include <stdexcept>
#include "basic_utility/fast_io.hpp"
int main() {
    std::FILE* file = std::tmpfile();
    if (!file) throw std::runtime_error("tmpfile failed");
    {
        cp::FastPrinter writer(file);
        writer.writeln(-42, "hello", 1.25);
        writer.flush();
    }
    std::rewind(file);
    {
        cp::FastScanner reader(file);
        __int128_t number; std::string word; double real;
        if (reader.read(number, word, real)) {
            cp::FastPrinter out;
            out.writeln(number, word);
            out.write_fixed(real, 2); out.writeln(); out.flush();
        }
    }
    std::fclose(file);
}
```

出力:

```text
-42 hello
1.25
```

## APIと使用条件

`FastScanner`・`FastPrinter`は各64KiBのバッファを持ちます。`BasicFastScanner<N>`・`BasicFastPrinter<N>`で正の容量Nを指定できます。既定のFILE*はそれぞれstdin/stdout。**FILE*を所有せず、closeしません。FILE*はオブジェクトより長く生存させます。** コピー・ムーブは不可。nullは`invalid_argument`です。

| API | 意味 |
|---|---|
| `read(T&) -> bool` | トークンを1つ読む。EOFならfalse、対象値は保持 |
| `read(a,b,...) -> bool` | 左から順に読む。途中でEOFならfalse、それ以前に読めた値は更新済み |
| `write(value)` | 1値を出力。charは1文字、文字列はそのまま |
| `write(a,b,...)` / `writeln(a,b,...)` | 空白区切り。後者のみ末尾改行。引数なしのwritelnは改行のみ |
| `write_fixed(long double x,int digits=10)` | 小数点以下digits桁、0〜1000。それ以外は`invalid_argument` |
| `flush()` / `failed()` | 内部バッファとstdioをflush／出力失敗状態を取得 |

整数はbool・charを除く組み込み整数（128bit含む）。charは空白を飛ばして**1バイト**だけ読みます。符号付き／なしcharを数値で使う場合はsigned/unsigned charを指定します。整数トークンは`[+-]?[0-9]+`、unsignedへの`-`は`-0`を含め拒否。極小値・極大値を読み書きできます。構文不正は`invalid_argument`、範囲外は`out_of_range`。エラーのトークンは末尾まで消費し、対象値は保持するため次のトークンを読めます。

空白はバイト値0〜32。stringは次のトークンで置換し、長さに固定上限はありません。float/double/long doubleは`strtof/strtod/strtold`で読み、`ERANGE`（処理系が報告するunderflowを含む）やNaN・無限大を`out_of_range`として拒否。通常出力は型の`max_digits10`桁、固定出力は`snprintf`を使用します。Cロケールの影響を受けるため、競技入力では既定の`C`ロケールを使ってください。浮動小数点の丸めは処理系の規約に従います。

I/Oエラーは`ios_base::failure`。出力エラー後はfailed状態になり、以降の書き込み・flushも例外です。デストラクタはflushを試みますが例外を外へ出さないため、エラーを確認する場合やインタラクティブ問題では**明示的にflush**してください。FILE*を共有する別のreaderやiostreamとの読み取り混在は、先読みしたバッファがあるため避けます。

整数・文字列の読み書きは処理バイト数Lに対しO(L)時間。固定バッファO(N)、string入力はO(L)、浮動小数点はトークン／整形文字列分の追加領域を使います。modintは`val()`を出力し、入力は整数を読んで構築するかiostreamの`>>`を使います。

### 旧入口からの移行

`fast_io_yosupo.hpp`・`fast_io_nyaan.hpp`は既存コード互換のため保持します。新規コードはこのヘッダと明示的な`cp::FastScanner in; cp::FastPrinter out;`を使ってください。`scanner.read`は戻り値boolを確認し、`printer.writeln`は`out.writeln`へ移します。旧APIのバッファ・公開フィールド・read_unsafe・closeなどの互換を新型に仮定しません。

## 使用する問題と検証

[Library Checker A + B](https://judge.yosupo.jp/problem/aplusb)のローカル入出力を検証します。1バイトのバッファ、末尾改行なし、20万バイトの空白／トークン、整数極値、浮動小数点往復、不正トークン、EOF、読み書きエラーを確認します。

`python3 verify/run_foundation.py` / `--sanitize`（`make verify-foundation`）で単独・重複include、全カテゴリとの混在、複数翻訳単位、Debug/Release、使用例、独立した比較を検証します。オンライン提出・ACは未確認です。[検証範囲](foundation.md)。

[操作できる図解](../../visualizations/foundation/index.html#fast_io) / [図解の範囲](../../visualizations/foundation/README.md) / [カテゴリ索引](README.md) / [一次資料・設計判断](references.md)
