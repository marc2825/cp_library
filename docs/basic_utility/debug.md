---
title: デバッグ出力
documentation_of: //basic_utility/debug.hpp
---

# デバッグ出力（V-05）

[基本ユーティリティ](README.md) / [開発基盤](../development_runtime/README.md) / [図解](../../visualizations/development_runtime/index.html#debug) / [参考資料](../development_runtime/references.md)

## 使用例

```cpp
#include <iostream>
#include <string>
#include <tuple>
#include <vector>
#include "basic_utility/debug.hpp"
int main() {
    std::vector<int> values{3, 1, 4};
    CP_DEBUG(values);
    std::cout << cp::debug_string(std::make_tuple("state", values, true)) << '\n';
    std::cout << cp::debug_string(values, {8, 2, 256}) << '\n';
}
```

[同じ例](../../examples/basic_utility/debug.cpp)を `g++ -std=c++17 -I . -DCP_ENABLE_DEBUG examples/basic_utility/debug.cpp` でコンパイルすると、CP_DEBUGが式名・ソース位置・値をstderrへ出します。定義しなければマクロは無効です。

## API・対応型

`cp::debug_string(value, DebugOptions={})` は整形した文字列を返します。`DebugOptions{max_depth=8,max_items=32,max_string=256}` は入れ子の深さ・各コンテナの要素数・文字列のbyte数を制限し、残りを `...` にします。rootの深さは0。明示的に呼んだ関数は提出設定でも実行されます。

| 型 | 表示 |
|---|---|
| 整数・bool・char | 10進整数（符号付き/なし128bitも可）、true/false、引用した1文字 |
| string/string_view・char配列・C文字列 | 二重引用符。改行、tab、制御byte、quote、backslashをescape |
| pair/tuple/std::array | `(要素, ...)` |
| constで反復可能なコンテナ | `[要素, ...]`。mapはpairの列。unorderedの順序はそのコンテナの反復順 |
| optional/variant | `nullopt`、`optional(value)`、`variant[index](value)`、valueless |
| 独自型 | `cp::DebugFormatter<T>::write(ostream&,const T&)` の特殊化を最優先。次に標準の対応規則、最後にoperator<<。それもなければ `<unprintable>` |

文字列はUTF-8のcode pointではなくbyte単位で打ち切ります。C文字列ポインタはnull、または有効なNUL終端列である必要があり、長さの取得には終端まで走査します。char配列は固定長を使い、最後のNULだけを除くので埋め込みNULを表示できます。queue/stack等、反復できない型はformatterかoperator<<を用意します。独自formatterの出力量・計算量には上限を強制しません。

## 有効化・無効化

`CP_DEBUG(a,b,...)` はstderr、`CP_DEBUG_TO(stream,a,b,...)` は指定streamへ出力します。値は1つ以上渡します。`CP_ENABLE_DEBUG` が定義され、`CP_DISABLE_DEBUG` と `ONLINE_JUDGE` が両方未定義のときだけ有効です。これらは最初のincludeより前に指定してください。`NDEBUG`とは独立です。

無効時は引数を評価せず、型検査もしません。`CP_DEBUG(++counter)` でcounterは増えません。有効時の各引数の評価は1回ですが、複数引数の評価順には依存しないでください。マクロは改行するだけでflushはしません。複数threadのログの不可分性は保証しません。

## 計算量・検証・用途

組み込みの整形処理は、実際に訪問した要素と文字列byte、出力長に比例し、戻り文字列の長さに比例するメモリと入れ子に比例するstackを使います。無効マクロは実行時コスト0です。

4種類のマクロ設定×Debug/Release、単独/重複include、全カテゴリ混在と2翻訳単位、整数極値・256個のsigned char・独自型・空tuple・文字列escape・打ち切り・引数評価を検証します。500個の入れ子配列をPython表現と比較します。問題の状態確認、対話問題のstderrログ、AHCの内部値観察に使えます。
