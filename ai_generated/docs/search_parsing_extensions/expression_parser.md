---
title: 再帰下降parser・構文木
documentation_of: //parsing/expression_parser.hpp
---

# X-01 再帰下降parser・構文木

[一覧](README.md) / [ヘッダ](../../parsing/expression_parser.hpp) / [実行例](../../examples/search_parsing_extensions/expression_parser.cpp) / [操作できる図解](../../visualizations/search_parsing_extensions/index.html#expression_parser) / [出典](references.md)

優先順位に従って再帰下降し、子を先に格納した構文木を反復的に評価する。

## APIと条件

parse_expression、ExpressionTree、evaluate_expression<T>、evaluate_integer_expression。

ASCII整数/識別子、単項+/−、+−*/%、右結合^、括弧、ASCII空白。−2^2=−(2^2)。ASTのsourceを所有し、nodeはpostorder・バイト半開区間。型付きParseErrorは位置を持つ。汎用評価callbackとBigInt整数評価を公開。

parse_expression(std::string,ParseLimits={})は入力を所有するExpressionTreeを返します。nodes()はpostorderのExpressionNode列、root()は根ID、source()とtext(node)で元の文字列を参照できます。各nodeのkind/begin/end/token/left/right/operationで構造を調べます。整数・識別子・単項・二項・groupを区別し、括弧自体のspanも保持します。

優先度は^、単項+/−、*/%、+−の順、^だけ右結合です。evaluate_expression<T>(tree,literal,variable,unary,binary)は各callbackを(literal文字列)、(識別子)、(演算子,値)、(演算子,左値,右値)で呼びます。evaluate_integer_expression(tree,variables,max_exponent=1000000)はBigInt。整数除算は0方向への切捨て、剰余は被除数の符号、0^0=1。小数・関数・暗黙の積は受けません。

ParseError::kind()/position()で失敗分類と0始まりバイト位置を確認できます。構文、閉じ括弧、未知変数、0除算、負指数、指数上限を区別します。ParseLimitsはmax_nodes=1000000/max_depth=256。再帰の上限でありAST高さとは異なり、BigIntのbit数や演算時間の上限ではありません。

旧parsing_template.hpp/parse_tree.hppは問題専用mainと重複したグローバル定義を除き、このAPIをincludeする移行入口にしました。旧グローバル関数・変数の互換APIは提供しません。include先を保ったままcp::parse_expressionと明示的な評価callbackへ移行してください。旧内容はGit履歴に残ります。

## 計算量・用途

入力長L・ノードNで構文解析O(L)、領域O(L+N)、再帰上限は既定256。評価は反復O(N)回の値演算と保存。BigInt演算のbit成長は別。

問題例・用途: 整数式の評価、構文木を用いる独自の評価器・式変換。

## 使用例

C++17、namespace cp。並列探索を含むプログラムは-pthreadでリンクします。

```cpp
#include "parsing/expression_parser.hpp"
#include <iostream>
using namespace cp;
int main() {
    auto tree = parse_expression("-2^2 + 3*(x+1)");
    std::cout << evaluate_integer_expression(tree, {{"x", 4}}) << "\n";
}
```

出力:

```text
11
```

## 検証

[共通の検証手順・独立比較](README.md#検証)を使用します。図解の参照値はこのC++使用例の出力と一致させています。オンラインジャッジへの提出結果ではありません。
