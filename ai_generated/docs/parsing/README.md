# 構文解析・有限オートマトン

[カテゴリ索引](../README.md) / [all.hpp](../../parsing/all.hpp)

[X全3項目のAPI・使用例・独立比較・図解](../search_parsing_extensions/README.md)を追加しました。式parser/AST、DFA/NFAの決定化・最小化・積・桁DP、Thompson正規表現を扱います。

旧parsing_template.hpp/parse_tree.hppは新しい式parserをincludeする入口です。問題専用mainと旧グローバル定義は除去しています。[移行と対応文法](../search_parsing_extensions/expression_parser.md)を参照してください。
