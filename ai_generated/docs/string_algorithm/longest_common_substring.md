---
title: Longest Common Substring
documentation_of: //string_algorithm/longest_common_substring.hpp
---

# Longest Common Substring

二つの文字列に連続して現れる共通部分文字列の最長長と、両方での位置を返します。

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/final_a/index.html#common_substring) / [追加APIと検証](../final_a/README.md) / [一次資料](../final_a/references.md)

## API・条件

`longest_common_substring(string_view first, string_view second) -> CommonSubstringResult`。結果はfirst_left、second_left、length、first_right()、second_right()。区間は半開です。同長ならsecond_leftが小さいもの、さらにfirst_leftが小さいものを選びます。空入力・一致なしは全て0。

任意のbyteを扱い、終端文字を入力へ追加しません。入力不変。firstはSAMの長さ上限(INT_MAX−1)/2、secondはINT_MAX以下。既に構築済みなら`SuffixAutomaton::longest_common_substring(second)`を使えます。部分列のLCSとは別の問題です。

## Usage

```cpp
#include <iostream>
#include "string_algorithm/longest_common_substring.hpp"
int main() {
    auto r = cp::longest_common_substring("banana", "ananas");
    std::cout << r.first_left << ' ' << r.first_right() << ' ' << r.second_left << ' ' << r.second_right() << '\n';
}
```

出力:

```text
1 6 0 5
```

[コンパイルできる使用例](../../examples/string_algorithm/longest_common_substring.cpp)。

## 原理

firstのSuffix Automatonでsecondを左から読み、現在一致しているsuffixの長さを持ちます。遷移がなければsuffix linkへ戻り、一致長を短くします。更新した最長長と状態の最初のend位置から、両方の区間を復元します。

## 計算量

n=|first|、m=|second|としてO((n+m) log σ)時間・O(n)領域。既存SAMの問い合わせだけならO(m log σ)時間・O(1)追加領域。σ<=256。

## 使用する問題と検証

[Library Checker Longest Common Substring](https://judge.yosupo.jp/problem/longest_common_substring)。出力区間だけでなく最適長と同長時の規則も比較します。

`python3 verify/run_final_a.py`（`--sanitize`対応）で単独include・複数翻訳単位・このusageと出力・独立した素朴解・境界と例外を確認します。[詳しい検証範囲](../final_a/README.md)。オンライン提出・ACは未確認です。
