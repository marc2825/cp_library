---
documentation_of: //string_algorithm/subsequence_automaton.hpp
---

# 部分列オートマトン・異なる部分列数（S-01）

[カテゴリ](README.md) / [S全11項目](../string_extensions/README.md) / [実装](../../string_algorithm/subsequence_automaton.hpp) / [操作できる図解](../../visualizations/string_extensions/index.html#subsequence)

## API・使用条件

`SubsequenceAutomaton(text)`、`next(state,byte)`、`match(pattern,start=0)`、`contains`。`distinct_subsequences(text,modulus,include_empty=true)`。

状態は消費したprefix長。nextは遷移後の長さ、失敗は−1。matchは最左の添字列をoptionalで返す。空patternは成功。個数は空部分列を既定で含み、正のuint64法（1も可）を指定する。

C++17、namespace `cp`。入力はbyte列、添字は0始まり・半開区間です。UTF-8の文字単位には変換しません。0を含む入力は長さ付きstring/string_viewで渡してください。入力長はintの範囲内、BWT/LCEはINT_MAX未満、SAMは状態数を含めてintに収まる範囲です。形状・法・予算等の違反はinvalid_argument/out_of_range/length_errorで拒否します。各ヘッダとstring_algorithm/all.hppから利用できます。

## 原理・計算量

各byteの次の出現位置へ進む。同じ文字の前回追加前の個数を記録し、重複した部分列を2倍した総数から引く。

構築O(256n)時間・領域、照合O(|pattern|)、next O(1)。個数のみはO(n+256)時間・O(256)領域。

## 使用例

[コンパイル可能なソース](../../examples/string_algorithm/subsequence_automaton.cpp)。

```cpp
#include "string_algorithm/subsequence_automaton.hpp"
#include <iostream>

template <class T> void print(const std::vector<T> &a) {
    for (std::size_t i = 0; i < a.size(); ++i)
        std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}

int main() {
    cp::SubsequenceAutomaton a("ababa");
    std::cout << cp::distinct_subsequences("ababa", 1000000007) << '\n';
    print(*a.match("baa"));
}
```

出力:

```text
20
1 2 4
```

## 使用する問題・検証

部分列の存在判定、最左埋込み、異なる部分列の数え上げ。

[比較検証一覧](../string_extensions/README.md#検証)。独立した全探索・動的計画法、通常/debug mode/sanitizerのDebug/NDEBUG、全カテゴリ混在・2翻訳単位・使用例を確認します。オンラインACは未確認です。

[一次資料・設計判断](../string_extensions/references.md) / [図解の対応範囲](../../visualizations/string_extensions/README.md)。
