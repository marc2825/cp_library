---
title: RSK対応と逆対応
documentation_of: //combinatorics/rsk.hpp
---

# RSK対応と逆対応（C-20）

[ヘッダ](../../combinatorics/rsk.hpp) / [8項目の索引・検証](../combinatorics_extensions/README.md) / [操作できる図解](../../visualizations/combinatorics_extensions/index.html#rsk)

## 使用例

```cpp
#include <iostream>
#include "combinatorics/rsk.hpp"
int main() {
    const auto pair = cp::rsk_word(std::vector<int>{3, 1, 4, 1, 5});
    const auto shape = cp::young_tableau_shape(pair.insertion);
    for (std::size_t i = 0; i < shape.size(); ++i) std::cout << shape[i] << (i + 1 == shape.size() ? '\n' : ' ');
    const auto word = cp::inverse_rsk_word(pair);
    for (std::size_t i = 0; i < word.size(); ++i) std::cout << word[i] << (i + 1 == word.size() ? '\n' : ' ');
}
```

出力:

```text
3 2
3 1 4 1 5
```

## API・条件

`RSKPair<Upper,Lower>` は `insertion`（Lower型のP）と `recording`（Upper型のQ）を保持します。`rsk_word(vector<T>,less)` はQに1始まりのsize_t添字を記録。`inverse_rsk_word(pair,less)` で元の列へ戻します。

`rsk_biword(vector<pair<U,L>>,upper_less,lower_less)` は上段、同値なら下段の辞書順に整列した二行配列を受け、`inverse_rsk_biword` はその順序で復元。比較関数はstrict weak orderで、両方向に同じものを渡します。値はコピー可能、交換可能であることが必要です。

`rsk_matrix(vector<vector<uint64_t>>,max_entries=1000000)` は矩形非負整数行列を(行+1,列+1)の多重集合へ展開。`inverse_rsk_matrix(pair,rows,columns,max_cells=1000000)` は末尾0行/列を含む明示サイズへ復元します。0行では列数はvectorに保持されません。

未整列biword、不正な盤、shape不一致、word版Qが1..Nの標準盤でない場合は `invalid_argument`。行列の展開数/出力セル数予算超過は `length_error`、復元ラベルが指定行列の外なら `out_of_range`。逆操作は盤を値渡しでコピーし、呼出し元は保持します。

## 仕組み

行の最初の「挿入値より大きい」セルを置換し、追い出した値を次の行へ送ります。追い出すセルがなければ行末へ追加して、その位置に時刻/上段値を記録。行は弱増加、列は狭義増加になります。

逆操作はQの最大値のうち最も右のセルから取り除き、上の行の「取り出した値より小さい最も右のセル」と交換して戻ります。wordのPの第1行長は最長非減少部分列の長さ。比較方向を変える場合も同じ規則です。

## 計算量

Nを展開後の要素数として、正/逆とも最悪O(N²)時間・O(N)追加空間。行内探索は二分探索ですが、列の深さに比例したbumpがあり、全体O(N log N)とは限りません。行列版は入力/出力セル数も加算。

## 使用する問題

順列と同形標準盤対の対応、単語の部分列構造、非負整数行列と半標準盤対の相互変換。

## 検証

n≤7全順列、3文字・長さ8までの全単語で往復と盤条件・LNDSを照合。ランダム行列の往復と転置によるP/Q交換、文字列型・逆比較・不正盤を検証。 `python3 verify/run_combinatorics_extensions.py` が単独・重複include、全カテゴリ混在2翻訳単位、説明と同一の使用例、Debug/Releaseを確認します。図解は同じ使用例のC++出力を比較し、別のモデル検証も行います。オンラインACは未確認です。

[一次資料・導出と設計判断](../combinatorics_extensions/references.md) / [カテゴリ索引](README.md)
