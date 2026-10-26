---
title: Young図形と標準盤の列挙
documentation_of: //combinatorics/young_tableaux.hpp
---

# Young図形と標準盤の列挙（C-21）

[ヘッダ](../../combinatorics/young_tableaux.hpp) / [8項目の索引・検証](../combinatorics_extensions/README.md) / [操作できる図解](../../visualizations/combinatorics_extensions/index.html#young_tableaux)

## 使用例

```cpp
#include <iostream>
#include "combinatorics/young_tableaux.hpp"
int main() {
    const auto count = cp::enumerate_standard_young_tableaux({3, 2}, [](const auto& tableau) {
        return cp::is_standard_young_tableau(tableau);
    });
    std::cout << count << '\n';
    const auto shape = cp::conjugate_young_shape({3, 2});
    for (std::size_t i = 0; i < shape.size(); ++i) std::cout << shape[i] << (i + 1 == shape.size() ? '\n' : ' ');
}
```

出力:

```text
5
2 2 1
```

## API・条件

`YoungShape=vector<int>` は正の非増加行長、`YoungTableau<T>=vector<vector<T>>` はそのセル。空shape/空盤は有効、長さ0の行は無効です。`is_young_shape`、`young_shape_size(shape,max_cells=1000000)`、`conjugate_young_shape`、`young_tableau_shape`、`transpose_young_tableau(tableau,max_cells)` を提供します。

`is_semistandard_young_tableau(tableau,less)` は行が弱増加・列が狭義増加かを判定。`is_standard_young_tableau` は整数ラベル1..Nが各1回かも検査。`young_tableau_descents` はi+1がiより下の行にあるiを昇順で返します。転置はセル配置の操作なので、半標準盤の転置が半標準になるとは限りません。

`enumerate_standard_young_tableaux(shape,callback,max_tableaux=100000,max_cells=1000000)` は標準盤を全列挙し、実際のcallback呼出し数を返します。callbackは `bool(const YoungTableau<size_t>&)`。falseで中止、参照はその呼出し中だけ有効です。空shapeも1盤。上限ちょうどで列挙が完結すれば成功、さらに盤が存在すると `length_error`（それ以前のcallbackは取り消しません）。不正shape/descents対象は `invalid_argument`、セル/行長の表現範囲超過は `length_error`。

## 仕組み

1..Nを、これまでに埋めた部分がYoung図形であることを保つ追加可能な角へ順に置きます。明示スタックで後戻りするため、背の高い図形でもC++呼出しスタックを深くしません。盤の個数だけ必要ならHook-Length Formulaを使います。

## 計算量

Nセル、h行、出力T盤として、検査・転置O(N)、shape検査O(h)。標準検査・降下はO(N)追加空間。列挙は最悪O(TNh)時間とO(N+h)作業空間にcallbackの処理を加算。全盤を内部に保存しません。

## 使用する問題

標準/半標準盤の検査、共役分割、盤の全列挙、RSK出力の解析、対称群の組合せ。

## 検証

n≤9の全整数分割を列挙し、標準盤の重複なし・転置往復・Hook数との一致を確認。空盤、予算、callback停止、1万行でも再帰stack不要なことを検証。 `python3 verify/run_combinatorics_extensions.py` が単独・重複include、全カテゴリ混在2翻訳単位、説明と同一の使用例、Debug/Releaseを確認します。図解は同じ使用例のC++出力を比較し、別のモデル検証も行います。オンラインACは未確認です。

[一次資料・導出と設計判断](../combinatorics_extensions/references.md) / [カテゴリ索引](README.md)
