---
title: 編集距離・操作列の復元
documentation_of: //dp_optimization/edit_distance.hpp
---

# 編集距離・操作列の復元

挿入・削除・置換が各1、保持が0のLevenshtein距離を求めます。文字列に限らず、任意の要素列に使えます。

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/dp_optimization/index.html#edit) / [実装](../../dp_optimization/edit_distance.hpp) / [参考資料](references.md)

## usage

```cpp
#include <iostream>
#include <string>
#include "dp_optimization/edit_distance.hpp"
int main() {
    std::string a = "kitten", b = "sitting";
    auto result = cp::edit_alignment(a, b);
    std::cout << result.distance << '\n';
    for (auto step : result.steps) {
        if (step.kind == cp::EditKind::Keep) std::cout << a[step.source_index];
        else if (step.kind != cp::EditKind::Erase) std::cout << b[step.target_index];
    }
    std::cout << '\n';
}
```

[動く使用例](../../examples/dp_optimization/edit_distance.cpp)。リポジトリのルートをincludeパスへ通します。

## API・使用条件

`edit_distance(a,b,equal=equal_to<>{}) -> int`、`edit_alignment(a,b,equal) -> EditDistanceResult`。

結果は `distance` と `steps`。各 EditStep は `kind`、`source_index`、`target_index` を持ちます。

| kind | 実行すること | 消費する添字 |
|---|---|---|
| Keep | a[i]を残す | iとjを1つ進める |
| Replace | a[i]をb[j]へ変える | iとjを1つ進める |
| Erase | a[i]を削除 | iのみ進める |
| Insert | b[j]を出力へ挿入 | jのみ進める |

添字は**元の両列を読むカーソル**です。変形中の文字列へ直接erase/insertする位置ではありません。stepsはKeepも含み、先頭から適用できます。空列の挿入／削除にも対応。列の条件は[LCS](lcs.md)と同じです。

## 原理・計算量・注意点

時間 O(NM+N+M)。距離のみ O(M+1) 空間、操作復元は O((N+1)(M+1)) 空間です。復元では対角を優先し、同点の最短操作列は一意ではありません。

接頭辞の表で上=削除、左=挿入、対角=保持／置換。等値なら対角の費用0、不一致なら1です。重み付き編集距離・転置操作を含む距離はこのAPIの対象外です。std::string はバイト列として扱います。

## 使用する問題・検証範囲

[AOJ DPL_1_E: Edit Distance](https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=DPL_1_E)。

二値文字列225組について、実際の文字列間の編集を辺とするBFSと比較。全操作のカーソル、再構成した列、費用を検査。空・整数列・等値判定・長さ2,000も確認。

[検証入口](../../verify/run_dp_optimization.py)は通常／ASan・UBSan、debug/release、単独include、混在include、複数翻訳単位、使用例、独立オラクル、ジャッジ形式の入出力を確認します。オンラインACは未確認です。
