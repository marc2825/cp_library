---
title: 差分更新型Beam Search
documentation_of: //heuristic_search/delta_beam_search.hpp
---

# E-05 差分更新型Beam Search

[一覧](README.md) / [ヘッダ](../../heuristic_search/delta_beam_search.hpp) / [実行例](../../examples/search_parsing_extensions/delta_beam_search.cpp) / [操作できる図解](../../visualizations/search_parsing_extensions/index.html#delta_beam_search) / [出典](references.md)

候補木の共通prefixをDFSし、辺をapply/undoしながら状態を共有する。残す候補だけを次のfrontierへ進める。

## APIと条件

delta_beam_search(state,evaluate,moves,delta,apply,undo,BeamSearchOptions)。

movesはvector<Move>、deltaはScoreと同じ符号付き型。applyは強い例外保証、undoはnoexcept。入力stateは成功/停止/例外でも復元する。同点は元のfrontier順位と子順。深さ間で評価値を比較しない。

evaluate(state)で初期Score、moves(state)でvector<Move>、delta(state,move)で差分、apply/undo(state,move)で状態を変更します。既存BeamSearchOptionsのwidth/max_depth/goal/stopを使います。各層のfrontierの子を比較し、同点はfrontier順位→子の生成順です。途中停止した層は捨て、最後に選抜が完了した層を返します。

結果はbest_state/best_score/moves/depth/evaluations/stored_nodes/termination。movesはrootからの復元手順です。異なる深さ間のscoreは比較しません。rootは最終状態のコピーが投げた場合もRAIIで復元します。共有prefixを反復DFSするため、深さに比例するC++再帰stackは使いません。

applyは例外なら状態を変えない強い保証、undoはnoexceptを要求します。delta/evaluate/movesは観測のみです。Stateのコピーは解の返却時に必要です。枝数やscoreの良さによる重複除去は行いません。

## 計算量・用途

幅W・深さDで保存O(WD)個のmoveと祖先。各層は有効祖先のDFSと候補CのO(C log C)選抜。一時候補も含めた領域はO(WD+C)。祖先走査は全層で最悪O(WD²)。Stateコピーは最後の解保存時だけ。

問題例・用途: 状態が大きく、各moveの適用/取消しが小さい構築型探索。

## 使用例

C++17、namespace cp。並列探索を含むプログラムは-pthreadでリンクします。

```cpp
#include "heuristic_search/delta_beam_search.hpp"
#include <iostream>
using namespace cp;
int main() {
    int root = 0;
    auto score = [](const int &x) { return (x - 5) * (x - 5); };
    BeamSearchOptions options;
    options.width = 3;
    options.max_depth = 4;
    auto result = delta_beam_search(
        root, score, [](const int &) { return std::vector<int>{-1, 2}; },
        [&](const int &x, const int &m) { return score(x + m) - score(x); },
        [](int &x, const int &m) { x += m; }, [](int &x, const int &m) noexcept { x -= m; }, options);
    std::cout << result.best_state << " " << result.best_score << " " << root << "\n";
}
```

出力:

```text
5 0 0
```

## 検証

[共通の検証手順・独立比較](README.md#検証)を使用します。図解の参照値はこのC++使用例の出力と一致させています。オンラインジャッジへの提出結果ではありません。
