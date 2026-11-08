---
title: Beam Search
documentation_of: //heuristic_search/beam_search.hpp
---

# Beam Search

[カテゴリ](README.md) / [実装](../../heuristic_search/beam_search.hpp) / [使用例](../../examples/heuristic_search/beam_search.cpp) / [図解](../../visualizations/heuristic_search/index.html#beam) / [一次資料](references.md)

各深さで全ての子を生成・評価し、評価順の上位width個を次のfrontierへ残します。同点は親のfrontier順、次に子の列挙順を保ちます。重複状態を自動統合しません。幅を狭めると最適解へ続く経路を落とす場合があります。

## usage

```cpp
#include <iostream>
#include <vector>
#include "heuristic_search/beam_search.hpp"
struct State { int depth, weight, value; };
int main() {
    const std::vector<int> weight{3, 4, 2}, value{4, 5, 3};
    cp::BeamSearchOptions options;
    options.width = 4;
    options.max_depth = 3;
    options.goal = cp::SearchGoal::maximize;
    const auto result = cp::beam_search(State{0, 0, 0},
        [](const State& s) { return s.value; },
        [&](const State& s) {
            const int i = s.depth;
            if (i == 3) return std::vector<State>{};
            std::vector<State> children{{i + 1, s.weight, s.value}};
            if (s.weight + weight[i] <= 5)
                children.push_back({i + 1, s.weight + weight[i], s.value + value[i]});
            return children;
        }, options);
    std::cout << result.best().score << ' ' << result.best().state.weight << '\n';
}
```

`beam_search(initial,evaluate,expand,options={})`。expand(const State&)は子状態の範囲を値で返します。vector<State>を推奨します。各子をコピー・ムーブでき、生成順が決定的である必要があります。経路復元が必要ならState内に経路や利用側の親IDを保持してください。

BeamSearchOptionsはwidth=10（1以上）、max_depth=100（0も可）、goal=minimize、任意のstop。返値BeamSearchResultはfrontier（ScoredSearchState{state,score}のvector）、depth、evaluations、termination。best()はfrontier先頭へのconst参照で、返値オブジェクトの生存中だけ有効です。結果の公開frontierを空にした後にはbest()を呼べません。

depthは子を生成・選別して完成した層数。max_depth=0なら初期状態1個。子が全くない層ではexhaustedとして直前の非空frontierを返し、depthは増えません。一部の親だけ終端なら、その親は次の層に残りません。終端解を保持したい場合は自分自身へ進む子を生成するか、終端解を利用側で保存します。異なる深さの評価値を比べるグローバル最良解は保持しません。

stopは各親の展開前と各子の評価前に確認。途中でtrueならその層を捨て、最後に完成したfrontierをstoppedで返します。その層で済んだ評価もevaluationsに含めます。展開・ソート中の中断や生成候補数の上限はありません。

## 計算量・使いどころ・検証

深さdの全生成候補数をC_dとすると、評価・生成以外に通常O(Σ C_d log C_d)時間、最大層でO(W+C_d)個の状態を保持します。stable_sortの作業領域確保ができない場合はO(C_d log² C_d)となり得ます。候補全体を保持する実装なので、メモリは幅Wだけでは決まりません。状態コピーと子生成のコストも必要です。

問題例は[AHC021](https://atcoder.jp/contests/ahc021/tasks/ahc021_a)の部分解を幅で残す探索。使用例の小ナップサックは説明用で、一般の入力での最適性を保証する例ではありません。

2,000個の閉路・重複・終端を含むケースを、最良候補を一つずつ取り出す独立オラクルで照合。狭い幅で大域最適を失う反例、全葉が残る幅、同点順、深さ0、候補枯渇、部分層中断、幅200・深さ200を検証します。Pythonの層ごとの定義とも比較します。
