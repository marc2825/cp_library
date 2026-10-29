---
title: 多点並列探索・並列Multi-Start SA
documentation_of: //heuristic_search/parallel_search.hpp
---

# E-20 多点並列探索・並列Multi-Start SA

[一覧](README.md) / [ヘッダ](../../heuristic_search/parallel_search.hpp) / [実行例](../../examples/search_parsing_extensions/parallel_search.cpp) / [操作できる図解](../../visualizations/search_parsing_extensions/index.html#parallel_search) / [出典](references.md)

開始点ごとに乱数系列を固定して並列に探索し、終了後にrun番号順で最良解を選ぶ。

## APIと条件

parallel_multi_start(starts,task,seed,options)、parallel_multi_start_sa。

C++17 std::thread、-pthread。各runに固定seedとtaskのコピーを割り当て、結果はrun順。共有可変データは同期が必要。stopは割当時に直列化、実行中taskにはatomic cancelを渡す。全threadをjoinしてから例外伝播。

ParallelSearchOptionsはworkers/goal/stop。task(index,Random&,const atomic<bool>& cancel)はbest_scoreを持つ任意の結果を返します。各runにtaskのコピーを作り、親Random(seed)からrun番号順にseedを先に割り当てます。純粋なcallback・固定回数予算ではworker数を変えてもrunごとの結果が同じです。共有参照先の変更やprogress callbackは呼び出し側で同期してください。

戻り値runsはrun番号順のoptional<Result>、completed、stopped、best_run_index。best_run()は未完了だけならlogic_errorです。同点は小さいrun番号を選びます。stopは新しいrunを割り当てる際にmutex下で呼びます。実行中taskにはcancelを通知しますが、taskが確認しなければ終了まで待ちます。例外時は全threadをjoinし、失敗したrunの最小番号の例外を再送します。時間制限・中断時の実行済み集合は順序依存です。

parallel_multi_start_saは既存SAを各workerで実行します。AnnealingOptionsのstopは直列化し、探索全体のcancelと組み合わせます。AnnealingOptionsのgoalを最終比較にも使います。C++17でコンパイル・リンク時に-pthreadが必要です。

## 計算量・用途

P workerで理想的には総仕事/Pだが負荷・同期・起動費用に依存。結果とseedはO(starts)、同時作業領域はPタスク分。

問題例・用途: 独立した開始点・seedでのSAや探索の並列実行。

## 使用例

C++17、namespace cp。並列探索を含むプログラムは-pthreadでリンクします。

```cpp
#include "heuristic_search/parallel_search.hpp"
#include <iostream>
using namespace cp;
struct Run {
    int best_state, best_score;
};
int main() {
    auto task = [](std::size_t, Random &random, const std::atomic<bool> &) {
        Run result{0, 1000};
        for (int i = 0; i < 3; ++i) {
            int x = int(random.uniform(100)), score = std::abs(x - 37);
            if (score < result.best_score)
                result = {x, score};
        }
        return result;
    };
    auto result = parallel_multi_start(4, task, 42);
    std::cout << result.completed << " " << *result.best_run_index << " " << result.best_run().best_state
              << " " << result.best_run().best_score << "\n";
}
```

出力:

```text
4 3 42 5
```

## 検証

[共通の検証手順・独立比較](README.md#検証)を使用します。図解の参照値はこのC++使用例の出力と一致させています。オンラインジャッジへの提出結果ではありません。
