# SA受理判定のベンチマーク

[APIと確率誤差](fast_annealing_acceptance.md) / [測定コード](../../verify/search_parsing_extensions/benchmark.cpp)

固定seedで生成した正のlossとtemperatureを使い、exp判定とテーブル判定を比較します。
測定には乱数生成と公開APIの引数検査を含み、近傍生成・score計算・状態保存・温度計算は含みません。

```sh
g++ -std=c++17 -O3 -DNDEBUG -Wall -Wextra -Werror -I . verify/search_parsing_extensions/benchmark.cpp -o /tmp/ex_benchmark
/tmp/ex_benchmark
```

どちらも試行ごとにraw乱数を1回消費します。exp側は53bitの[0,1)格子、table側は12bitの区間中点です。
固定系列での採否は同一ではなく、採択件数の差は個々の採否差の件数を意味しません。
テーブルの実数計算上の確率誤差は1/8192以下に浮動丸めを加えます。
loss=0ではテーブルは乱数を消費しませんが、この測定は正のlossだけを対象にします。

性能はCPU・コンパイラ・精度・テーブルの大きさ・cache状況・scoreの費用に依存します。
測定結果は探索全体の速度を保証せず、テストの合否条件にも使いません。
