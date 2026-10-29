# SA受理判定の実測

[APIと確率誤差](fast_annealing_acceptance.md) / [測定コード](../../verify/search_parsing_extensions/benchmark.cpp)

2026-10-29、Intel Core Ultra 9 285K（x86_64）、GCC 11.4.0、C++17、-O3 -DNDEBUGで測定しました。lossを[0.001,8.001)、temperatureを[0.1,4.1)から固定seedで前生成し、200万回の悪化採否を評価します。1回のwarmup後に7回実行し、exp/テーブルの順を交互にして中央値を取ります。CPU固定は行わず、他の検証プロセスと環境を共有したローカル値です。

| 項目 | 結果 |
|---|---:|
| 従来形：rng.real() < exp(−loss/T) | 90.031 ms |
| 12bitテーブル：loss < T×threshold[raw上位12bit] | 10.191 ms |
| 時間比（exp / table） | 8.834 |
| テーブル構築（測定ループ外） | 0.129 ms |
| テーブル領域（4096×sizeof(long double)） | 65536 bytes |
| 採択件数（exp / table） | 491601 / 491599 |

どちらも試行ごとにraw乱数を1回消費します。exp側は53bitの[0,1)格子、table側は12bitの区間中点です。固定系列での採否は同一ではなく、集計件数差も個々の採否差の件数を意味しません。テーブルの実数計算上の確率誤差は1/8192以下に浮動丸めを加えます。loss=0はテーブルでは消費0ですが、この測定は正のlossだけです。

測定には乱数生成と公開テーブルAPIの引数検査を含み、近傍生成・score計算・状態保存・温度計算は含みません。long double版のexpと比較しています。CPU、コンパイラ、精度、テーブルの大きさ、cache状況、scoreの費用が変われば時間比も変わるので、探索全体が8.8倍になるという保証ではありません。性能値はテストの合否条件にしません。

```sh
g++ -std=c++17 -O3 -DNDEBUG -Wall -Wextra -Werror -I . verify/search_parsing_extensions/benchmark.cpp -o /tmp/ex_benchmark
/tmp/ex_benchmark
```
