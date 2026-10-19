# 実行・観察ツールの参考資料

[一覧](README.md) / [図解](../../visualizations/development_runtime/index.html)

2026-10-19確認。これらはAPIと挙動の確認資料で、実装はこのリポジトリ向けに作成しています。

- [GCC Variadic Macros](https://gcc.gnu.org/onlinedocs/cpp/Variadic-Macros.html): 引数文字列化と可変引数。C++17で値1個以上を受け取るマクロを使用し、無効時は引数を置換結果へ含めません。
- [Python selectors](https://docs.python.org/3/library/selectors.html): nonblocking I/Oのread/write監視。POSIXではpipeを扱い、Windowsではpipeを監視できない制約。
- [Python subprocess](https://docs.python.org/3/library/subprocess.html): argv、cwd、process session、waitとtimeout。[前回の資料](../development/references.md)と共通。
- [Python concurrent.futures](https://docs.python.org/3/library/concurrent.futures.html): ThreadPoolExecutorによる外部プログラムの並列管理、保留中futureの取消。

既存の[stress tester](../../tools/cp_tools/stress.py)のプロセス管理を[共有部品](../../tools/cp_tools/process.py)へ切り出して使用します。C++の128bit判定は既存の[整数traits](../../basic_utility/detail/integer_traits.hpp)を共有します。AHCのスコア方式を一律に仮定せず、外部scorerの生スコアと比較方向を明示的に指定します。
