# 標準ライブラリdebug modeの共通設定

[一覧](README.md) / [launcher](../../tools/cp_tools/suite.py) / [GCC公式説明](https://gcc.gnu.org/onlinedocs/libstdc++/manual/debug_mode_using.html)

```sh
# 既存の全runnerを列挙
python3 -m tools.cp_tools.suite --list
# 既定のUnion-Find / Range Query
make verify-stdlib-debug
# 任意のカテゴリをまとめて実行
python3 -m tools.cp_tools.suite --stdlib-debug --runners ordered_structure offline_query
# 全runner（時間がかかります）
python3 -m tools.cp_tools.suite --stdlib-debug --runners all
# ASan/UBSanも併用
python3 -m tools.cp_tools.suite --stdlib-debug --sanitize --runners union_find
```

既存runnerの共通 `--compiler` オプションへ、コンパイラと追加フラグを渡します。コマンドはargvとして実行し、shellを介しません。複合コンパイラコマンドは `--compiler 'ccache g++'` のように指定できます。リポジトリ内に追加した `verify/run_*.py` も自動的に対象になります。

| 選択 | profile名 | 追加内容 |
|---|---|---|
| 無指定 | standard | 各runnerの通常設定 |
| `--stdlib-debug` | stdlib-debug | `_GLIBCXX_DEBUG`、`_GLIBCXX_DEBUG_PEDANTIC`、`_GLIBCXX_ASSERTIONS` |
| `--sanitize` | sanitize | 各runnerの既存ASan/UBSan設定 |
| 両方 | stdlib-debug-sanitize | debug modeと既存サニタイザ設定 |

debug modeはlibstdc++を必要とし、実行前に実コンパイル・起動でマクロと標準ライブラリを検査します。libc++向けの設定を提供したものではありません。`NDEBUG`とは独立しており、既存runnerがDebug/Release双方を試す場合は両方に適用します。`--compiler` からdebug用マクロを別指定することは避け、launcherの選択を使います。

libstdc++ debug modeは標準コンテナのサイズや動作を変えます。コンテナを渡し合う翻訳単位は同じ設定でコンパイルする必要があります。この入口はrunnerのコンパイラコマンド全体へフラグを付け、新しく一時バイナリを作ります。事前に作った異なる設定のオブジェクトとの混在は対象外です。バックトレース用の追加ライブラリは要求しません。

`--timeout` はrunner単位の秒数（既定1800秒）、`--output` はstdout/stderrの保存先、`--evidence` は実行記録の保存先を変更します。0終了かつ実行中にソースが変化しなかった場合に成功を記録します。失敗があるとlauncherは1を返し、残る指定runnerの実行を続けます。timeoutや中断時はPOSIXでprocess groupを終了します。

対象選択はrunner単位です。ASan/UBSan、標準コンテナの検査、独立した素朴解との比較はそれぞれ異なる不具合を検出するため、既存の通常・サニタイザ検証と組み合わせて使います。オンラインジャッジ用のcompetitive-verifier設定にはこのモードを強制しません。

順序統計とオフラインクエリのstressは、通常モードでは20万要素、debug modeでは2,000要素で同じ操作・境界を検査します。debug modeの二分探索前の範囲検査が大規模な反復を極端に遅くするためです。通常検証の大規模ケースは維持しています。
