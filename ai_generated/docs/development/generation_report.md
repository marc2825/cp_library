# 生成ライブラリ・検索カタログの対応状況

[API・対応制約](generation.md) / [JSON・CLI・stress](generation_json.md) / [全機能検索](../../FEATURES.md) / [JSONカタログ](../../catalog/features.json)

主な追加先は `ai_generated/tools/cp_tools/` です。依存パッケージを増やさず、既存Random・Graph・stress・ヘッダメタデータを再利用しています。

## 完了した範囲

- P0: 任意精度整数、複数整数・タプル、依存スカラー、配列・文字列、制約検査、出力分離、再現性、CLI、機能索引・README導線。
- P1: 基本行列・到達性グリッド・木・疎グラフ・区間・クエリ、複数ケース、総和・二乗和予算、特殊分布、JSON、既存stress接続。
- P2の一部: 状態付き集合・多重集合・スタック・キュー・辺操作、拡張callback、正しい括弧列、回文・アナグラム。
- P3の一部: O(N²)候補列挙を避ける新グラフAPI、カタログ自動収集・検査、実行例22項目、性能測定スクリプト、CIへの追加。

P1の全制約組合せへの対応ではありません。各APIが保証する構成と制限はAPI説明に記載しています。

## 未実装・一部対応

一般制約ソルバー、行和と列和の同時指定、一般迷路・島数、任意次数・指定SCC数、負閉路の有無の保証、
高度な文字列条件、素数・GCD/LCM専用サンプラー、一般幾何配置、制約を保つ縮小器、ストリーミング出力は未実装です。
幾何は既存の凸多角形生成を再利用します。総予算は独立整数のsum範囲と二乗和上限まで対応します。
任意predicate・JSON constraintsは試行回数に上限があり、予算切れを不可能と断定しません。

## ファイルの役割

| 変更・追加 | 内容 |
|---|---|
| tools/cp_tools/random_data.py | 既存64bit乱数列を維持し、bits/randbelow/choice/weighted_choiceを追加 |
| tools/cp_tools/generators.py | 基本型、タプル、配列、文字列、サイズ割当 |
| tools/cp_tools/structures.py | 行列、グリッド、木・グラフ、区間・状態付きクエリ |
| tools/cp_tools/formatting.py | 行レイアウト・複数ケース |
| tools/cp_tools/generation.py | JSON検査、依存式、独立ケースseed、予算、CLI |
| tools/cp_tools/feature_catalog.py | 公開機能収集、日英検索、更新・チェック |
| tools/cp_tools/contest.py | gen/search/catalogコマンド追加、既存find等を維持 |
| catalog/、FEATURES.md | JSON、Markdown索引、ハッシュ付き部分検証証跡 |
| examples/development/generation/ | 20実行例・3種JSON・stress設定・利用手順 |
| verify/generation/、verify/run_generation.py | 独立検証・CLI・カタログ検証・ベンチマーク |
| docs/development/generation*.md | API、設定、使用例、制限、実測・検証報告 |
| README.md、docs/README.md、docs/development/{README,random_data}.md | ナビゲーション追加 |
| Makefile、../.github/workflows/verify.yml | 新検証ターゲット・CI |

リポジトリルートのREADMEにも索引リンクを追加しています。既存のC++アルゴリズム実装は変更していません。

## 検証と性能測定

2026-11-15のローカル検証結果:

| 検証 | 結果 |
|---|---|
| 新生成器・JSON・CLI・カタログ | 29テスト成功。小グラフの全実現可能辺数、複数seedの構造・操作列も検査 |
| 既存開発ツール | Python 14テスト、既存CLI 42コマンド、Python参照200例が成功 |
| 既存C++統合 | 5プログラムの原本・bundle版をDebug/Releaseで比較し成功 |
| 差分テスト | 既存generatorと新JSON generatorで各80ケース成功（seed 100〜179） |
| 出典・検証台帳 | 14テストと、801ヘッダ・371 C++テストソース・60 runnerの整合性確認が成功 |
| リンク・索引 | 開発ドキュメント522リンク、全FEATURESリンク・frontmatter、決定的再生成を確認 |
| 出典台帳 | 11レコード・明示追跡14ファイルの既存検査が成功 |

環境にBoostヘッダがなかったため、Ubuntuのlibboost1.83-devを `/tmp` に展開し、C++統合テストの
compilerに `g++ -isystem /tmp/cp-library-boost/extracted/usr/include` を指定しました。システムへのインストールは行っていません。
GitHub上のworkflow自体、全C++ runnerの全モード、オンラインジャッジの実行は今回実施していません。

カタログは **889件**（ai_generated 845件、depricated 44件、completedは実装0件）。
C++789件（公開ヘッダ・集約ヘッダ・既存テンプレート）、Python公開関数・クラス100件を登録しています。
既存のcategory公開ヘッダ727件、旧構成15ヘッダ、depricatedの44ヘッダも登録済みです。
41件はソースハッシュに一致するローカル証跡に基づくpartially_tested、残り848件はunknownです。
C++のpublic_api全関数名を網羅的に抽出したカタログではなく、説明・manifestのAPI要約とヘッダ単位の索引です。

性能測定（Python 3.12.3、Linux x86_64 / WSL2、seed=42、1回の実行値）:

| 処理 | 秒 | ピーク追跡メモリ MiB |
|---|---:|---:|
| 整数配列200000要素 | 0.189 | 7.653 |
| 文字列200000文字 | 0.208 | 3.099 |
| 連結グラフ200000頂点・200000辺 | 0.609 | 29.213 |
| 状態付き集合クエリ200000件 | 0.310 | 19.641 |
| 10000ケースのサイズ予算割当 | 0.010 | 0.082 |
| JSON 10000ケース・配列生成・整形 | 0.270 | 9.086 |

計測コマンドと生の結果は [benchmark.py](../../verify/generation/benchmark.py) と、実行後の `build/generation-benchmark.json` です。
再実行手順:

```sh
make verify-generation
make verify-development
make verify-development-records
python3 tools/cplib.py catalog --check
python3 verify/generation/benchmark.py --out build/generation-benchmark.json
```

ベンチマークは生成時間と、別実行でtracemallocが観測したピークメモリを分けて測定します。
OS全体のRSS、出力ファイルI/O、他環境での速度は保証しません。

## 互換性と改善候補

既存関数シグネチャとbelowの上限制約、64bit範囲内の整数乱数消費を維持します。
旧グラフ生成器は既存の小・中規模APIとして残し、大規模用途には新APIを追加しました。
カタログのC++登録単位は公開ヘッダです。全メソッドの自動抽出やテスト網羅率の推測は行っていません。
今後は明示的なC++ APIメタデータ、構造別縮小、相互依存する予算、ストリーミングを拡張できます。
