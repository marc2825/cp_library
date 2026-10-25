# cp_library 開発の引き継ぎ書

更新日: 2026-10-25。新しいセッションでは、このファイル・[実装予定リスト](docs/implementation_plan.md)・対象カテゴリの README を読んでから作業してください。追加後には、実装予定リスト・下の「現状」・検証コマンドを更新します。

## ユーザーの方針

競技中に必要な実装を取り出しやすく、詳細を読みたいときには説明と図解へ辿れるライブラリを作っています。AI にカテゴリを順番に依頼し、数回おきに新しいセッションへ切り替える予定です。

- **`3.xx` ごとに独立したカテゴリフォルダを作る。** 新しい実装をすべて `data_structure/` に集めない。
- ヘッダはコピーしやすい短い説明と実装を中心にする。長い `/* ★ Name ... */` 形式の説明を新たに置かない。
- 長い説明は `docs/`、操作できるビジュアル解説は **`docs/` とは別の `visualizations/`** に置く。
- 各ライブラリに説明・usage・API／使用条件・計算量・使用する問題・検証範囲・参考資料を用意する。
- 静止画だけでなく、入力や操作を変えて内部状態を追えるインタラクティブ解説を用意する。実装言語は自由。
- 説明・カテゴリ索引・共通の図解一覧から相互に辿れるようにする。
- 有名ライブラリを適宜調べ、実用的な API と検証を揃える。オンライン AC をしていないものを AC 済みと書かない。

## A/B項目の補完状況と次の作業

2026-10-18の追加依頼で、[未実装・追加仕様の統合一覧](docs/remaining_work.md)を作成しました。依頼本文・添付2資料・元C全101掲載箇所を実コードと照合し、重複を除いて245作業単位（未実装174、一部対応40、旧実装の整備待ち6、要仕様化25）に整理しています。元Cから各IDへの全件対応表、実装済みとして除外した27組の根拠も同ファイルにあります。P-recursiveの既知漸化式prefixなど、元Cでも既に満たす部分があるため、ランクだけで未実装と判断しません。初回はコミットと一覧化のみでした。2026-10-19の「未実装のものを順に」の依頼で一覧順に着手し、F-01〜F-07、続く「次」の依頼でF-08〜F-10を整備しました。「再開」の依頼でF-11も整備しました。「もう少しまとめて」の依頼でF-12〜F-17の6項目を追加しました。[V-01〜V-04の開発・検証基盤](docs/development/README.md)も整備しました。[V-05〜V-07の実行・観察ツール](docs/development_runtime/README.md)も整備しました。現在は残141（未実装104、一部対応18、整備待ち5、要仕様化14）と追加整備済み104。[V-08〜V-10の出典・検証台帳と共通debug mode](docs/development_records/README.md)も整備しました。[U-01〜U-11・U-20〜U-22の14項目](docs/utility_search/README.md)も整備しました。[D-01（SWAG Deque）](docs/static_query/swag_deque.md)も整備しました。[D-02〜D-10の9項目](docs/data_structure_extensions/README.md)も整備しました。[D-20〜D-35の16項目](docs/data_structure_advanced/README.md)も整備しました。D-11〜D-19の行はありません。今回の指定で[T全7項目](docs/tree_extensions/README.md)も整備しました。[G-01〜G-15](docs/graph_extensions/README.md)と[G残り15項目](docs/graph_advanced/README.md)も整備しました。**掲載順の未着手先頭はP-01**です。Python移植は別計画です。

ユーザー提供の3.1〜3.25の全リストを、[実装予定リスト](docs/implementation_plan.md)へS/A/B/Cごと保存しました。**A全169掲載箇所は補完済みです。2026-10-17の指定に従い、3.1〜3.6のB全43掲載箇所も整備しました。続いて2026-10-18に3.7〜3.11のB全36掲載箇所を整備し、続いて3.12〜3.15のB全30掲載箇所を整備し、続いて3.16〜3.20のB全31掲載箇所を整備し、続いて3.21〜3.25のB全28掲載箇所を整備し、B全168掲載箇所が完了しました。** 次のカテゴリ・ランクは以後の依頼に従い、未指定のS/B/Cを自動で広げません。

2026-10-17の追加依頼で、**3.1〜3.25全体のPython版ライブラリを作成する計画**も実装予定リストへ追記しました。今回は計画の追加であり、Python版の実装には未着手です。配置予定は `python/cp_library/<category>/`。C++ APIとの対応表、Python向けの型・整数・再帰・性能条件、個別説明・使用例・比較検証・索引を揃える方針です。検証用PythonスクリプトをPython版実装として数えません。

1. 開始時にGit状態と実装予定リストを読み、実ファイル・既存API・説明・検証を再確認する。作業ツリーに未コミット変更がある場合は保持する。
2. **3.1から3.25のカテゴリ順、同じカテゴリでは掲載順**を基本とし、その回の指定対象を選ぶ。A全169掲載箇所と3.1〜3.25のB全168掲載箇所は実装済み。全Bは整備済み。S/CやPython版の次の着手範囲はユーザーの指定に従う。3.16と3.19の集合畳み込み、GCD/LCM・Min-Plus・漸化式推定・Matrix Treeの重複は共通実装済み。3.1の共通部品は `basic_utility/` に配置済み。
3. 「実装あり」「旧実装あり」は再実装しない。他カテゴリで機能を満たすものや重複項目は既存の実体を共有する。「一部対応」は注記された機能・公開API・組合せ使用例／検証などの不足分を補う。
4. 空ファイル・説明雛形・WIP・内部専用関数を、完成した公開APIと数えない。特に旧2D Segtreeは未完成、旧Functional Graphは説明雛形。内部CRTがあっても汎用CRT APIは未実装として扱う。
5. 既存の開発方針どおり、実装だけでなく個別説明・usage・API条件・計算量・問題例・出典・動く使用例・差分検証・操作できる図解を揃える。適切な検証を実行して、`all.hpp`・各索引・必要なMakefileターゲット・予定リストの状態と参照先・この引継書を更新する。

予定リストのチェックは掲載箇所単位で、重複を含みます。現在のAは169箇所（実装あり166、旧実装あり3、一部対応0、未実装0）。この数値は実装追加で変わるため、再開時は予定リストと実ファイルを優先します。2026-10-17に3.1〜3.6の不足A32掲載箇所を補完しました。Mo on Treeは3.11の重複項目も更新。続いて3.7〜3.11の不足A15掲載箇所、3.12〜3.15の不足A19掲載箇所も補完しました。続いて3.16〜3.20の不足A9掲載箇所、3.21〜3.25の不足A10項目を補完しました。

## 共通基盤 F-01〜F-07（2026-10-19）

[公開API・使用例・検証](docs/basic_utility/foundation.md) / [一次資料](docs/basic_utility/references.md) / [7モードの図解](visualizations/foundation/index.html)。`basic_utility/all.hpp`に追加。

- StaticModInt・DynamicModIntはuint32全域の正の法、合成数法・optional逆元に対応。DynamicはIDと世代検査付きで、set_modは同じ法でも古い値を無効化。
- Barrettはuint64全域の剰余。Montgomeryは奇数法1以上2^63未満の実行時reducer／固定法modint。2^61−1専用modintとハッシュ接続例。
- FastIOはFILE*非所有の共通ヘッダ、128bit・浮動小数点・EOF・極値・不正入力。旧FastIO2種と既存型は保持。
- 共通安全整数は検査・飽和・非負上限判定・番兵加算・符号付きfloor/ceil。GCC/Clangの128bitとoverflow builtinを使用。
- 検証入口: `make verify-foundation` / `python3 verify/run_foundation.py --sanitize`。Python任意精度12,196入力をDebug/Releaseで比較、10万reducer・8bit全探索・I/O・7使用例・全カテゴリincludeと複数翻訳単位・2ジャッジ形式を確認。オンラインACは未確認。
- 図解: `node verify/visualizations/foundation_model_test.js` / `node verify/visualizations/foundation_browser_test.mjs --browser /path/to/chromium`。

今回の実行結果：通常・ASan/UBSan/LeakSanitizerのDebug/Release、7使用例、7件のC++／図解参照値、2ジャッジ形式×32入力が成功。図解モデル6,000比較、全7モードのブラウザ操作と390px表示が成功し、desktop/mobile画像も確認しました。関連文書リンク1,345件に欠落なし。既知のsocket/ptrace制限に従い、ChromiumとLeakSanitizerは承認レビュー付きで同じローカル検証を制限外から実行しました。

## 共通基盤 F-08〜F-10（2026-10-19）

[公開API・使用例・検証](docs/foundation_structures/README.md) / [一次資料](docs/foundation_structures/references.md) / [5モードの図解](visualizations/foundation_structures/index.html)。各カテゴリのall.hppへ追加。

- SeededHashは既存RandomのSplitMix64を共有。整数・128bit・pair/tuple/array・列に対応し、固定seedで再現可能。HashMapは線形探索・削除印を使うopen addressing。コピー可能なキー／値、findは値ポインタ、rehashで参照無効化、容量再構築の例外保証を明記。
- CSRGraphは共有graphとgraph_basicの入口に配置。論理辺IDは入力順、無向のrevは平坦arc配列の番号、有向は-1。Graphとの変換はコピー。既存Graph専用アルゴリズムへはto_graph()で接続。
- MonoidSegtree／ActedMonoidLazySegtreeは既存実装の別名。min/max＋個数、最大部分配列、行列積、affine関数、一次分数、rolling hashを追加。関数の結合は左を適用してから右、lazy作用のcompositionはf after g。
- 多項式／等差加算は絶対座標のmomentを保持。初期化はvalues()、点更新はleaf(value,coordinate)を使う。固定幅型の途中値が表現可能であることが必要。
- 検証入口: `make verify-foundation-structures` / `python3 verify/run_foundation_structures.py --sanitize`。6単独ヘッダ、全カテゴリ混在と複数翻訳単位、5使用例、衝突・削除・例外・20万キー／頂点、全区間の素朴比較、Python独立比較4,470出力、2ジャッジ形式×24入力をDebug/Releaseで確認する。オンラインACは未確認。
- 図解: `node verify/visualizations/foundation_structures_model_test.js` / `node verify/visualizations/foundation_structures_browser_test.mjs --browser /path/to/chromium`。5モード、Monoidの7種類、C++参照値、独立モデル1,200比較。

今回の実行結果：通常・ASan/UBSan/LeakSanitizerのDebug/Release、5使用例・5件のC++／図解参照値・Python4,470出力・2ジャッジ形式×24入力が成功。図解モデルと全5モード／7種類のMonoidのブラウザ操作、390px表示が成功し、Hash Map・CSR・モバイルの画像も確認しました。関連文書リンク1,514件に欠落なし。前回F-01〜F-07の通常回帰検証も成功。Chromiumのsocket制限とLeakSanitizerのptrace制限による起動失敗後、承認レビュー付きで同じローカル検証を制限外から実行しています。

## 共通基盤 F-11（2026-10-19）

[多倍長整数のAPI・使用例・検証・性能](docs/basic_utility/big_integer.md) / [4モードの図解](visualizations/big_integer/index.html)。basic_utility/all.hppに追加したため、同入口にもBoostヘッダが必要です。実行時の追加リンクライブラリは不要。

- cp::BigIntはBoost cpp_intのラッパー。通常演算・比較・増減・cp::abs/divmod/gcd/lcm、厳密な10進／16進文字列変換、128bitまでのto_integral範囲検査。通常整数の既存gcd/lcmは保持。
- 商は0方向へ切り捨て、余りは被除数と同符号。0除算はdomain_error。gcd(0,0)=0、LCMの0入力は0。負の16進出力は符号付き絶対値。ストリームは常に10進、不正トークンは消費してfailbit・元の値を保持。
- 固定桁数上限なし。今回検証は入力1万桁・積2万桁。Boost 1.74・GCC 11.4で検証し、1000／10000桁のparse・乗算・除算・GCD・出力の参考計測を記録。乗算はBoostのKaratsuba等に委譲。
- `make verify-big-integer` / `python3 verify/run_big_integer.py --sanitize`。単独／重複include、全カテゴリ混在と複数翻訳単位、使用例、65,536小整数ペア、変換極値・不正入力・自己演算・FastIO・Matrix・多項式作用、Python任意精度比較。`--benchmark`は合否閾値のない参考計測。
- 図解は基数1000の筆算やEuclid法の説明モデルで、Boostの内部トレースではない。4,200比較・90境界・4つのC++参照値とブラウザ操作を検証する。オンラインACは未確認。

今回の実行結果：通常・ASan/UBSan/LeakSanitizerのDebug/Release、単独／全カテゴリinclude・複数翻訳単位、65,536小整数ペア、Python5,207出力、使用例、4つのC++参照値が成功。図解モデルと4モード・5編集入力・390pxのブラウザ検証が成功し、desktop/mobile画像も確認しました。文書リンク1,324件に欠落なし。Boost 1.74・GCC 11.4で参考性能も計測済み。既知のsocket/ptrace制限に対応するため、Chromiumとサニタイザは承認レビュー付きで制限外から実行しました。

## 共通基盤 F-12〜F-17（2026-10-19）

[6項目のAPI・使用例・検証](docs/foundation_tools/README.md) / [一次資料](docs/foundation_tools/references.md) / [6モードの図解](visualizations/foundation_tools/index.html)。basic_utility/all.hppに追加。

- F-12 BigRationalは既存BigIntを共有。分母正・既約・0/1、交差約分、四則演算・比較・n/d入出力、分母0・0除算のdomain_error。整数文字列もparseでき、出力は常にn/d。Boostヘッダ依存。
- F-13 Arena<T,BlockSize>は型付き固定サイズblock。追加時にID/参照を維持し、整列を守る。clearは逆構築順に破棄・全block解放・ID再利用、全参照無効。コピー／ムーブ／個別解放はなし。401版の永続木への接続を検証。
- F-14 uint64のtopbit/lowbitは位置、0なら-1。次の同popcountはoptional、FixedPopcountMasksは0≤k≤n≤64を遅延列挙し、64bit端を処理。
- F-15 GridShape・4/8近傍・回転／転置を共有。grid_bfsの探索は保持し、矩形・始点・セル数を例外で検査。0行の列数をvectorで保持できない規約を説明。
- F-16 RollbackArrayはコピー可能・nothrow swap可能なTを保持。bool対応。snapshotはowner／epoch／履歴通番で枝を検査。巻き戻した枝は同じ深さまで更新しても無効。コピー／ムーブ不可、付随情報はTへまとめる。
- F-17 安定LSDは8〜128bit整数キーを1回評価して保存し、byteごとに添字を整列、最後にレコードをコピーしてswap。Tはコピー構築可能、signedは符号bit反転。
- 検証入口は `make verify-foundation-tools` / `python3 verify/run_foundation_tools.py --sanitize`。7ヘッダ、6usage、全カテゴリ混在／複数翻訳単位、有理数73,984組・20万Arena・16bit全列挙・全512盤面×9始点・1万Rollback操作・10型1500整列ケース、Python1,916比較。オンラインACは未確認。

今回の実行結果：通常・ASan/UBSan/LeakSanitizerのDebug/Release、6使用例・6つのC++図解参照値、Python1,916出力が成功。図解モデル300ランダムバッチ・55全列挙クラス、6モード／6編集入力のブラウザ操作、390px表示が成功し、Arena・Grid・Radix・mobile画像も確認しました。文書リンク1,414件に欠落なし。既存graph/treeの53単独ヘッダ・3使用例・比較検証・互換入口・深い鎖の回帰検証も成功。既知のsocket/ptrace制限のためChromiumとサニタイザは承認レビュー付きで制限外から実行しています。

## 開発・検証基盤 V-01〜V-04（2026-10-19）

[API・使用例・検証](docs/development/README.md) / [一次資料](docs/development/references.md) / [4モードの図解](visualizations/development/index.html)。Python 3.10以上の開発用ツールで、C++ライブラリのPython移植とは別です。

- V-01: competitive-verifier 4.1.2と推移依存を固定し、GitHub Actionsのlocal/online jobと文書artifact生成を追加。オンライン対象は既存Union-Find・Point Add Range Sum・Suffix Arrayの3ラッパ、別にFastIO/BigInt/RollbackArrayのstandalone。全ラッパ登録・全体台帳はV-09の範囲。`tools/verify_ci.py --offline` / `--resolve-only` / 無指定で実行範囲を選ぶ。
- V-02: `python3 -m tools.cp_tools.stress`。JSON argv・cwd・timeout・exact/tokens比較、最初の失敗をseed/input/stdout/stderr/終了状態ごと保存し、保存入力でreplay。POSIXでは子process groupも終了させる。環境やバイナリのsnapshot、出力量制限は持たない。
- V-03: `tools.cp_tools.random_data`の固定SplitMix64、木・森・成分数指定/多重グラフ・DAG・凸な単純多角形。全構造の一様分布ではなく、グラフ候補列挙はO(n²)。成分数指定は大成分1つと孤立点、polygonは放物線上の凸多角形。
- V-04: `python3 -m tools.cp_tools.bundle SOURCE --check`。リテラルlocal include、guard・重複・循環・条件付き経路、展開後compile、依存一覧。システムincludeは保持し、マクロinclude/include_nextは非対応。BigInt/BigRational使用時は提出先にもBoostヘッダが必要。
- 検証入口: `make verify-development` / `python3 verify/run_development.py --sanitize`。5実プログラムの元/展開後をDebug/Releaseで比較、80 seedのC++ DSU対Python探索、生成器・保存/replay・timeout等の単体検証、Python図解参照200件を確認。固定版verifierの接続は仮想環境で `python verify/run_development.py --with-verifier`。必要ファイルを/tmpのGit snapshotにコピーし、実indexは変えない。
- 図解: `node verify/visualizations/development_model_test.js` / `node verify/visualizations/development_browser_test.mjs --browser /path/to/chromium`。4モードと入力編集、エラー、再生、390px表示。

今回の実行結果：通常・ASan/UBSan/LeakSanitizer、5組のDebug/Release展開比較、80 seed、200図解比較、ブラウザ4モードが成功。固定版verifierのstandalone実行・文書生成・3問題の依存解決も成功。GitHub Actions上の実行、オンライン全ケース検証、サイト公開は行っておらず、オンラインACとは扱わない。Chromiumとサニタイザは既知の制限に従い承認レビュー付きで実行。Nodeの同期子プロセスがsandboxのEPERMで停止したため、図解参照はPythonで検証するfixtureを使う形にした。

## 実行・観察ツール V-05〜V-07（2026-10-20）

[API・使用例・検証](docs/development_runtime/README.md) / [一次資料](docs/development_runtime/references.md) / [3モードの図解](visualizations/development_runtime/index.html)。C++デバッグ出力はbasic_utility/all.hppへ追加。Pythonツールは引き続きライブラリのPython移植とは別です。

- V-05: `cp::debug_string`、`CP_DEBUG`、`CP_DEBUG_TO`。コンテナ・pair/tuple/array・文字列・128bit・optional/variant・独自formatter/ostream型を表示し、深さ/要素数/文字列長を制限。CP_ENABLE_DEBUGのみで有効化し、CP_DISABLE_DEBUGまたはONLINE_JUDGEで無効化。無効時は引数の評価・型検査なし。NDEBUGとは独立。
- V-06: `python3 -m tools.cp_tools.interactive`。POSIXのnonblocking双方向pipeとEOF伝達、wall/idle timeout、出力量上限。通信をbase64のJSONL、stdout/stderrを別ファイル、終了状態をresult.jsonへ保存する。既存保存先は上書きしない。数字当てのC++ solver/Python judge例あり。
- V-07: `python3 -m tools.cp_tools.benchmark`。生成器はseedごとに1回、各設定へ同じ入力を渡して外部solver/scorerを並列実行。正規表現で有限scoreを1つ取得し、成功分の統計と全seed成功の比較対象を区別する。同点は設定順、平均/中央値は有理数経由でoverflow回避。ケース保存・入力固定replay・timeout・キャンセル付き。数列の先頭/最大値を選ぶ比較例あり。
- 前回stressのResult/executeを `tools/cp_tools/process.py` へ共有し、cancelledフラグと任意のキャンセルEventを追加。従来のstressからのimport/APIは維持。POSIXはprocess group、Windowsは直接の子を終了させる。対話pipe自体はWindows非対応。
- 検証入口: `make verify-development-runtime` / `python3 verify/run_development_runtime.py --sanitize`。Python15ケース＋既存stress5ケース、4マクロ設定×Debug/Release、単独・全カテゴリ混在・2翻訳単位、usage、500個の独立整形比較。9対話セッション、2設定×12 seedを4並列、保存入力からの独立スコア・replayを検証する。
- 図解: `node verify/visualizations/development_runtime_model_test.js` / `node verify/visualizations/development_runtime_browser_test.mjs --browser /path/to/chromium`。C++6整形参照、Python40集計参照、3モードの編集・無効化・timeout・エラー・再生・390px表示。

通常検証の実処理、ASan/UBSan/LeakSanitizerの全runnerと参照比較は成功。通常runner末尾のリンク検査は文書作成中に失敗した後、修正して別途成功。Chromiumで3モードの操作・入力編集・エラー・再生・390px表示も成功し、デスクトップとモバイルのスクリーンショットを目視確認済み。Chromiumとサニタイザは既知の制限に従い承認レビュー付きで実行。CIのlocal jobにも新runnerと図解モデルを追加しましたが、GitHub Actions上の実行・オンラインACは未確認です。続くV-08〜V-10は下記を参照。

## 出典・検証台帳と共通検証モード V-08〜V-10（2026-10-20）

[使用方法・対応範囲・検証](docs/development_records/README.md) / [出典JSON](verification/provenance.json)。新しいC++ APIの追加はありません。

- V-08: `python3 -m tools.cp_tools.provenance`。移植・旧帰属・Boost依存の8レコード/10ファイルを追跡し、コード・ライセンス原文のSHA-256、重複、欠落、未登録帰属を検査。ACLの取り込みcommitは既存記録を使用。他の過去の取り込み版や旧2件のライセンスは未確認として保持。今回確認した上流commit/元コードのSHA-256を別フィールドに保存。全体ライセンスの設定や全コードの出典監査完了とは扱わない。
- V-09: `python3 -m tools.cp_tools.catalog`。導入当時の583ヘッダ（検証補助も含む）・304 C++検証ソース・全43runnerとmanifest/PROBLEM/include到達関係を集計。ソースfingerprintと一致するローカル成功、明示的な提出AC証拠、それ以外の未verifyを区別する。検索・状態絞り込みができるHTMLとJSONをbuild/catalogへ生成。
- V-10: `python3 -m tools.cp_tools.suite --stdlib-debug --runners NAME...`。全runnerを共通選択し、libstdc++ debug mode・既存サニタイザを併用可能。起動前の実コンパイル検査、runner単位timeout/process group終了、モードごとの最新結果とログ保存。新しい検証記録はverification/evidence以下へ保存し、コード変更時には失効する。
- 順序統計・オフラインクエリのstressのみ、debug modeでは2,000要素へ縮小。通常モードの20万要素は維持し、同じ操作と境界を検査する。V-01〜V-07の2つのrunnerも複合compiler argvを扱えるよう統一。
- 検証入口: `make verify-development-records`。14テストで出典・ハッシュ・失効・提出AC証拠・timeout・HTMLエスケープを検査し、無効イテレータがNDEBUG有無の両方で検出されることと2翻訳単位のコンテナ受け渡しを確認。全43runnerの共通オプションも確認する。
- HTML検証: 台帳生成後に `node verify/development_records/browser_test.mjs --browser /path/to/chromium`。検索、3状態、0件、390px表示とdesktop/mobile画像。CIのlocal jobに台帳・debug modeとartifact保存を追加。

今回の実行結果：新台帳の14テストと全43runnerの共通オプション検査が成功。Union-Find・Range Query・Ordered Structure・Offline Query・Foundation Tools・Development・Development Runtime・B-21〜25・Development Recordsの9runnerでlibstdc++ debug modeが成功。Ordered StructureとOffline Queryは通常モードの20万要素も成功。Union-Findはdebug mode＋ASan/UBSan/LeakSanitizer併用も成功。V-10完了時の12件の実行記録は当時のソースfingerprintに一致しました。以降のソース追加で失効するため、現在の一致状況は台帳から確認してください。台帳のブラウザ検索・3状態・0件・390px表示も成功し、desktop/mobile画像を確認済み。文書リンク・進捗件数・workflow構文・diff検査も成功。GitHub Actions上の実行と実際の提出ACは未確認です。ChromiumとLeakSanitizerは既知の制限に従い承認レビュー付きで実行しました。

ユーザーの「V-10まで実装して一旦コミット」に従い、以前の未コミット分F-11〜F-17・V-01〜V-07も含めて保存します。この時点での次項目はU-01（尺取り法）でした。V-10までの作業はコミット6979196で保存済みです。

## ユーティリティ U-01〜U-11・U-20〜U-22（2026-10-20）

[14項目のAPI・使用例・検証](docs/utility_search/README.md) / [出典](docs/utility_search/references.md) / [入力を操作できる図解](visualizations/utility_search/index.html)。basic_utility/all.hppに14ヘッダを追加しました。

尺取り・Kadane・動的MEX・半分全列挙・順列演算・黄金分割/Fibonacci探索・集合hash・順列木・括弧列・整数分割・集合分割を実装。U-20は最新版の点代入・過去版参照・最初の成立時刻を扱い、既存部分永続DSUにも接続しました。U-21は事前登録時刻上のMonoid操作の追加・置換・削除・任意時点の合成を既存Segtreeで実現。U-22はuint64の回文・数字制限・指定因子の積の昇順列挙です。U-12〜U-19の行はなく、計14作業単位。U-22完了時点では残作業204、整備済み41、次はD-01（SWAG Deque）です。この時点でD-31のRetroactive Priority Queueは未実装の別項目でした（2026-10-24に整備済み）。

検証入口は `make verify-utility-search`、共通suiteの `--runners utility_search`、図解モデルとChromiumのbrowser test。順列木はn≤8の全順列と20万要素、括弧列は全候補と深さ10万、他は全区間・bitmask・再帰生成・全履歴コピーと比較します。追加2項目はstd::mapの再生・20万時刻・全数字集合・因子集合・uint64極値で確認します。libstdc++ debug modeの時刻数だけは二分探索の線形な前提検査を考慮して2000へ縮小し、通常・サニタイザの20万時刻は維持します。実行結果と最新の証拠は[検証記録](docs/utility_search/README.md)を参照してください。

今回の実行結果：通常／Release、libstdc++ debug mode、ASan・UBSan・LeakSanitizer、14使用例・14件のC++/図解参照比較、図解モデル・ブラウザ14モード／16変更入力、390px表示、文書リンク1610件が成功。U-22完了時点の3構成の実行記録は当時のソースに一致しました。D-01の追加で失効するため、現在の一致状況は台帳で確認してください。台帳は597ヘッダ・307 C++テスト・44runner。ChromiumとLeakSanitizerは既知の制限に従い承認レビュー付きで実行し、追加2モードのdesktopとmobile画像も確認しました。GitHub Actions上の実行とオンライン提出ACは未確認です。

ユーザーの「U-22まで実装した後コミット」に対応したコミット対象は、以前の未コミット分U-01〜U-11・U-20と今回のU-21・U-22、説明・使用例・検証・図解・索引です。

## SWAG Deque D-01（2026-10-20）

[API・使用例・検証](docs/static_query/swag_deque.md) / [両端操作と再構築の図解](visualizations/static_query/index.html#swag_deque)。`static_query/swag_deque.hpp`と同カテゴリのall.hppに追加。今回はD-01のみを実装しました。

`SWAGDeque<S,op,e>`の両端push/popはならしO(1)、constなfront/back/prodはO(1)。片側が空なら半分ずつに再構築して、非可換な順序も保持します。空の積はe、逆元不要、空のpop/front/backはassertの前提検査。再構築途中の例外では元の内容を保持します。

検証入口は既存の `make verify-static-query` と共通suiteの `--runners static_query`。長さ8までの全操作履歴1,092,253状態、24万ランダム操作、非可逆なaffine・非デフォルト構築型・自己参照・例外、20万要素と交互削除の演算回数を確認します。既存S項目と一緒に4使用例・6ジャッジ形式・図解6モードを検証します。通常／Release・libstdc++ debug mode・ASan/UBSan/LeakSanitizer、図解モデル218,753操作と6モードのブラウザ・390px表示が成功。D-01完了時の3件の実行記録は当時のソースfingerprintに一致し、文書リンク1430件、出典・台帳・進捗件数も確認しました。ChromiumとLeakSanitizerは既知の制限に従い承認レビュー付きで実行しています。台帳は598ヘッダ・310 C++テスト・44runner。詳細な実行結果は個別説明を参照してください。D-01完了時は残作業203、整備済み42、次はD-02でした。D-01はコミット4bf98ccに保存済みです。D-02〜D-10の変更により古い実行記録は失効するため、現在の一致状況は台帳を確認してください。

## データ構造 D-02〜D-10（2026-10-24）

[9項目のAPI・使用例・検証](docs/data_structure_extensions/README.md) / [一次資料・設計判断](docs/data_structure_extensions/references.md) / [9モードの図解](visualizations/data_structure_extensions/index.html)。ユーザーの「D2〜19」に対応し、一覧にあるD-02〜D-10を整備しました。D-11〜D-19の行はありません。

- D-02〜D-05: multiset共有の両端キュー、prefix最終出現位置の永続化によるRange MEX、相殺候補と頻度再検証のRange Majority、標準区間のmapによるオンラインPoint Set Range Frequency（O(log² n)）。
- D-06: pool所有・int rootの汎用PersistentLeftistHeap。既存LeftistHeapと同じ比較方向（lessで最大優先）。線形buildと永続meld/push/popを分け、k_shortest_walksの内部実装を共有化。
- D-07: 既存DynamicSegtreeを不変ノード共有へ拡張。コピーO(1)、座標を保持するsplit_copy/merged、入力を消費するsplit/merge。重複点のcombine、明示的なe()の点、所有権と例外時保持を定義。
- D-08/D-09: 左から右へ標準ノードを列挙するSegmentTreeCoverと、任意Stateのsnapshot/apply/rollbackを受けるSegmentTreeOverTime。OfflineDynamicConnectivityも共有。適用順は全体の登録順ではない。apply/visitの例外でもStateを復元する。
- D-10: 通常/rollback DSUの併合直前callback、任意Dataを借用して同時復元するRollbackDisjointSetUnionWithData。旧2引数APIは保持。Data自身の履歴契約、古い枝・別個体snapshot、途中例外を検証。

検証入口: `make verify-data-structure-extensions`、共通suiteの `--runners data_structure_extensions`、同名の図解model/browser test。単独/重複include、全カテゴリ混在2翻訳単位、9使用例・C++/図解9サンプル、2ジャッジ形式×12独立Python入力、全探索401,041区間・版分岐・20万要素と関連4テストの回帰を含みます。

ブラウザ9モード・5変更入力・390px表示とモデル2,718比較が成功し、desktop/mobile画像も確認しました。通常52.6秒・stdlib-debug76.8秒・ASan/UBSan/LeakSanitizer93.6秒の全構成が成功し、3つの記録は当時のソースfingerprintに一致しました。D-20〜D-35追加後の再実行記録も同じ場所に保存します。[検証記録](docs/data_structure_extensions/README.md)を参照。台帳は606ヘッダ・316 C++テスト・45runner。ChromiumとLeakSanitizerは既知のsocket/ptrace制限に従って承認レビュー付きで実行しました。CI設定は更新しましたがGitHub Actions実行・オンラインACは未確認です。

D-02〜D-10完了時点の進捗は追加整備済み51、残194。今回のD-35までのコミットには、この未コミット分も含めます。

## データ構造 D-20〜D-35（2026-10-24）

[16項目のAPI・使用例・検証](docs/data_structure_advanced/README.md) / [出典・設計判断](docs/data_structure_advanced/references.md) / [16モードの図解](visualizations/data_structure_advanced/index.html)。ユーザーの「D-35まで実装してcommit」に従い、前回のD-02〜D-10と今回の16項目をまとめてコミットします。

非可換群DSU、ETTと代替辺走査のオンライン動的連結、整数時刻のKinetic、RBSTの動的列、floor除算Beats、Mode/LIS/Inversions、動的Waveletとbit列、永続Finger Tree、bridge法のRetroactive Priority Queue、完全永続Lazy RBST、1〜10次元のbox/最近傍/Fenwickを追加。D-23/24/29/34は、ユーザーが承認した操作として仕様化しました。D-21の木辺削除とD-34の最悪問い合わせは線形、D-29は非圧縮、D-30の両端操作はstrict評価で最悪O(log n)です。D-26はtko919/libraryの固定commitをCC0で改変し、ライセンス・出典・hashを台帳へ追記しました。

検証入口は `make verify-data-structure-advanced`、共通suiteの `--runners data_structure_advanced`、同名の図解model/browser test。16ヘッダ、16使用例とC++/図解参照、2ジャッジ形式×12入力、全列挙・素朴解・版分岐・大きい入力を検証します。前回の `data_structure_extensions` も同じソースで再実行します。最新の結果は[検証記録](docs/data_structure_advanced/README.md#検証)を参照してください。

通常50.0秒・stdlib-debug91.4秒・ASan/UBSan/LeakSanitizer394.3秒でDebug/Releaseを通過しました。前回分も通常42.7秒・stdlib-debug73.3秒・サニタイザ85.3秒で再確認し、6記録は同じ完了時ソースfingerprintに一致します。図解モデル3,016比較、ブラウザ16モード・5変更入力・390px表示も成功し、desktop/mobile画像を確認しました。関連リンク1,908件・出典10記録・12追跡ファイル・進捗245行・workflow構文を確認。台帳は622ヘッダ・320 C++テスト・46runner。ChromiumとLeakSanitizerは既知の制限に従い承認レビュー付きで実行しました。GitHub Actions実行とオンラインACは未確認です。

D-35完了時点の進捗は追加整備済み67、残178（未実装130、一部対応25、整備待ち5、要仕様化18）。Python移植は未着手です。

## 木の追加項目 T-01〜T-03・T-20〜T-23（2026-10-24〜25）

- T全7項目を実装。概要は[対応表と検証](docs/tree_extensions/README.md)、[一次資料](docs/tree_extensions/references.md)。
- T-01は重心と別の木の中心。非負重みでは頂点中心と連続中心を別に返し、整数の半整数位置を2倍表現で保持。T-02は明示nのPrüferでn=1/2を区別。T-03は既存HLD順の高々2区間、森の成分境界を保持。
- T-20/T-21はLuzhiledの版固定Unlicense coreを共有。preferred pathとlight枝をsplayで維持し、link/cut・任意根・spine/off-spineのlazyを扱う。RAII、rakeノード再利用、祖先からpush、0始まりのbool/optional APIに変更。[台帳](verification/provenance.json)に登録。
- T-22は既存可換ActedMonoidの部分木prod/apply、パス・成分も同じ実体。T-23は汎用finish付きDPと独立集合・根依存距離和。逆元を要求せず、代入/affineと合成数mod/零を検証。
- 入口は `make verify-tree-extensions`、`python3 verify/run_tree_extensions.py --sanitize`、共通suiteの `--stdlib-debug --runners tree_extensions`。通常/ASan/UBSan・Debug/NDEBUG、単独/重複include、全カテゴリ2翻訳単位、usage7本、LC2形式×独立12入力、15万動的操作、15000 DP段階、10万/20万頂点を検査。
- [図解7モード](visualizations/tree_extensions/index.html)はモデルを独立比較し、ブラウザ操作・入力編集・390px表示を確認。C++のsplay回転列は再現せず、clusterの意味と作用範囲を表示する。オンラインACは未確認。
- 最終確認: 通常・stdlib-debug・ASan/UBSan/LeakSanitizerのDebug/NDEBUGが成功。既存 `run_tree.py` と `run_b_7_11.py` も成功。図解は400動的トレース・89704スナップショット、ブラウザ7モード・3編集入力・390px表示を検証。関連リンク1576件、出典11レコード・13追跡ファイル、進捗245行の件数も確認。
- [通常実行の記録](verification/evidence/tree_extensions/standard.json)は完了時のソースfingerprintに一致（31.9秒）。新しい公開7ヘッダとcoreは検証台帳でローカル検証済み。CIにもrunnerと図解モデルを登録しました。GitHub Actionsの実行・オンラインACは未確認。
- T完了時点では残作業171、追加整備済み74、次はG-01でした。今回のG追加後の状態は次節を参照。

## グラフ G-01〜G-15（2026-10-25）

[API・使用例・検証](docs/graph_extensions/README.md) / [一次資料とライセンス](docs/graph_extensions/references.md) / [15モードの図解](visualizations/graph_extensions/index.html)。ユーザーの指定範囲G-01〜G-15を実装しました。

- 区間辺グラフの元頂点/操作ID対応、二部最小頂点被覆と独立集合、一般/二部の最小辺被覆、DinicによるGomory–Hu、非再帰st順序、分数厳密なKarp平均閉路、次数列の判定/構成。
- Borůvkaの成分別最小辺oracleと全uint64のXOR-MST、LCTで辺を交換する増分MSF（辺ノード再利用）、Cactus共有のなもり、推移閉包上の反鎖、一般単純グラフΔ+1辺彩色。
- 差分制約と全成分/始点指定の負閉路は128bitの共通証拠core。最小費用素パスは非負整数費用、指定本数、内部頂点/辺の素性を定義し元辺IDを復元。負費用一般への拡張はしません。
- VizingはKACTL固定commitのCC0。C++と図解の改変・hashを出典台帳に登録。st順序はstd::listと反復DFSで実装し、ライセンス未確認の比較対象コードは取り込んでいません。
- `make verify-graph-extensions`、共通suiteの`--runners graph_extensions`、同名の図解model/browser testが入口。単独include、2翻訳単位、15使用例、全探索・独立解・5万頂点・極値を確認。詳細は上の検証ページへ。
- LCTの`add_vertex(value)`を追加。既存IDを保持し、既存APIは同じままです。この時点ではG-16以降とPython版は未着手でした。
- 最終検証: 通常47.3秒・stdlib-debug67.8秒・ASan/UBSan/LeakSanitizer87.8秒のDebug/NDEBUGが成功し、3つの実行記録は完了時ソースに一致。既存run_tree.pyの回帰、図解1,898トレースとブラウザ15モード/4変更入力/390px表示、出典12レコードも確認。ChromiumとLSanは既知の制限により承認レビュー付き。オンラインAC・GitHub Actions実行は未確認。
- G-15完了時点は残156、追加整備済み89でした。現在は次節を参照。

## グラフの残り G-16・G-20〜G-33（2026-10-25）

[API・仕様・検証](docs/graph_advanced/README.md) / [一次資料とライセンス](docs/graph_advanced/references.md) / [15モードの図解](visualizations/graph_advanced/index.html)。ユーザーの「G16-33（Gの残り全て）実装してコミット」に対応。G-17〜G-19の行はなく、G全30項目が整備済みです。

- Cost Scaling b-flow、k種類重みFIFO、任意群のShortest Nonzero Path、連続中心から最小直径全域木、分離対探索SPQR、最小木幅subset DPとmin-fill、cograph標準形。
- 重み付きBlossom、BigIntによるrank-maximal、辺の有効化/無効化を伴う動的有向単位容量流。連結/強連結グラフ係数、BESTの根/先頭辺指定、一般同型の厳密判定、有向二端子SP式木。
- 仕様化4項目は少種類重み・cograph・単位容量更新・二端子直並列。SPQRは線形時間版でなく分離対全探索、最小木幅はn<=22、一般同型は最悪指数時間。FastSUPの距離計算は高速版、復元は最悪O(n²)。各APIの条件を参照。
- Boost 1.74の旧weighted matchingでLSanリークを検出し、Boost Graph 1.89 commit `54a94911e68dccf5812b32d018566566700499f3` のcoreを隔離して取り込み。BSL本文・原hash・変更点・ローカルhashを出典台帳に登録。基本型にはシステムBoostヘッダが必要。
- `make verify-graph-advanced`、共通suiteの `--runners graph_advanced` / `--stdlib-debug` / `--sanitize`、同名図解model/browser testが入口。15使用例、16単独ヘッダ、全カテゴリ2翻訳単位、全探索と独立解、2万頂点、整数極値を検証。
- 最終結果: 通常57.4秒・stdlib-debug79.6秒・ASan/UBSan/LeakSanitizer104.1秒のDebug/NDEBUG成功。3記録は完了時同一ソースfingerprintに一致。前回G-01〜G-15の通常回帰43.7秒も成功。図解1,016トレース・ブラウザ15モード/6変更入力/390pxと画像確認が成功。Chromium/LSanは既知制限に従い承認レビュー付きで実行。オンラインACとGitHub Actions実行は未確認。
- 現在は残141（未実装104、一部対応18、整備待ち5、要仕様化14）、追加整備済み104、追跡245。掲載順の次はP-01。Python移植は未着手。台帳は664ヘッダ・331 C++テスト・49runner。

## 作業場所・Git

```text
/home/marc2825/procon/             # IDE のワークスペース
  main.cpp, test.cpp, input.txt …  # ユーザーの問題を解く作業ファイル
  ac-library/                    # 外部 ACL。新実装の実行依存にはしない
  cp_library/                    # このライブラリの Git リポジトリ
```

今回の作業ディレクトリは `/home/marc2825/cp_library` です。Git コマンドは `cp_library/` を作業ディレクトリにして実行します。外側のワークスペースの `.git` を操作しません。開始時に必ず `git status --short` と `git branch --show-current` を確認し、既存の未コミット変更を保持します。`main.cpp`、`test.cpp`、入出力ファイルを検証用に上書きしません。

この文書を書いた時点のブランチは **`ai/generated-libraries`** です。

- `7fc6034`: グラフ・木の機能別分離（元ブランチ `refactor/graph-tree-library`）。
- `3bd6a08`: AI 生成ライブラリ用の基準点、既存実装と説明フォルダの準備。
- `06feda5`: 3.2〜3.10 の実装・説明・図解・検証と引き継ぎ書のスナップショット。
- `ce24f45`: 3.11〜3.14 の実装・説明・図解・検証、索引と引き継ぎ書のスナップショット。
- `bbd20a6`: 3.15〜3.18 の実装・説明・図解・検証、索引と引き継ぎ書のスナップショット。
- `f6f5ece`: 3.1〜3.6のA補完、33公開ヘッダ・図解・検証・予定リスト更新を保存。
- `064483e`: 3.19〜3.25の実装・説明・図解・検証を保存。
- `7f9d976`: 高度な20項目の不足分21ヘッダと説明・図解・検証、全実装予定リストとA補完方針を保存。
- `65cccb0`: 3.21〜3.25の不足A10項目と説明・図解・検証を保存。
- `8bb00e0`: 3.7〜3.20の残りのA補完・共通索引・全A完了の進捗を保存。
- `bc003e1`: 3.1〜3.11のB全79掲載箇所とPython版作成計画を保存。
- `12236e4`: 3.12〜3.15のB全30掲載箇所を保存。
- `9067220`: 3.16〜3.25のB全59掲載箇所・説明・図解・検証を保存し、B全168掲載箇所を完了。
- `dcbd9b9`: F-01〜F-10の共通基盤・説明・使用例・検証・図解を保存。

この Git 状態は今後変わるため、実際の状態を優先します。commit・push・ブランチ変更は、そのときの依頼の範囲で行います。

## 追加するファイルの構成

```text
<category>/algorithm.hpp
<category>/all.hpp                 # カテゴリのヘッダをまとめる入口
docs/<category>/README.md          # 選び方・API への索引・検証方法
docs/<category>/algorithm.md       # 同名の個別説明
docs/<category>/references.md      # 調査した一次資料、出典、設計判断
examples/<category>/*.cpp          # コンパイルして動く最小例
verify/<category>/unit_test.cpp    # 独立した素朴解との比較
verify/<category>/*.test.cpp       # PROBLEM を指定したジャッジ形式
verify/run_<category>.py           # カテゴリのローカル検証入口
visualizations/<category>/         # インタラクティブ解説
verify/visualizations/             # 図解モデルとブラウザの検証
```

各説明の先頭:

```yaml
---
title: アルゴリズム名
documentation_of: //<category>/algorithm.hpp
---
```

`//` はこの Git リポジトリのルートです。将来の verification-helper 系ドキュメント生成を想定したメタデータです。**公開サイトの自動生成・デプロイはまだ構築していません。** GitHub 上で .hpp を開くだけで .md が自動表示される仕組みは未実装です。

追加時には次の索引も更新します。

1. ルート `README.md` のカテゴリ表・検証案内。
2. `docs/README.md` のカテゴリ一覧。
3. `visualizations/index.html` のカードと `visualizations/README.md`。
4. `Makefile` のカテゴリ検証ターゲット・`verify-visualizations`。
5. このファイルの現状・検証方法。

## C++ の設計方針

新カテゴリは C++17、`namespace cp`、0 始まり・半開区間 `[l,r)` を基本にします。各ヘッダは単独 include 可能、標準ライブラリ以外はリポジトリ内の明示的な include だけにします。`ll`、マクロ、先行 include、`using namespace std` に依存しません。

型・関数名は既存の新カテゴリに揃え、汎用化で使い方を過度に複雑にしないでください。必要な前提、戻り値、空入力や境界の扱い、整数の中間値の範囲を明記します。テンプレート演算の計算量も説明します。

旧 API は移行用ヘッダ／別名を残す方針です。公開フィールドまで使える旧実装は、単純な型別名への置換で壊れることがあります。既存の使用箇所を検索して判断し、変更理由と移行方法を説明します。古いカテゴリすべてを一度に整備する必要はありません。

## 現状

| 分野 | 実装・説明 | 主な注意点 |
|---|---|---|
| 開発・検証基盤 V-01〜V-10 | `tools/`, `.competitive-verifier/`, `.github/workflows/`, `docs/development/`・`docs/development_runtime/` | 固定版verifier・CI、stress・データ生成・バンドラ、C++デバッグ出力・対話judge・AHC複数seed比較。出典・検証台帳と共通libstdc++ debug modeも整備。docs/development_recordsを参照。U-01〜U-22は下記。 |
| グラフ・木 | `graph/`, `tree/`, `docs/graph_tree.md` | 機能別に分離済み。旧 `graph_theory/` は互換入口。新 `Graph` は既定で無向、論理辺 ID を両向きで共有。HLD は方向付き半開区間を返す。基本グラフの図解は 3.7、連結性・Lowlink は 3.8、追加Aはgraph_aに整備。木のS項目の図解は3.11に整備。Block-Cutの図解は今回のgraph_aへ追加。 |
| 3.1 基本ユーティリティ | `basic_utility/`, `docs/basic_utility/`, `visualizations/a_priority/`・`b_1_6/` | F-01〜F-08のmodint・高速剰余・FastIO・安全演算・seed付きHash Mapを追加。F-11/12のBigInt／BigRationalはBoostヘッダ依存。F-12〜F-17のArena・bit・grid・Rollback・radix等も追加。図解はfoundation・foundation_structures・big_integer・foundation_tools。従来Aに加え、catalog FC・uint64整数根・Stern–Brocot・分数境界・平方分割・非単調分枝限定・共通RangeSet。 |
| 3.2 Union-Find | `union_find/`, `docs/union_find/`, `visualizations/union_find/`・`b_1_6/` | 通常/重み/永続等に加え、区間対Parallel・動的ラベル・成分/点加算・増分橋・共通Offline Dynamic Connectivity。重み制約は `potential[v]-potential[u]=w`。 |
| 3.3 セグメント木・Fenwick | `range_query/`, `docs/range_query/`, `visualizations/range_query/`・`b_1_6/` | F-10のMonoidカタログ・等差／多項式作用を追加。従来版に加えRollback・永続Lazy・一意keyのSortable（点更新可）・Range Tree・FC・可換2D Dual。composition(f,g) は g の後に f。 |
| 3.4 静的クエリ・順序統計・集合管理 | `static_query/`, `docs/static_query/`, `visualizations/static_query/`・`b_1_6/` | 重み和Wavelet Matrix・Wavelet Tree・永続Trie・Priority Sum・動的bitset・rank/select・線形RMQ・静的Sqrt Treeを追加。 |
| 3.5 平衡二分探索木・永続データ構造 | `ordered_structure/`, `docs/ordered_structure/`, `visualizations/ordered_structure/`・`b_1_6/` | Splay・左傾Red-Black・再構築型Weight-Balanced・永続Treap・Skew/Leftist・削除予約heap。LazyとRadixは既存共有。 |
| 3.6 オフラインクエリ・二次元データ構造 | `offline_query/`, `docs/offline_query/`, `visualizations/offline_query/`・`b_1_6/` | 矩形加算と和/点取得・Kd木・共通Range Tree・Hilbert Mo・種類数・オンライン点追加と矩形affineを追加。 |
| 3.7 グラフ：基本探索・最短路・MST | `graph_basic/`, `docs/graph_basic/`, `visualizations/graph_basic/`・`b_7_11/` | F-09のCSR Graphを共有graphに追加。BFS、DFS、0-1、Dijkstra、Bellman–Ford、Warshall–Floyd、トポロジカル順、閉路、Kruskal、Prim、経路復元。 |
| 3.8 グラフ：連結成分・構造分解 | `graph_connectivity/`, `docs/graph_connectivity/`, `visualizations/graph_connectivity/` | SCC・縮約 DAG・Lowlink・橋・関節点・二部グラフ・無向連結成分。 |
| 3.9 グラフ：フロー・マッチング・最適化 | `graph_flow/`, `docs/graph_flow/`, `visualizations/graph_flow/` | Dinic・最小カット復元・Hopcroft–Karp 二部最大マッチング。 |
| 3.10 グラフ：組合せ構造・数え上げ | `graph_combinatorics/`, `docs/graph_combinatorics/`, `visualizations/graph_combinatorics/` | 三角形・全／極大クリーク、最大クリーク／独立集合、厳密彩色 DP、行列木定理。 |
| 3.11 木アルゴリズム | `tree/`, `docs/tree/`, `visualizations/tree/` | LCA・直径・Euler Tour・部分木／深さ・全方位木DP・Binary Lifting・Tree DP。既存木APIを保持。 |
| 3.12 DP・最適化 | `dp_optimization/`, `docs/dp_optimization/`, `visualizations/dp_optimization/` | LIS・LCS・編集距離、3種ナップサック・Bitset、区間／桁／部分集合DP、Largest Rectangle。追加AはD&C/Knuth・CHT/Li Chao DP・Slope Trick・WQS。 |
| 3.13 数論：基本整数演算 | `number_theory_basic/`, `docs/number_theory_basic/`, `visualizations/number_theory_basic/` | GCD/LCM・拡張Euclid・mod累乗／逆元・素数篩・SPF・素因数分解・約数列挙、φ/μ・CRT/Garner・floor_sum・素数原始根。 |
| 3.14 数論：乗法的関数・高度な整数問題 | `number_theory_convolution/`, `docs/number_theory_convolution/`, `visualizations/number_theory_convolution/` | 約数／倍数zeta・mobius、Dirichlet・GCD・LCM畳み込み。 |
| 3.15 組合せ論・数え上げ | `combinatorics/`, `docs/combinatorics/`, `visualizations/combinatorics/` | 階乗・逆階乗、二項係数、順列／組合せ、Catalan・重複組合せ、包除原理、Stirling/Bell/分割/Montmort・Burnside/Pólya。 |
| 3.16 畳み込み・高速変換 | `convolution/`, `docs/convolution/`, `visualizations/convolution/` | NTT・通常のmod／整数畳み込み・実数／複素数FFT、Karatsuba・FWHT・共通bitwise、BのRelaxed・多次元・分割積・Middle・Chirp-Z。 |
| 3.17 多項式・形式的冪級数 | `fps/`, `docs/fps/`, `visualizations/fps/` | 基本演算・微積分、inv/log/exp/pow/sqrt、除算、多点評価・補間、Taylor Shift、Bostan–Mori。Bの合成・Sparse/Online・GCD/Mod Pow・再利用積木・重根部分分数。 |
| 3.18 線形漸化式・母関数 | `linear_recurrence/`, `docs/linear_recurrence/`, `visualizations/linear_recurrence/` | 行列累乗・Kitamasa・Berlekamp–Massey・共通のBostan–Mori、P-recursive prefix・EGF・有理式・係数抽出・重み付き等比和。 |
| 3.19 集合関数・Bitmask変換 | `set_function/`, `docs/set_function/`, `visualizations/set_function/` | Submask／Supermask列挙、和・汎用SOS、部分集合／上位集合ゼータ・メビウス、bitwise・Ranked Zeta・Subset Convolution、SPS inv/log/exp・合成・重み付き分割。 |
| 3.20 線形代数 | `linear_algebra/`, `docs/linear_algebra/`, `visualizations/linear_algebra/` | Matrix・行列積／累乗、Gauss–Jordan・XOR基底、逆行列・Bitset GF(2)・空間の交差・特性多項式、CSR・Black-Box・Wiedemann・決定的最小多項式・Woodbury。 |
| 3.21 文字列アルゴリズム | `string_algorithm/`, `docs/string_algorithm/`, `visualizations/string_algorithm/`・`b_21_25/` | Z・KMP・Hash・Trie・SA/LCP・回文・SAM・Lyndon・LCS、Runs・静的Suffix Tree・wildcard・動的Hash・最小回転・複数比較・回文分割。 |
| 3.22 計算幾何 | `computational_geometry/`, `docs/computational_geometry/`, `visualizations/computational_geometry/`・`b_21_25/` | 整数／浮動を分離。基本幾何・凸包・包含・切断・最近／最遠対、最小包含円・円交点・凸層・Minkowski・共有MST・Pick・矩形内半平面交差。 |
| 3.23 組合せ最適化・マトロイド | `combinatorial_optimization/`, `docs/combinatorial_optimization/`, `visualizations/combinatorial_optimization/`・`b_21_25/` | Hungarian・Assignment・Blossom・最小費用流、非重み付きMatroid Intersection・Graphic/Partition・重み付き独立集合・線形Parity。 |
| 3.24 ゲーム理論 | `game_theory/`, `docs/game_theory/`, `visualizations/game_theory/`・`b_21_25/` | Nim・Misère・mex／DAG Grundy・直和、nim積・周期証明、Nimber体演算・閉路ゲームwin/lose/draw・有限Partizan。 |
| 3.25 探索・ヒューリスティック | `heuristic_search/`, `docs/heuristic_search/`, `visualizations/heuristic_search/`・`b_21_25/` | 乱数・Timer・SA・Beam・局所／Tabu・Zobrist・差分更新、Multi-Start・温度較正／制御・paired-seed tuning・Bandit・重み付き近傍管理。 |

3.7〜3.11のB追加分の図解は、各カテゴリ索引から共通の `visualizations/b_7_11/` へ辿れます。

## 3.21〜3.25 B項目の整備（2026-10-18）

[28項目の公開API・適用範囲・検証](docs/b_21_25/README.md) / [一次資料](docs/b_21_25/references.md)。全28掲載箇所を公開実装・個別説明・使用例・図解付きで整備。これでB全168掲載箇所が完了。3.16〜3.20の変更とともに9067220へコミット済みです。

- 3.21：2順序のLyndon根によるRuns、SA/LCPの静的Suffix Tree、両側1byte wildcardの正確な整数畳み込み、ImplicitTreap共有の動的二重Hash、線形最小回転、厳密な複数文字列比較、Eertree series linksの最小回文分割と復元。
- 3.22：固定seedの最小包含円・円交点、境界共線点込みの凸層、凸包Minkowski、共有キャリパーの最遠対、3.7共通Manhattan MST、Pick、境界矩形を明示するO(H²)半平面交差。有理数/128bitとlong doubleを分離。
- 3.23：独立性オラクルのBFS Matroid Intersection、Graphic/Partition、1マトロイドの重み付き貪欲と一般グラフ64頂点の厳密MWIS、線形Matroid Parityの小規模厳密DFSと検証付きMonte Carlo代数版。代数版の最大性に確率的失敗があり、実行可能性は検査。Cの重み付き交差は保持。
- 3.24：uint64のNimber体演算、閉路を許す通常ゲームのwin/lose/drawと手・終局距離、有限short Partizanの和・負号・差・部分順序。Partizanは構文の共有とゲーム値の等価を区別し、生成数とbirthday予算あり。
- 3.25：独立seedのMulti-Start、悪化幅からの平均採用率較正と観測温度制御、同じseed列でのgrid tuning、UCB1/epsilon-greedy、segment treeの重み付き近傍選択・統計・managed SA。スコア改善を保証しない。

検証入口：`python3 verify/run_b_21_25.py` / `--sanitize` / `--links-only` / `make verify-b-21-25`。28公開入口の単独/重複include、全カテゴリ混在の複数翻訳単位、28使用例、4独立C++比較、4ジャッジ形式×24入力、C++と28図解の参照値。図解：`node verify/visualizations/b_21_25_model_test.js` / `b_21_25_browser_test.mjs --browser /path/to/chromium`。

S/CとPython版の計画は保持。オンライン提出・AC、探索のスコア改善は未確認。B追加分は9067220へコミット済みです。

今回の検証結果：通常・ASan/UBSan/LeakSanitizerのdebug/release、28使用例、28件のC++・図解参照値、4ジャッジ形式×24入力が成功。最後に補強した巨大なUCB探索係数・極端な重みの抽選・overflow更新時の状態維持も、通常とサニタイザの両構成で再確認しました。既存string_algorithm/computational_geometry/combinatorial_optimization/game_theory/heuristic_search/final_a/advanced/graph_basic/graph_combinatorics/b_7_11/ordered_structure/b_16_20の12系統の通常回帰と、`make verify-visualizations`の全モデル回帰が成功。追加図解は2,478入力・20,239状態・21拒否条件、28モードの全フレームのSVG描画範囲・ブラウザ操作・390px表示が成功。デスクトップ4種とモバイルの画像を確認し、関連ローカル文書リンク1,413件に欠落なし。既知のsocket/ptrace制限に従い、ChromiumとLeakSanitizerは承認レビュー付きで同じローカル検証を制限外から実行しました。

## 3.16〜3.20 B項目の整備（2026-10-18）

[全31項目のAPI・条件・比較検証](docs/b_16_20/README.md) / [一次資料](docs/b_16_20/references.md)。対象全31掲載箇所を公開入口・個別説明・使用例・図解付きで整備。既存のGCD/LCM、Min-Plus、BMと第N項、Matrix Tree、Subset Convolutionは共有します。

- 3.16：両列のRelaxed、任意軸の通常tensor積、NTT上限を超えるブロック積、全長積から切り出すMiddle Product、三角数を使うChirp-Z。
- 3.17：baby-step/giant-stepの有限多項式合成、Newton合成逆、Sparse inv/log/exp/pow、Online inv/exp、monic Euclid GCD、剰余二分累乗、再利用SubproductTree、重根を含む部分分数。既存polynomial_interpolationは公開積木を使うよう統合。
- 3.18：共有BMとKitamasa、既知P-recursiveの逐次prefix、階乗スケールEGF、積・有理式の係数抽出、約分した有理式の和・積、多項式重み付き等比和。
- 3.19：高位変数分割のSPS inv/log/exp、有限多項式のSPS合成、重み付き集合分割と個数別DP、ranked-zeta積のint素数mod入口。SPS exp/logは集合分割の整数多項式として小標数でも検証。
- 3.20：重複座標を保持するCSR、callbackのKrylov・冪作用・候補解、固定seedのWiedemann、決定的な行列最小多項式（最悪O(n⁴)）、共有Matrix Tree、Woodbury逆更新と行列式比。射影多項式を行列全体の最小多項式と扱わず、返す候補解は残差を検証。nulloptは非可解性の証明に使わない。

検証入口：`python3 verify/run_b_16_20.py` / `--sanitize` / `--links-only` / `make verify-b-16-20`。31入口の単独・重複include（debug/release）、全カテゴリ混在の複数翻訳単位、31使用例とusage、3本の独立C++比較、4ジャッジ形式×24入力。図解は `visualizations/b_16_20/`、`node verify/visualizations/b_16_20_model_test.js` / `b_16_20_browser_test.mjs --browser /path/to/chromium`。

最適化の範囲と計算量は各説明を参照。高速合成・高速中間積・Holonomic推定・Frobenius標準形などのCは保持し、Python版も未着手。オンライン提出・ACは未確認。B追加分は9067220へコミット済みです。

今回の検証結果：通常・ASan/UBSan/LeakSanitizerのdebug/release、31使用例、31件のC++・図解参照値、4ジャッジ形式×24入力が成功。追加の4096項Online FPS、256項合成逆、128次巡回作用とuint64最大冪、16次の決定的最小多項式、P-recursiveの非定数係数とEGF容量境界も確認しました。既存convolution/FPS/linear_recurrence/set_function/linear_algebra/transform_a/number_theory_convolution/b_12_15/advanced/graph_combinatoricsの10系統の通常回帰が成功。`make verify-visualizations`による全図解モデルの回帰も成功。図解モデルは2,394入力・16,582状態・22拒否条件、31モードの全フレームのSVG描画範囲・ブラウザ操作と390px表示が成功。デスクトップ・モバイル画像を確認し、関連ローカル文書リンク1,390件に欠落なし。Chromiumのsocket制限・LeakSanitizerのptrace制限を避けるため、同じローカル検証を承認レビュー付きで制限外から実行しました。

## 3.12〜3.15 B項目の整備（2026-10-18）

[対応API・制約・検証](docs/b_12_15/README.md) / [一次資料](docs/b_12_15/references.md)。対象全30掲載箇所を整備。既存Min-Plusを拡張し、29公開ヘッダと30個別説明・30使用例・30図解モードを追加。4カテゴリのall.hpp・索引・Makefile・実装計画を更新しました。整数根・商列挙・素数カウントは共通実体を使います。

- 3.12: 凹×凹の端点最小と任意×凹の部分包絡、noshi91 CC0の行・列縮約LARSCHとO(N) Monge最短路、SMAWK各段O(KN)の辺数制約とパス復元、CDQと畳み込みDP、signed128の分数上界・非再帰の分枝限定0-1、64bit列のM4RM Boolean行列積。Monge条件は利用者の前提。Four RussiansのLCS版ではありません。
- 3.13: 非互いに素と最小指数のBSGS、素数検査付きTonelli–Shanks、共有uint64根と閉区間商列挙、飽和判定付き有限modテトレーション、合同式の全解を等差列で返すAPI、uint64 n/kのLucas、小さい素数法のO(p)表、共有Lucy DP。BSGSとLucyの確保予算を明記。
- 3.14: 商列挙による約数個数和、Du Jiaoのφ総和/Mertensとmemo共有、商状態のLucy個数、prime_power callbackからの線形乗法的prefix、μで数えるO(sqrt N) Square-Free。大きな総和はsigned128。一般化Min_25・任意関数の巨大N総和はCに残す。
- 3.15: Eulerian、B1=-1/2のBernoulli、Narayana、既存FPS冪によるLagrange–Bürmann係数、Stirling/二項の正逆変換、上昇/下降階乗基底、既存補間とProductTreeによる標本点シフト、一般法の二項prefix。二乗表の計算量・unit条件・FPS上限を個別に明記。

検証入口: `python3 verify/run_b_12_15.py` / `--sanitize` / `--links-only` / `make verify-b-12-15`。30入口の単独/重複include（debug/release）、全カテゴリ混在の複数翻訳単位、30使用例とusage一致、3本の独立C++比較、4ジャッジ形式×24入力。小さい法の全軌道、全剰余根、全合同式、全品物subset、全順列・Dyck・集合分割、直接べき和・形式的固定点・Horner、Python任意精度の大きな指数も確認。オンライン提出・ACは未確認。

図解: `visualizations/b_12_15/`。`node verify/visualizations/b_12_15_model_test.js` / `node verify/visualizations/b_12_15_browser_test.mjs --browser /path/to/chromium`。小入力化や全走査への置換は同READMEに明記。Python版全体の作成計画は保持し、今回もC++版の作業です。

今回の検証結果: 通常・ASan/UBSan/LeakSanitizerのdebug/release、30使用例、4ジャッジ形式×24入力、既存dp_optimization/number_theory_basic/number_theory_convolution/combinatorics/FPS/math_a/advancedの7系統の回帰が成功しました。図解モデルは3,300入力・30,101状態・15拒否条件、ブラウザは全30モードと390px表示が成功。デスクトップとモバイルの画像を確認し、関連ローカル文書リンク1,187件に欠落なし。既知のsocket/ptrace制限に従い、ブラウザとサニタイザは承認レビュー付きで制限外から同じローカル検証を実行しました。今回の変更は未コミットです。

## 3.7〜3.11 B項目の整備（2026-10-18）

[対応API・制約・検証](docs/b_7_11/README.md) / [一次資料](docs/b_7_11/references.md)。対象全36掲載箇所を整備。35公開ヘッダ（共有用forward入口を含む）と既存Link-Cut Tree、36個別説明・36実行例・36図解モードを用意し、5カテゴリのall.hpp・索引・Makefile・予定リストを更新しました。Link-Cut Treeの元の説明も保持しています。

- 3.7: O(VE)のChu–Liu/Edmondsと親辺復元、128bit座標変換のManhattan sweep、非負の最短閉路2種、HLDの最大2異重み・パスchminでMST感度と別の木／strictな第二最小木。
- 3.8: 非再帰Lengauer–Tarjan、Unlicenseのpath absorptionによる三重辺連結、ループ／平行辺も区別するCactus、MCSのChordal、次数2消去とfillの幅2分解、縮約DAGの増分SCC、全対bitset／64問い合わせバッチのDAG到達。
- 3.9: 負下限・負費用・有限負費用閉路を扱うb-flowと循環流、signed128費用と全成分の双対potential、Stoer–Wagner、FIFO Push–Relabel、DMと使用可能辺、Δ色の交互路辺彩色、既存Assignmentを使う最大重み二部マッチング。一般グラフの重み付きBlossomはCのまま。
- 3.10: 独立集合分割の彩色多項式、次数順2辺walkのC4、共有Bron–Kerbosch、Dreyfus–WagnerとSteiner辺復元、元辺だけのDAG cover、単位容量の辺素／内部頂点素パス、未確定を返す1-WL。
- 3.11: 高さO(log²V)のrake/compress式、Context付き動的rerootと距離和Spec、implicit treapのETT、共有LCT、辞書完全一致の木同型、重心頻度畳み込みの距離分布、Contourの点加算/帯和と帯加算/点取得、固定森の辺重み更新直径、共有MergeTree。動的reroot/直径はtopology固定。link/cut込みの直径・部分木集約・Fully Dynamic Tree DPはC側。

検証入口: `python3 verify/run_b_7_11.py` / `--sanitize` / `make verify-b-7-11`。36入口の単独/重複include、全カテゴリ混在の複数翻訳単位、36使用例と文書usage、4本の定義比較テスト、3ジャッジ形式×24入力をdebug/releaseで実行。復元木・パス・流量・双対証明、非可換LCT、4万頂点chain/starのtop tree、10万頂点Dominatorも確認。オンライン提出・ACは未確認です。O(VE)の有向MSTと容量依存SSPはジャッジ最大制約の時間保証をしません。

図解: `visualizations/b_7_11/`。`node verify/visualizations/b_7_11_model_test.js` / `node verify/visualizations/b_7_11_browser_test.mjs --browser /path/to/chromium`。教材の全列挙・直接走査への置換は同READMEへ一覧化しています。

今回の検証結果: 通常・ASan/UBSan/LeakSanitizerのdebug/release、36使用例、3ジャッジ形式×24入力、既存graph_basic/graph_connectivity/graph_flow/graph_combinatorics/tree/graph_a/advanced/combinatorial_optimizationの回帰が成功しました。図解モデルは1,851入力・11,702状態・10拒否条件、ブラウザは全36モードと390px表示が成功。デスクトップとモバイルの画像も確認し、関連ローカル文書リンク1,346件に欠落はありません。ブラウザのsocket制限とLeakSanitizerのptrace制限が再現したため、同じローカル検証を承認レビュー付きで制限外から実行しました。既存の未コミット変更を保持し、commit/pushは行っていません。

## 3.1〜3.6 B項目の整備（2026-10-17）

[対応API・制約・検証](docs/b_1_6/README.md) / [一次資料](docs/b_1_6/references.md)。対象全43掲載箇所を整備しました。43公開入口・43個別説明・43使用例と、42モードの図解（Range Treeは3.3／3.6共通）を追加。実体は既存カテゴリごとに配置し、6カテゴリのall.hpp・README・共通索引・Makefile・予定リストを更新しています。

- 3.1: 連鎖catalogのFractional Cascading、uint64全域の整数k乗根、Stern–Brocotのrun経路、分子/分母上限付き分数二分探索、モノイド平方分割、正しい区間下界を必要とする非単調分枝限定、既存RangeSet。
- 3.2: Parallelは区間同士のoffset併合（スレッド並列ではない）。動的DSUは未知ラベルの追加（オンライン辺削除ではない）。成分加算/点加算はtag・offset・点固有値を分離。Incremental Bridgeは辺追加と橋数/二重辺連結性。Offline Dynamic Connectivityは既存実装を共有。
- 3.3: Sparse Dynamicは既存実体。Rollback、版の任意分岐と確保しない検索のPersistent Lazy、runのsplit/meldと外側集約によるSortable、一意keyと点set、Range Tree、Segtree FC、可換作用の密な2D Dual。
- 3.4: key同値なら元の添字順にk個を選ぶ重み和Wavelet Matrix、別実装のWavelet Tree、uint64永続Binary Trie、上位/下位k和、Dynamic Bitset、word+prefixのrank/select（selectはO(log N)）、O(N)構築O(1)のLinear RMQ、静的Sqrt Tree。
- 3.5: Splay多重集合、左傾Red-Black、3/4節点weightで部分再構築するBST、shared_ptrの不変Treap、list所有とspliceによるSkew/Leftist、削除予約を個数辞書で検査するPriority Queue。Lazy Treapとiwiwi Radixは共有。heapはpop済み節点をclearまで保持。
- 3.6: 四隅の係数を座圧BITで保持する矩形加算/和、inclusive prefixを追加した矩形加算/点取得、静的Kd木の報告/最近点、共通Range Tree、Hilbert順Mo、最終出現BITの種類数、lazy Kd木のオンライン点追加/点代入/矩形affine・和。新点は過去の作用を受けず、Kdの矩形処理は最悪O(N)。

検証入口: `python3 verify/run_b_1_6.py` / `--sanitize` / `make verify-b-1-6`。43ヘッダの単独/重複include（debug/release）、全カテゴリ混在の複数翻訳単位、43使用例と説明usageの一致、4本の独立比較検証、4ジャッジ形式×24バッチ。整数根はPython任意精度、矩形加算は直接交差面積、Sortableはmod998244353の直接関数適用、種類数はsetと比較します。Linear RMQ/Sqrt Treeは20万/13万要素と構築・query演算回数も検査。オンライン提出・ACは未確認です。

図解: `visualizations/b_1_6/`。`node verify/visualizations/b_1_6_model_test.js` / `node verify/visualizations/b_1_6_browser_test.mjs --browser /path/to/chromium`。入力・前後・再生・エラー回復に対応。木を階層配置し、赤黒の色を表示します。小入力の簡略化は同READMEへ明記し、モデルとC++検証は別に実施します。

検証結果: 通常・ASan/UBSan/LeakSanitizerのdebug/release、4ジャッジ形式×24バッチ、ローカルリンク1,600件、図解モデル4,112入力・131,777状態・8拒否条件、ブラウザ42モードと390px、既存union_find/range_query/static_query/ordered_structure/offline_query/a_priority/advancedの回帰が成功しました。ブラウザのsocket制限とLeakSanitizerのptrace制限が再現したため、同じ検証を承認レビュー付きで制限外から実行しています。最新の結果はこの作業の検証出力と上記入口を参照してください。Python版は実装計画への追記のみで、今回のC++成果には含みません。

## 3.21〜3.25 A項目の補完（2026-10-17）

[追加APIと検証範囲](docs/final_a/README.md) / [一次資料](docs/final_a/references.md)。不足10項目を補い、対象のA全25掲載箇所が実装済みです。10公開ヘッダ・10個別説明・10使用例を追加。3.22・3.23、Aho–Corasick/Eertree、ゲームの直和、Hill Climbingは既存実装を共有します。

- 3.21: 任意byteのSuffixAutomaton、DuvalのLyndon分解、共通SAMのLongest Common Substring、二つの素数法のRollingHash2D。SAMはappend後に出現数cacheを無効化、count_occurrencesは重複を含み空patternはn+1。LCSは同長ならsecond_left→first_leftの順。2D getは(top,bottom,left,right)、baseと形の比較を含み、衝突あり。
- 3.24: uint64全域のnim_product（加算はXOR）、観測列のdetect_grundy_periodとsubtraction_grundy。観測した反復は未来の証明として扱わず、有限の正の引き算手では最大手M個の同一窓を証明条件にする。未検出の未観測値は拒否。Nim小表はmex、二次拡大で64bitへ拡張し、128bitやSIMD不要。
- 3.25: 固定seedのZobristHash、訪問keyと期限・aspirationのTabu、同じ深さのkeyごとに最良を残すbeam_search_unique、try_scored_moveとsimulated_annealing_delta。旧beam_searchは重複保持を維持。差分はsigned Scoreと同じ型、applyは強い例外保証、undoは例外を投げず正確に復元。棄却・accept例外でも状態とscoreを戻す。

検証入口: `python3 verify/run_final_a.py` / `--sanitize` / `make verify-final-a`。10公開入口の単独/重複include（debug/release）、全カテゴリ混在の複数翻訳単位、10使用例と説明中usage、5ジャッジ形式各24入力、C++の独立比較とPython 9,505ケースをdebug/releaseで検証します。全Lyndon分割・全byte部分文字列・二変数多項式の直接和・64bitの4,096単項基底nim積・全引き算手集合・状態窓辞書・Tabu履歴・差分全再計算を使用。差分SAは同じseedの既存SAと採否・最良/現在解・次の乱数まで比較します。オンラインAC・スコア改善は未確認。

図解は `visualizations/final_a/` の10モード。`node verify/visualizations/final_a_model_test.js` / `node verify/visualizations/final_a_browser_test.mjs --browser /path/to/chromium`。3,078入力・32,128状態・14拒否条件を独立解と比較します。SAMのendpos、各prefixの状態長、直接矩形hash、nim単項基底、周期証明、期限とapply/undoを確認。小入力の教材モデルで、64bitはBigInt、Grundyの候補と証明を分離します。

今回の検証結果：通常・ASan/UBSan/LeakSanitizer（debug/release）、既存文字列/ゲーム/探索/幾何/組合せ最適化/advancedの回帰、全30本の図解モデルが成功。ブラウザ全10モード・390px表示が成功し、デスクトップとモバイル画像を確認。関連ローカルリンクに欠落なし。既知のptrace/socket制限に従い、サニタイザとブラウザは承認レビュー付きで制限外から実行。3.21〜3.25は65cccb0へ保存済み。その後の追加依頼に従い、3.7〜3.20の残りの変更も追加コミットへ含め、全A補完をGitに揃える。

## 3.16〜3.20 A項目の補完（2026-10-17）

[追加APIと検証範囲](docs/transform_a/README.md) / [一次資料](docs/transform_a/references.md)。不足9掲載箇所（8系統）を補い、対象のA全29掲載箇所が実装済みです。10公開ヘッダ・10個別説明・10使用例を追加。3.17・3.18と既存の行列式・階数は共有します。

- 3.16: 環上のKaratsuba、自然順FWHT、AND/OR/XOR畳み込み。FWHTとbitwiseは3.19の実体への共通入口。Karatsubaは短い側32以下を直接積、整数の途中超過を検査。FWHTは逆の各段で2を割り、整数の割り切れを検査。集合の長さは同じ正の2冪で、自動paddingなし。
- 3.19: 公開ranked_zeta_transform / ranked_mobius_transform。[rank][mask]で返し、一般のrank表の逆も可。既存Subset Convolutionの内部変換を公開APIへ置き換えて共有。AND/ORは合成数法の環でも可、XORのmod型は2の逆元が必要。
- 3.20: matrix_inverse / _mod、bitsetのgf2_gauss_jordan / gf2_rank、体型/素数modのlinear_space_intersectionとXorBasisのxor_space_intersection、characteristic_polynomial / _mod。逆行列は特異ならnullopt。GF(2)はbit容量と実際の変数数を分離し、RHSを係数側pivotにしない。交差は独立基底と係数の零空間、XOR版はU側成分の追跡。特性多項式はHessenberg相似変形と主小行列の漸化式、Mod<=nも可。実数版の条件・精度限界は個別説明を参照。

検証入口: `python3 verify/run_transform_a.py` / `--sanitize` / `make verify-transform-a`。10入口の単独/重複include（debug/release）、全カテゴリ混在の複数翻訳単位、10使用例、8ジャッジ形式各24ケース、C++の定義比較・再帰乗算回数・mod12環・129bit GF(2)・65,536項XOR単位元、Python任意精度7,336ケースをdebug/releaseで検証します。小体の全生成空間・全二値解・置換和の行列式と特性多項式・逆の左右の積を独立オラクルとします。オンラインAC未確認。

図解は `visualizations/transform_a/` の8モード。`node verify/visualizations/transform_a_model_test.js` / `node verify/visualizations/transform_a_browser_test.mjs --browser /path/to/chromium`。モデルは1,924入力・31,226状態・15拒否条件で、rank各段・行同値・相似変形・主小行列多項式・全空間を比較。教材のKaratsuba閾値は2、行列は小さい素数法、bitpackingは0/1表で表現します。

今回の検証結果：追加分の通常・ASan/UBSan/LeakSanitizer（debug/release）、既存convolution/FPS/線形漸化式/集合関数/線形代数/advancedの回帰、全29本の図解モデルが成功。ブラウザは全8モード・390px表示が成功し、デスクトップとモバイル画像を確認。関連ローカルリンク1,001件に欠落なし。既知のptrace/socket制限によりサニタイザとブラウザは承認レビュー付きで制限外から実行。既存の未コミット変更は保持しました。

## 3.12〜3.15 A項目の補完（2026-10-17）

[追加APIと検証範囲](docs/math_a/README.md) / [一次資料](docs/math_a/references.md)。不足19掲載箇所を補い、対象のA全32箇所が実装済みです。16の新しい公開ヘッダと18の個別説明・使用例。CHT/Li Chaoは既存実装をDPで使用し比較検証、CRTと一般化CRTは一つのAPIを共有。3.14の既存5項目を保持しました。

- 3.12: D&Cの段更新・到達不能optional・分割復元、Knuthの区間DP・merge_order、単位hingeのSlope Trick、離散凸・整数罰則のWQS。D&Cは既存Monotone Minima、KnuthはIntervalDPResultを共有。最左argminの単調性は呼出側の前提。WQSは同値なら個数が多いoracle、端点がKを挟まなければnullopt。CHT/Li Chaoは同じ二乗距離DPの例と素朴解を揃えました。
- 3.13: Euler φ/μの線形SPF篩と64bit単一値、一般化CRT、任意長Garner、負係数floor_sum、素数法の最小原始根。単一値/原始根は既存Miller–Rabin/Pollard–Rhoを共有。CRTのprefix lcm超過は例外。Garnerの出力法は入力法と共通因子可。floor_sumはn,m<2^32、a,bはsigned64、正規化中の超過も例外（相殺される最終値だけの保証ではない）。
- 3.15: Stirling第1種の符号あり/なし・第2種、Bell、整数分割、Montmort、Burnsideの固定点平均、全置換群のCycleIndex/Pólya。mod平均は群の位数が可逆であること。Burnsideのexact版はuint64の固定点総和を整数で割る。CycleIndexは置換を検査するが群の完全列挙/群作用は呼出側の責任。

旧CHTの公開名を保持し、標準名を明示修飾、ヘッダ内の特殊化をinlineに変更。グローバルusing namespace stdと複数翻訳単位の重複定義を除去しました。旧CHTの演算は型内に収まる前提で、数値範囲の保証を拡張していません。

検証入口: `python3 verify/run_math_a.py` / `--sanitize` / `make verify-math-a`。18入口の単独/重複include（debug/release）、全カテゴリ混在の複数翻訳単位、18使用例、8ジャッジ形式各24ケース、3本のC++差分検証＋Python任意精度5,000ケース。オンラインAC未確認。図解は `visualizations/math_a/` の18モード。`node verify/visualizations/math_a_model_test.js` / `node verify/visualizations/math_a_browser_test.mjs --browser /path/to/chromium`。

今回の検証結果：通常・ASan/UBSan/LeakSanitizer、既存DP/基本数論/数論畳み込み/組合せ論/advancedの全回帰が成功。図解は2,337入力（うち64bit文字列3境界例）と23,940状態、ブラウザ全18モードと390px表示が成功。デスクトップ・モバイル画像を確認。関連ローカルリンク1,435件に欠落なし。既知のptrace/socket制限によりサニタイザとブラウザは承認レビュー付きで制限外から実行。

## 3.7〜3.11 A項目の補完（2026-10-17）

[追加APIと検証範囲](docs/graph_a/README.md) / [一次資料・問題例](docs/graph_a/references.md)。不足15掲載箇所を補い、対象のA全36箇所が実装済みです。13の新アルゴリズム、Block-CutとFunctional Graphの共有入口、経路の共通結果型を16公開ヘッダと個別説明・使用例に整備。3.10の既存6項目を保持します。

- 3.7: Radix Heap Dijkstra、Dial、DAG最短路、Johnson、Yenの単純K-shortest、EppsteinのK-shortest walk、補グラフBFS、Functional Graph。通常の最短路は既存結果型。補グラフは元の辺IDがないため頂点列だけ復元。Walkはcostsとsidetrack履歴を保持しpath(i)で遅延復元。無向自己ループは一つの論理辺として数える。
- 3.8: Block-Cutは既存block_cut_forestを共有（元の全頂点＋ブロックの森）。2-SATは既存SCCを共有。Eulerは有向/無向、自己ループ/多重辺、始点指定を扱う。Functional Graphは3.7と共用。
- 3.9: 下限制約付き循環/実行可能s-t流/最大流。s-tのvalueは負も可。Project Selectionは単項費用・罰金・必須含意/強制・submodular二変数費用。両方ともDinicを共有し再solve可。容量/需要/費用の中間値は型内、違反overflow_error。必須条件の容量はsoft総和+1。
- 3.11: 二重modの根付き部分木ハッシュと重心からの非根付き木ハッシュ。固定seed、頂点番号・辺順・重みを無視。ハッシュ一致は同型の厳密証明ではない。

既存Radix Heapはtupleのincludeを補い、グローバルなusing namespace stdを名前空間内の型usingに変更。公開クラス名/操作は保持し、混在時にstd::rankが他カテゴリのrankフィールドと衝突する問題を防ぎました。graph/all.hppも基本グラフの新APIを含みます。旧Functional Graphの雛形は保持。

検証: `python3 verify/run_graph_a.py` / `--sanitize` / `make verify-graph-a`。16ヘッダ単独/重複include（debug/release）、逆順・複数翻訳単位、16使用例、6ジャッジ形式各24ケース、5本の差分検証。オンラインAC未確認。図解は `visualizations/graph_a/` の14モード。`node verify/visualizations/graph_a_model_test.js`、`node verify/visualizations/graph_a_browser_test.mjs --browser /path/to/chromium` で検証。モデルとC++の差は同READMEに記載。

今回の検証結果：通常・ASan/UBSan/LeakSanitizerの全検証（debug/release）、全26カテゴリ混在の複数翻訳単位、既存5カテゴリ＋graph/treeの回帰検証が成功。図解は2,319入力・20,682状態、ブラウザ全14モード・390px表示が成功し、デスクトップ/モバイル画像も確認。関連ローカルリンク1,093件を確認しました。既知のsocket/ptrace制限のためブラウザとサニタイザは承認レビュー付きで制限の外から実行。

## 3.1〜3.6 A項目の補完（2026-10-17）

[追加API・検証範囲](docs/a_priority/README.md) / [一次資料と問題例](docs/a_priority/references.md)。未実装/一部対応32掲載箇所を補い、対象6カテゴリのAは47箇所すべて実装済みです。33公開ヘッダ（3.4のOrderedSet forward includeを含む）と個別説明・使用例を追加。既存APIと旧2D SegtreeのWIPを保持し、6カテゴリall.hpp・索引・予定リストを更新。

- 3.1: basic_utilityを新設。商列挙のみ閉区間[l,r]、他の区間は半開。順列は0..n-1、n<=20。倍数は非負uint64範囲。全(S,T⊆S)は既存submaskを共有。
- 3.2: 部分永続/完全永続DSU、可換成分集約、奇閉路状態付きDSU、区間内全頂点の併合、MergeTree。部分永続は失敗mergeも時刻が進む。完全永続は既存永続セグ木を配列として共有。
- 3.3: long long長のDynamic/Dynamic Lazy、密な2D BIT/Segtree、Add/Assign/Affine和モノイド。Dynamic Lazyは未作成区間のdefault_prodが必要。2D Segtreeは可換性が必要。作用のcompositionは新しいfが後。
- 3.4/3.5: DST、XOR付きBinaryTrie、階層FastSet、Treap/RBST/OrderedSet、汎用作用付きLazy Reversible Treap、永続Stack/Queue。Queueは分岐時も上界が保てる祖先ダブリング版（push/front O(log M)、pop/back O(1)）。
- 3.6: Mo on Tree、Rollback Mo、座圧2D BIT、RangeSet、Offline RMQ。木上Moは頂点集合の追加/削除、経路順の集約ではない。Rollback Moは利用者のsnapshot/rollbackを使い終了時reset。座圧BITは更新点の事前登録だけ必要で操作順はオンライン。

検証入口: `python3 verify/run_a_priority.py` / `--sanitize` / `make verify-a-priority`。33ヘッダの単独/重複include（debug/release）、逆順・6カテゴリ混在・複数翻訳単位、33使用例とusage一致、6ジャッジ形式×24バッチ×300操作、5本の差分検証（debug/release）。オンラインAC未確認。

図解は `visualizations/a_priority/` の32モード。モデル検証 `node verify/visualizations/a_priority_model_test.js`、ブラウザ検証 `node verify/visualizations/a_priority_browser_test.mjs --browser /path/to/chromium`。入力・前後・再生に対応。部分/完全永続DSUは版ごとの論理配列、Lazy Treapは論理列を描き、C++の内部配置との差はREADMEに明記。

追加時の検証結果：通常・ASan/UBSan/LeakSanitizerの全検証（debug/release）、既存5カテゴリの回帰検証、全26カテゴリall.hppの混在コンパイル、関連ローカルリンク1,162件が成功。図解モデルは2,586入力・87,734状態、ブラウザ全32モード・390px表示が成功し、木の階層図とモバイル画像を確認しました。socket/ptrace制限のためブラウザとサニタイザは承認レビュー付きで制限の外から実行。

この環境ではnode/chromiumはPATH外ですが、`/tmp/cp-tree-tools/node-v22.14.0-linux-x64/bin/node` と `/tmp/cp-tree-tools/chrome-linux64/chrome` を利用できます。

## 高度なアルゴリズム20項目の追加（2026-10-17）

[全項目の対応表・API・検証](docs/advanced/README.md)。既存のHLD、FFT/NTT、FPS一式、Bostan–Mori、Berlekamp–Massey、一般Blossom、旧CHTを保持し、不足分を11カテゴリ・21ヘッダへ追加しました。

- `range_query/segment_tree_beats.hpp`、`static_query/wavelet_matrix.hpp`。
- `union_find/rollback_disjoint_set_union.hpp`、`offline_query/offline_dynamic_connectivity.hpp`・`mo_with_updates.hpp`。
- `ordered_structure/persistent_segment_tree.hpp`・`persistent_order_statistics.hpp`・`implicit_treap.hpp`。
- `tree/virtual_tree.hpp`・`dsu_on_tree.hpp`・`link_cut_tree.hpp`。
- `dp_optimization/li_chao_tree.hpp`・`monotone_minima.hpp`・`smawk.hpp`・`min_plus_convolution.hpp`。
- `number_theory_convolution/miller_rabin.hpp`・`pollard_rho.hpp`（3.14の高度な整数問題に分類）。
- `set_function/subset_convolution.hpp`、`convolution/convolution_mod.hpp`。
- `string_algorithm/aho_corasick.hpp`・`eertree.hpp`。

各カテゴリall.hppとREADMEへ登録。各ヘッダに個別説明・動く使用例、共通の一次資料一覧を用意しました。空だった旧Beats・DynamicConectivity・AuxiliraryTreeは新ヘッダへのinclude入口です。既存の非空APIは変更していません。

新しい検証入口は `python3 verify/run_advanced.py` / `--sanitize` / `make verify-advanced`。21ヘッダの単独・重複include、11カテゴリallの混在・逆順・複数翻訳単位、21使用例、13ジャッジ形式サンプル、4本の独立差分検証（debug/release）。マニフェストは `verify/advanced/manifest.json`。サンプルだけでなく各構造を素朴配列・BFS・集合・全submask・直接照合と比較します。オンラインACは未確認。

`visualizations/advanced/` は21モードの共通教材。入力変更・前後・再生に対応。モデル検証は `node verify/visualizations/advanced_model_test.js`、ブラウザ検証は `node verify/visualizations/advanced_browser_test.mjs --browser /path/to/chromium`。TreapとLCTは論理列／森の図、RhoはFloydによる1回の分割、CRTの積は直接計算など、C++との差は同ディレクトリREADMEに明記。既存カテゴリの図解ページは保持。

注意：Beatsは値を厳密に±2^60内とし全中間値がlong long内。任意modは法1..2e9・結果長2^24以下。64bit数論は標準C++17の加算倍増剰余乗算で、__int128版より遅いがuint64全域で正確。SMAWKは全単調、Monotone Minimaは最左argmin単調が必要。ACは疎なmap辺で、failure探索をメモ化しないbuildの保守的上界はO(VL log σ)。永続木のrootハンドルは同じオブジェクトの返したrootに限る。

追加時の検証：通常・ASan/UBSan/LeakSanitizer（debug/release）、既存14カテゴリのunit回帰、215ローカルリンク、図解モデル2,472入力・43,096状態、ブラウザ全21モード・390px表示が成功。Beatsのcap/floor未伝播の子に大きな加算を送る境界と、mod型Subset Convolutionも追加検証。サンドボックス内では既知のptrace／socket制限が再現したため、同じサニタイザ／ブラウザ検証を承認レビュー付きで外側実行。追加実装と実装予定リストは、ユーザーの「一旦コミット」の依頼でまとめて保存しています。

## 既存カテゴリの補足

3.3 の補足:

- Fenwick `lower_bound(w)` は最初に累積和が w 以上になる**要素添字**、該当なしは n、w<=0 は 0。全要素非負が探索条件。
- Lazy の長さ付き S では、`LazySegtree(n)` の初期値は e()。ゼロ配列が必要なら `vector<S>(n,{0,1})` 等を渡す。
- Dual は値 S と更新 F を分け、一点 set/get と区間 apply を提供。非可換な更新に対応。
- Segtree/Lazy は ACL の CC0 実装を基に依存を除去・調整。出典とライセンスは `docs/range_query/references.md` に記載。
- 旧 `segtree` は新実装への別名。旧 `BIT` は公開 `n/bit/val` の互換性のため保持。

3.4 の補足:

- Sparse Table は冪等なモノイド専用。空区間は e()、和や一般の関数合成には使わない。build で全体を再構築する。
- MonotonicStack.push は追加前の最近傍の適格な添字を返す。strict=true は < / >、false は ≤ / ≥。見つからない添字は -1。size は候補数。
- 一括の `previous_better_indices` / `next_better_indices` を用意。less/greater で小さい／大きい側を選ぶ。
- MonotonicQueue.pop は論理先頭を削除する。size は論理長、best_index は clear 以降の通算添字。同値は最も左の候補を返す。
- Sliding Window Minimum は値と argmin を提供。k>0 が前提、k>n は空、添字は元配列の位置。
- SWAG は `S,op,e`、prod は左から右の順序。空の積は e()、pop/front はならし O(1)・最悪 O(n)。
- 図解では SWAG の移し替え途中の状態も表示する。その途中では back の残り→front の逆順が元の列になる。新操作は安定した開始／完了位置から行う。

3.5 の補足:

- 標準コンテナを覆う独自クラスは作らず、前後検索・erase_one・pop_first/pop_last の補助関数を提供。検索は該当なしで end() を返す。
- predecessor/successor は比較器順の前／後。既定は厳密、inclusive=true で同値を含む。const・透明な比較器の異種キー検索に対応。
- CoordinateCompression は元の値を保持し、同値を比較器で判定。index(x) は optional、lower/upper_bound は未登録の閾値にも位置を返す。
- CompressedOrderStatistics は事前登録された候補に対する多重集合。insert/erase は bool、erase_all は削除個数、kth は 0 始まりの optional<T>。
- 未登録の更新・負の個数・個数不足・総数の LLONG_MAX 超過は論理状態を変えず拒否。c=0 は登録済みなら成功。問い合わせ閾値は登録不要。
- 3.3 の FenwickTree を再利用。clear は個数だけを消して登録候補を保持する。set の問題では count==0 のときだけ insert する。
- 図解は set/multiset/map/圧縮/順位管理の 5 モード。標準コンテナの内部木は描かず、論理的な並びを示す。

3.6 の補足:

- Mo は `Mo(n)` に `[l,r)` を登録して ID を受け取り、run の answer(id) で元の順へ保存。左右 4 コールバック／共通 2 コールバック、空区間、処理順取得・幅指定に対応。run 前に利用者が空の集計へ初期化する。終了時の状態を空へ戻さない。
- parallel_binary_search は `M,Q,reset,apply,check` を受け取る汎用関数。時刻 t は先頭 t 回の更新適用後。答え 0=初期成立、M+1=該当なし。判定は false→true に単調。毎ラウンド reset、apply(i) は 0 始まり順。Q=0 ならコールバックなし。スレッドは使わない。
- Sweep Line は `static_rectangle_sum(points,rectangles)`。点は `{x,y,weight}`、長方形は `{left,down,right,up}`。x の二境界と圧縮 y の BIT で静的な半開長方形和を解く。
- CDQRectangleSum は add_point/add_query で時刻順に登録し、solve() const で ID 順の答え。過去の点だけを参照し、繰り返し solve・追記・clear に対応。展開イベントを時間で二分し、左右を x 順にマージする。今回入れた BIT 更新だけを巻き戻す。
- 空・逆向きの長方形は 0、同一座標・負の重み可。座標は < による順序のみ、境界 −1 不要。重みは逆元を含む全中間値が収まる加法可換群。
- 座標圧縮（3.5）と Fenwick（3.3）を再利用。`misc/mo_algorithm.hpp` は空の既存ファイルなのでそのまま保持し、新 API は offline_query/mo.hpp に置いた。
- 図解は 4 モード。バッチ入力を変更して履歴を再生成する方式。CDQ の時刻列は 4 prefix に展開した後のイベント列で、途中の答えは部分寄与。入力変更を反映すると常に最初から計算する。

3.7 の補足:

- 基本アルゴリズム本体を graph_basic へ移動、旧 graph/*.hpp は互換入口。共有 Graph / 入力 / 距離の算術補助は graph/ に保持。tree や LowLink の Graph と辺 ID を共用する。
- MST は kruskal.hpp / prim.hpp / spanning_forest.hpp に分離。minimum_spanning_forest.hpp は共通入口。非連結も解き、cost / 採用 Edge / component_count を返す。空の connected() は true。
- 既存 depth_first_search は単一始点の ShortestPathResult<int> を維持。追加 dfs_forest は全成分の parent / parent_edge / depth / component / roots / preorder / postorder / tin / tout を返す。first_root=-1 なら頂点順、それ以外はその根から優先。部分木は preorder[tin[v]..tout[v])。有向 DFS の component は SCC ではない。
- Bellman–Ford を辺列の直接走査へ変更して O(V+VE)。無向辺は両向きを緩和。孤立点が多く負閉路がある場合の全頂点走査を避ける。同距離で選ぶ経路の順は保証しない。
- 距離の結果型は共通。reachable は負閉路の影響下でも true、has_finite_distance は影響下で false。vertex_path / edge_path は復元不可で空、始点自身の頂点列は {s}（有限時のみ）。FW は始点別の結果列。
- 距離最大値は INF 専用。負閉路検出中の値を含め全加算が型に収まることが条件で、範囲外は overflow_error。負閉路影響フラグと数値 dist を区別する。
- 図解は 11 モード。全成分 DFS、キュー/deque/優先候補、FW の全点対行列、負閉路、始点限定の閉路、多重辺、MST、親の復元を表示。入力全体から履歴を再生成する。

3.8 の補足:

- SCC は反復 Kosaraju 法、scc / strongly_connected_components は同じ結果型を返す。component / groups / size / same。成分 ID はトポロジカル順、異なる成分への辺は from<to。各 groups の頂点は昇順、無関係な成分同士の ID 順は保証しない。
- condensation_graph(g,deduplicate=true) は scc / dag / original_edge を返す。内部辺を除外、既定は平行な成分間の辺をまとめ、false なら全て残す。縮約辺の重みは 1。代表元辺 ID は最小 ID 保証なし（成分→頂点順→隣接順で最初の辺）。線形時間の行先マークを使う。
- SCC／縮約は有向用。それ以外は無向用。全て重みを無視し、空・非連結・自己ループ・多重辺に対応。
- Lowlink / bipartite / connected_components 本体を graph_connectivity に移動し、旧 graph パスは互換入口。graph/all.hpp に新カテゴリ全体を含めた。既存の二重辺／頂点連結成分は graph/ に残し、新 Lowlink を経由する。
- bridges / articulation_points は ID／頂点の昇順一覧の補助関数で、その都度 low_link を呼ぶ。両方必要なら一度の low_link 結果を使う。
- 橋は ord[p]<low[v]、根以外の関節点は ord[p]≤low[v]、DFS の根は子が 2 個以上。親頂点でなく親辺 ID の逆向きだけ除外する。孤立点の削除で成分数が減っても関節点ではない。
- bipartite_coloring の成功は optional の 0/1 色列、失敗 nullopt、空グラフは成功した空列。奇閉路の復元 API は含まない。無向 connected_components にも size/same を追加。
- 図解は 7 モード。仮削除では元の ord/low を保持し、色分け・一覧は削除後の連結成分へ切替。C++ の Lowlink を再計算する操作ではない。SCC の逆パスと DAG 重複切替、二部グラフの衝突辺も表示する。

3.9 の補足:

- `Dinic<Cap>` は Graph と別の残余ネットワーク。Cap は bool 以外の整数型、非負容量。元辺ごとに正逆の残余辺を対で持ち、自己ループ・平行辺・逆向き元辺に対応。add_edge は元辺 ID、get_edge / edges は from/to/capacity/flow の値。
- max_flow は今回の追加量を返し、limit の省略値は型の最大値。全体の最大流が型に収まることが条件。最初の呼出し（limit=0 を含む）から reset_flow まで s,t を固定。辺追記は可、容量変更は未提供。reset_flow は容量・ID を保持し、流量と s,t の固定をリセット。
- Dinic は BFS + current arc + 明示的な経路／最小容量スタック。再帰なし。一般容量 O(V²E)、E=0 は O(V)。逆辺を通ると元辺の flow が減る。
- min_cut(s) は現在の残余到達性のみ。`minimum_cut(network,s,t)` は残りも流してから capacity/source_side/edge_ids を返す。network は更新される。容量 0 を含む元辺 S→T のみを ID 昇順で復元。型超過で overflow_error の場合も、それまでに流した分は戻さない。
- `BipartiteMatching(L,R)` は左右別の 0 始まりで辺を登録。maximum_matching() const は毎回空から計算し、cardinality/left_match/right_match/pairs を値で返す。未マッチ -1、pairs は左昇順、相手の辞書順最小は保証しない。辺重複・片側 0・追記可。
- Hopcroft–Karp も反復実装。BFS は全自由左頂点が根、自由右頂点までの最短長を明示し、同じ長さの交互路だけを使う。O((V+E)√(V+1))。一般グラフのマッチングや二部判定は含めない。
- 図解は 3 モード。入力変更から履歴を再生成、逆辺表示・次の増加路／交互路・上限停止と未確定カットを表示。経路の青破線は、フローでは残余逆辺、マッチングでは反転前に外す採用辺。level/distance の -1 は未発見またはその段階で使い切った頂点。

3.10 の補足:

- 共有 Graph を使う無向アルゴリズム。三角形・クリーク・独立集合・彩色は単純グラフが条件、重みを無視。行列木定理だけは多重辺・自己ループを許容（ループは除外）。単純性・向きは assert。
- enumerate_triangles は次数・頂点番号の順に辺を向け、O(N+M√M) 時間、コールバックへ昇順の三頂点を返す。全体の列挙順は未保証。
- enumerate_cliques は番号の昇順で候補を交差させる全列挙で、64頂点制限なし。enumerate_maximal_cliques は64ビットの pivot 付き Bron–Kerbosch。両方 bool(const vector<int>&) のコールバック、空集合は既定で除外、false で打切り。参照を保持せず保存時はコピー。
- 極大は追加できない集合、最大は頂点数が最大の集合。空集合が極大になるのは空グラフのみ。列挙関数は全探索完了で true、コールバック停止で false。
- maximum_clique は貪欲彩色の色数を上界に分枝限定。maximum_independent_set は自己ループを除いた補グラフで同じ探索。64頂点まで、結果は昇順の頂点列。同点解の辞書順保証なし。再帰深さ64以下、最悪指数時間。
- minimum_coloring は厳密 O(3^N) DP、22頂点まで。最小頂点を含む独立集合を一色として除く。非独立集合で2色が見つかったら打切り可。chromatic_number/color/groups を返し、色は0始まり。chromatic_number(g) は数だけの入口。空なら0色。
- 64／22頂点の制限超過は release でも length_error。上限は表現・メモリの制限であり、全ての入力の実行時間保証ではない。指数時間・出力数の制約を説明に明記。
- count_spanning_trees<Mod> は辺 cost を無視して数える。count_weighted_spanning_trees<Mod> は整数 cost の積の和。Mod は int の素数、コンパイル時判定。負・符号なし64ビット重みも正規化。乗算は long long に収まる。removed_vertex=-1 は末尾の行・列。
- 空グラフの全域木数は0、一頂点は1。多重辺は別々、ループは無視。答え0 mod pは非連結を意味しない。determinant_mod<Mod> は long long 正方行列を値で受け、空行列式1、行交換と非零 pivot を使う。
- 図解は6モード。列挙上限・極大のみ・空集合・補グラフ表示、DPと彩色復元、重み・素数mod・除く行列の頂点を編集。行列の乗算は BigInt。消去後は余因子内の行番号を表示し、元ラプラシアンと分ける。

3.11 の補足:

- 既存 tree/ を3.11の独立カテゴリとして整備。RootedTree に終了順の postorder を追加し、既存の配列・API、LCA・直径・Euler Tour・HLD・重心・旧互換入口を保持。tree/all.hpp は新しい3ヘッダも含む。
- BinaryLifting(next,max_steps=UINT64_MAX) は Graph を使わない一意な遷移。-1終端・自己ループ・閉路可。up[b][v] は2^b回先。jump(v,k) はuint64_t、k=0は自分、途中で尽きれば-1。k>max_stepsはreleaseでもout_of_range。既定64段、上限0でも1段保持。
- tree_dp / rerooting は g,identity,merge,add_vertex,lift,root=0 を受ける関数。merge(S,S)->S、add_vertex(S,v)->R、lift(R,Edge<T>)->S。R は戻り値から推論、S と異なる型も可。S はコピー／代入可能、R はさらにデフォルト構築可能。戻り値は頂点ID順のvector<R>。
- lift の辺は必ず情報の送信元→受信先。子→親ではfrom=子,to=親。cost/idxは元の無向辺のもの。from/to/idxに応じた非対称な変換も可。
- tree_dp はpostorderで子の寄与をg[v]の順に併合。rerooting は下向きDPと親側の寄与を元の隣接位置で集約し、prefix[i]とsuffix[i+1]で子iを除く。結合則・左右の単位元が必要、可換性・逆元は不要。非可換の文字列DPと全根の素朴再計算で確認。
- 全方位DPの答えは各頂点を根としたその成分の状態。最初のrootに依存しない。コールバックは渡された引数のみから結果を作り、入力／状態を変更しない。空では空列、コールバックなし。状態操作O(1)ならO(N)、重い状態ではその費用を掛ける。
- 新DPもRootedTreeを使うため、重みを無視するDPでも根距離の算術条件（全中間値が型に収まり、最大値未満）を引き継ぐ。DPコールバック内の加算／積の範囲は利用者が管理する。
- 図解は7モード。根変更・森・空・負辺／0重み・LCAの経路とk番目、往復列、2回の直径走査、全方位のincoming/prefix/suffix、独立集合の白黒状態、任意の遷移を表示。LiftingのkはBigIntで入力、C++と同じuint64範囲。図解は最大12頂点、表はkに必要な段数のみ。

3.12 の補足:

- 新しい独立カテゴリはdp_optimization/。全体の入口はall.hpp。旧DP/LIS.hpp・DP/subset_dp.hpp・DP/TSP.hpp・misc/Largest_Rectangle.hppは保持。旧スニペットに依存せず自己完結した新ヘッダを使う。
- lis(a,strict=true,compare=less<T>)はlength/indices/length_endingを返し入力を変えない。greaterで減少、falseで同値可。LCSはlcsで対応添字、lcs_lengthでO(M)空間。編集距離はedit_distanceでO(M)空間、edit_alignmentでKeep/Insert/Erase/Replaceの操作列。位置は元の両列を読むカーソル。
- ナップサックはzero_one_knapsack / unbounded_knapsack / bounded_knapsack(capacity,items)。品物は{weight,value}、個数制限は{weight,value,count}。結果のdp[w]は重さちょうどのoptional<long long>、未到達nullopt。best_value()は容量以下の最大値をO(W)で求める。空集合を許容。負価値可、重さ0は0-1と個数制限だけ可。完全版は重さ正。
- 個数制限は重さの剰余類ごとの単調キューでO(NW)。得点はold[r+jw]+(t-j)*value、有効範囲t-count<=j<=t。countはlong longでLLONG_MAXも可。重さ0は正価値なら上限まで、0以下なら0個。全加算・乗算の中間値がlong longに収まる必要があり、超過でoverflow_error。最大値も有限値として使える。
- BitsetKnapsack(max_sum)は0-1部分和の到達可能性。add/contains/reachable_sums/words/reset。0は常に到達、範囲外containsはfalse。語の大きい側からshift-ORし、64ビットシフトを避け、余白ビットを消す。値の最適化・解の復元・個数は持たない。
- interval_dp(n,cost)は半開の二分割、長さ0/1は0。cost(l,k,r)はlong long。dp/split/value()/merge_order()を返す。O(N³)、復元は非再帰で子→親の結合順。一般の多状態区間DP全てを扱うテンプレートではない。
- digit_dp<Count>(upper,S,initial,transition,accept)は0〜upperの十進整数。transition(state,digit,started,next_started)はstateか-1。先頭0も渡すので状態の扱いは利用者が定義。0は未開始のまま。Countは0/1構築・==・+=。標準整数は超過検出、mod型は独自に定義。先頭0可、空・非数字はinvalid_argument。O(10DS)、O(S)空間。
- subset_dp.hppはfor_each_submask(uint64_t,...,include_empty=true)、in-placeのsubset_zeta_transform / subset_mobius_transformとminimum_hamiltonian_cycle。集合変換は正の2冪長。3.19の共通実装へ移動し、標準整数の超過を検査、独自型の算術は利用者が管理。TSPはoptional<long long>正方行列（nullopt=辺なし）。exists/cost/vertices。空・1頂点は移動なしで費用0、2頂点以上は末尾に始点を重ねる。20頂点まで（超過length_error）、O(N²2^N)時間・O(N2^N)空間、20頂点のDP配列は通常約200MiB。
- largest_rectangle_histogramはarea/left/right/height、largest_rectangle_gridはarea/top/left/bottom/right。グリッドはtrue=障害物・false=使用可。全て半開座標。空・0列・全障害物は面積0・全座標0。既存3.4の左右の厳密に小さい境界を再利用し、ヒストグラムO(N)、グリッドO(HW)・O(W)空間。
- 図解は12モード。部分集合ではTSP／部分集合列挙／ゼータ／メビウスの4処理。DP途中の表、0-1と完全の走査方向、個数制限の候補キュー、64ビット語、桁のtight/started、復元経路を表示。区間と桁は隣接結合／桁和の具体例で、C++の任意のコールバックを実行する画面ではない。

3.13 の補足:

- 独立カテゴリはnumber_theory_basic/、入口all.hpp。旧number_theory/の関数・ファイルを保持し、新ヘッダは標準C++17のみ。
- gcd(a,b)はunsigned long long、lcm(a,b)はoptional<unsigned long long>。入力はlong long全域、LLONG_MINも可。gcd(0,0)=0、LCMは0を含むと0、ULLONG_MAX超過でnullopt。一般の符号なし入力全域を受け取るAPIではない。
- extended_gcdは{gcd,x,y}、a*x+b*y=gcd。gcdのみ符号なし、係数はlong long。余り0の係数更新を省き、最小整数の絶対値・除算を符号なしで扱う。利用側の恒等式の中間積はlong longに入るとは限らない。解の最小化は保証せず、AOJ用ラッパで補う。
- mod_pow(base,exponent,modulus)、mod_multiply(a,b,modulus)はlong long。法>=1、指数>=0、違反invalid_argument。負数の底・乗数可。法1は0、0^0=1 mod m。積がunsigned long longに入れば直接、他は加算・倍増。一般の法で乗算O(log m)・累乗O(log e log m)、m<=2^32なら乗算O(1)。__int128不要。
- mod_inverse(a,m)はoptional<long long>。m>=1、GCD!=1はnullopt。合成数法可、法1は代表元0。
- PrimeSieve(N)は[0,N]のvector<bool>と素数列、SmallestPrimeFactor(N)は線形篩のvector<int>と素数列。N>=0。is_prime/table/primes/limit、SPFはoperator[]とfactorizeもある。表の範囲外はout_of_range。SPF[0]=0,SPF[1]=1、分解は1〜Nだけ。
- prime_factorization(n)は昇順の(素数,指数)、divisors(n)は正の約数を昇順で返す。n>=1、0以下はinvalid_argument。1の分解は空、約数は{1}。商による平方判定であふれを回避。最悪O(√n)なので大きい素数の64ビット分解用ではない。
- 図解は8モード、BigIntで64ビット境界・Euclidの係数・63ビット指数・逆元なし・LCM超過・篩の除去・SPFの生成と分解・約数の対を表示。篩は80、試し割りは10,000まで。図解のquery=0は分解なしの指定で、C++のfactorize(0)とは異なる。

3.14 の補足:

- 独立カテゴリはnumber_theory_convolution/、入口all.hpp。入力は算術関数の係数列で、乗法的関数である必要はない。3.13のPrimeSieveを明示的に再利用し、以前の実装を保持。
- divisor_zeta_transform / divisor_mobius_transform / multiple_zeta_transform / multiple_mobius_transformはvector<T>&のin-place。正の添字1〜Nだけを扱い、f[0]は保持。空・長さ1も可。約数zetaはk昇順、倍数zetaはk降順。逆変換は素数とkを両方逆順にして減算する。
- dirichlet_convolution(a,b)はij=n、gcd_convolutionはgcd(i,j)=n、lcm_convolutionはlcm(i,j)=nへa[i]*b[j]を集計する。出力長max(a.size(),b.size())、短い入力は0補完、入力0は無視・出力0はゼロ。両方空は空、一方空は他方の長さのゼロ列。入力は変更しない。
- DirichletとLCMはNを超える組を捨てる。大きい出力上限が必要なら先に入力を0拡張。GCDは大きい添字も小さい結果へ寄与するため、入力を先に切らない。
- zetaは+=、mobiusは-=、DirichletはT(0)/コピー/+=/*、GCD/LCMはさらに-=を要求。加算は可換、乗算は分配的。除算・逆元を使わないため合成数法も可。乗算順はa[i]*b[j]、非可換な行列でも検証。
- bool以外の標準整数は中間の加減乗算を検査し、超過overflow_error。符号なしも折り返さない。in-place変換の例外ではそれ以前の更新が残る。独自型の算術条件は利用者が管理。Nはint上限まで、超過length_error。
- 4変換とGCD/LCMはO(N log log N)、DirichletはO(N log N)。畳み込みの素数列は3変換で共有。ヘッダ10本（内部補助2本含む）、使用例7本。
- 図解は7モード。各素数/kの更新、読み取りと更新先、GCD/LCMのzeta→積→mobius、定義からの項と途中の行・履歴を表示。入力は正の添字12個まで、整数BigIntとmod12/998244353/1。C++整数の超過例外は再現しない。空欄は添字0だけの配列を表す。

3.15 の補足:

- 独立カテゴリはcombinatorics/、入口all.hpp。旧Combination/の2実装は公開フィールドとAPIを保持。標準C++17のみで、3.13の安全なmod乗算・逆元を明示的に再利用。
- factorials(max_n,modulus=998244353)は0!〜N!のvector<long long>。任意の正の法で可。FactorialTableは同じ階乗と逆階乗を前計算し、gcd(N!,m)!=1ならdomain_error。法1は全て0の規約。N>=0、m>=1、不成立はinvalid_argument。factorial/inverse_factorial/max_n/modulus/factorials/inverse_factorials、添字は上限を含み範囲外out_of_range。
- binomial(table,n,k)はFactorialTableまたはPascalBinomialを使う。PascalBinomialはO(N²)の任意正法の表、rows()はconst参照、operator()(n,k)も可。n<0/k<0/k>nは先に0、有効nが上限超過ならout_of_range。巨大nのLucas・素数冪/CRTによる高速版は含めない。
- permutations(table,n,k)はFactorialTableでnPk、combinationsはbinomialと同義で両表に対応。0個選ぶ場合も有効nは上限内を要求。
- catalan(table,n)はC(2n,n)-C(2n,n+1)。n<0は0、2nまで表が必要。n+1の逆元は不要で、Pascalなら合成数法でも可。2nの計算前に上限確認。
- multiset_combination(table,n,k)はH(n,k)。負の引数0、k=0は1 mod m（n=0含む、表の上限不要）、n=0/k>0は0。それ以外はn+k-1まで前計算が必要で、加算前に範囲確認。
- inclusion_exclusion_union<T>(sets,intersection)は非空maskの共通部分を交互和。complementは空mask=全体集合を含む。maskはuint64_tの昇順、setsは0〜63。表現上の上限であり全列挙は指数時間。Tは0構築/+=/-=、標準整数の途中の範囲超過はoverflow_error。途中で負になり得るため符号付き整数かmod型を使う。コールバック内の算術は利用者が管理。
- 階乗前計算は一般の大きい法でO(N log m+log m)、問い合わせO(log m)。m<=2^32ならO(N+log m)、問い合わせO(1)。Pascalは加算だけでO(N²)、問い合わせO(1)。包除はコールバック費用込みO(2^sets)。
- 図解は7モード。前計算上限24、n/kは12まで、包除は12要素・6集合。階乗・逆階乗の方向、Pascalの行、式の項、順序の区別、括弧列の反射、星と仕切り、包除の各項と要素係数を表示。階乗モードは逆元なしでも階乗だけの結果を表示し、FactorialTableの拒否条件を説明する。

3.16 の補足:

- 独立カテゴリconvolution/、入口all.hpp。3.14の添字が約数／倍数の畳み込みと区別する。標準C++17のみで外部依存・__int128なし。7ヘッダ（内部補助・all含む）、5使用例。
- ntt<Mod=998244353,Root=3>(vector<int>&,inverse=false)、ntt_max_size<Mod,Root>()はconstexpr。Modは2〜2×10^9の素数、Rootは乗法群の原始根でコンパイル時検査。自然順→自然順、順は正の指数、逆は負の指数と1/n。空はno-op、負の入力も正規化、非2冪invalid_argument、法の容量超過length_errorで変更前に拒否。既定の上限2^23。
- convolution<Mod=998244353,Root=3>(vector<T>,vector<U>)はvector<int>。bool以外の64ビット以下の標準整数型・異種型を正規化し、通常のc[k]=Σ_(i+j=k)a[i]b[j] mod Mod。空を含めば空、非空は|a|+|b|−1個、末尾0も保持。切り上げ後の2冪が容量内であることを要求し、超過length_error。任意法への高速CRT版は含まない。
- convolution_ll(vector<long long>,vector<long long>)は正確な64ビット整数積。非空の結果長2^24以下。min(sum|a|*max|b|,sum|b|*max|a|)<=2^63を要求し、超過overflow_error。打ち消しで最終値が収まる場合でも拒否する十分条件。0列は可。167772161/469762049/1224736769の3法を混合基数で復元し、P/2との桁比較で符号決定。uint64_tの規定の演算で大きさを得る。+2^63は例外、−2^63はLLONG_MIN。
- fft(vector<complex<Real>>&,inverse=false)、fft_real(vector<Real>)。Realはfloat/double/long double。空・1・2冪長、全n成分を返しパッキングなし。fft_realは順変換、逆は返り値へfft(...,true)。正の指数と1/nの規約はNTTと共通でFFTWと順の符号が逆。非2冪・非有限入力invalid_argument、非有限の途中値overflow_error。途中の例外では更新が残る場合がある。
- convolution_fft(vector<Real>,vector<Real>)とcomplex<Real>のオーバーロード。自動0埋め・元の結果長へ戻す通常の積。入力不変。空を含めばもう片方の値を検査せず空。実数版は実部を返す。FFTは近似値で整数への丸めの一般的保証なし。誤差は型・長さ・係数に依存する。
- 全変換・畳み込みはO(n log n)。NTTは共有単位根キャッシュO(log Mod)、初回O(log Mod)で構築。FFT in-placeは補助O(1)、返す版や畳み込みはO(n)。サイズ加算・切り上げの超過を検査する。
- 図解は6モード。変換16要素・畳み込み各8要素。ビット反転・各バタフライ・単位根・各点積・逆の正規化・0埋め・CRTの3法と符号、定義の積を表示。NTT/CRTはBigInt、FFTはNumber。C++の浮動型ごとの差は再現しない。

3.17 の補足:

- 独立カテゴリfps/、入口all.hpp。C++17・namespace cp・vector<int>の関数API。3.16のNTTを明示的に再利用し、modintや外部依存なし。14ヘッダ（内部2本・all含む）、11使用例。
- 係数は昇冪順、負数やINT_MIN/MAXを[0,Mod)へ正規化し、全APIで入力不変。Modは2〜2×10^9の素数、既定998244353。乗算系はRootが原始根（既定3）であることもコンパイル時検査。
- polynomial_add/subtractは長さmax、multiplyは空を含めば空・それ以外は和−1。derivativeは長さmax(0,n−1)、integralは定数0・長さn+1でn<Mod。fps_truncate(a,n)は正確にn項へ切り詰め／0補完。polynomial_trimは正規化後の末尾0を除去、polynomial_evaluateはHorner法。
- fps_max_size<Mod,Root>()はNTT容量C/2（C=1なら1）の共通精度上限Lで、既定2^22。通常の全長の積から切り詰める実装なので作業容量を明示的に確保する。乗算自体は全結果がC以下なら可。上限超過length_error。
- fps_inv/log/exp/pow/sqrt(a,n)は正確にn項、n≤L、入力のn次以上を無視。n=0は代数条件を検査せず空。invはa0≠0、logはa0=1、expはa0=0（空も可）、違えばdomain_error。定数項の逆元以外はNewton法・微積分。powの引数順は(a,uint64 k,n)、0^0=1、先頭次数v*kは除算で超過回避、logに掛けるk mod Modと先頭係数の整数乗を区別。
- fps_sqrtは奇素数専用でoptional<vector<int>>。先頭n項が零なら零列、非0先頭次数が奇数／係数が非平方剰余ならnullopt。Tonelli–Shanksで小さい先頭根を選びNewton法。先頭次数vの場合、結果の末尾v/2項は未確定なので0を選ぶ。精度0は成功した空列。
- polynomial_divmodは{商,剰余}を返し、末尾0を除去して零は空。除数0はdomain_error。正規化・trim後の両入力長≤L。小さい側は筆算、大きいものは反転とinvでO(M(N))。quotient/remainderの単独補助は毎回divmodする。
- multipoint_evaluation(a,xs)は点順、重複可、空の点は入力多項式を処理せず空。polynomial_interpolation(xs,ys)は同じ長さが必要、違えばinvalid_argument。点が法で重複すればdomain_error。同じ値でも拒否。結果長は点数、末尾0保持。点数<L、評価多項式の有効長≤L。積木と剰余木でO(M(n)+M(m+1)log(m+1))、補間はO(M(m+1)log(m+1))。木は内部型で再利用APIなし、バッチ逆元を使用。
- taylor_shift(a,long long c)は有限多項式f(x+c)、入力と同じ長さ・末尾0保持。n≤L、n≤Mod。階乗を用いた畳み込み、先に入力を切ると低次も変わる。
- bostan_mori(P,Q,uint64 k)は[x^k]P/Q、Q0≠0、trim後の両長≤L。分子次数が分母以上、定数分母も可。Q(-x)との積と偶奇抽出でO(M(N)log(k+1))。linear_recurrence_nth(initial,c,uint64 n)はa[t]=Σc[j]*a[t−1−j]。同じ非空長d、n<dは初期値を返す（容量制限なし）、それ以外はd+1≤L。
- 図解は12モード。BigInt、入力各16項・出力精度16（法17は8）、点数は法17で7以下。Newton精度倍増、積木・剰余・補間の重み、Taylor積、Bostan–Moriの偶奇と巨大項。図の乗算は定義式、除算は筆算で、C++のNTT／反転の高速化は再現しない。

3.18 の補足:

- 独立カテゴリlinear_recurrence/、入口all.hpp。C++17、namespace cp、vector<int>、入力不変。6ヘッダ（内部1本・all含む）、4使用例。既存matrixのグローバルAPIと誤綴りmatrirx_powを保持し、新APIをcpへ追加。
- 既知漸化式はa[t]=Σc[j]*a[t−1−j] (t≥d)、initialはa[0]から昇順、c[0]は直前の項。全手法で統一。initial/cは同じ非空長、違えばinvalid_argument。項番号は0始まりuint64全域、n<dなら初期項を直接正規化。末尾0も次数に含む。
- matrix_power_mod<Mod>(A,uint64 k)は正方行列の累乗、matrix_power_applyはA^k*initialの列ベクトル。指数0でも形を検査、0×0/空の状態は空、法1の単位行列は全0。linear_recurrence_matrixは状態[a[d−1],…,a[0]]、先頭行c・副対角1の行列をn−(d−1)回適用。時間O(d³log(n+1)+d²)、領域O(d²)。
- kitamasa<Mod>(initial,c,uint64 n)はO(d²log(n+1)+d)、領域O(d)。kitamasa_coefficients(c,n)はX^n mod (X^d−c[0]X^(d−1)−…−c[d−1])をd個の重みで返す。重みはa[0]からの順、n=0は単位、d=1ではX≡c[0]。除算／NTT不要。両手法はMod=1〜2×10^9の合成数法・法1に対応。
- berlekamp_massey<Mod>(samples)は素数法2〜2×10^9、非NTT素数も可。観測列s[t]=Σc[j]s[t−1−j] (L≤t<S)を満たす最短の長さLを返す。空／全0は空、末尾0は削除しない。{1,0,0,0}は{0}、{0,0,1}はL=3となる。複数解の辞書順最小は保証しない。接続多項式C[0]=1・保存B・不一致delta・距離shiftで更新し、右辺係数c=-C[1..L]を返す。時間O(S²+Slog Mod)、領域O(S)。
- linear_recurrence_from_prefix<Mod>(samples,uint64 n)は観測範囲ならその値、以降はBMが選んだ規則をKitamasaで延長。空／全0は零列を選ぶ。真の無限列が次数d以下の規則に従う前提なら先頭2d項で再現できる。有限観測だけで未知の将来を無条件に特定できるとは説明しない。
- linear_recurrence/bostan_mori.hppは3.17のfps/bostan_mori.hppをincludeする共通入口。同じcp::bostan_moriとlinear_recurrence_nthで再定義なし。既存API・容量・例外はそのまま。3.17の説明からも3.18へ相互リンク。P=(initial*Q) mod x^d、Q=1−Σc[j]x^(j+1)の母関数の対応を説明。
- 図解は4モード。漸化式6次・任意行列6×6・観測20項・直接P/Q各12項、BigInt。列ベクトルの各2冪、Kitamasaの高次還元、BMの不一致項と接続C/B/次数、母関数P/Qと偶奇。図のBostan積は定義式、C++ではNTT。法1／合成数／非NTT素数／巨大項／末尾0の違いを操作できる。

3.19 の補足:

- 独立カテゴリset_function/、入口all.hpp。C++17、namespace cp。9ヘッダ（内部1本・all含む）、7使用例。3.12のfor_each_submask・subset_zeta_transform・subset_mobius_transformを移動し、旧dp_optimization/subset_dp.hppのincludeと名前を保持。TSPは3.12に残る。
- for_each_submask(uint64 mask,callback,bool include_empty=true)は数値降順、mask自身と既定で0を含む。0を除けばmask=0で呼出しなし。for_each_supermask(mask,int bits,callback)は下位bitsビットの全体集合内を昇順、基準maskと全体集合を含む。bitsは0〜64、範囲外のmask／bitsはcallback前にinvalid_argument。0／UINT64_MAXの終端を確認してから次へ進み、折り返し・1<<64を避ける。戻り値による打切りなし。
- 4変換はvector<T>&をin-place更新。subsetはビット0→1、supersetは1→0に加減算。正の2冪長、空／非2冪は更新前にinvalid_argument、長さ1は不変。ゼータはビット昇順、メビウスは段階を逆順にし、交互和を復元。加法は可換・結合的、逆には加法群が必要。bool未対応、mod型は合成数法・法1も可、逆元不要。
- 標準整数の加減算は3.14のdetail/arithmetic.hppを再利用して超過をoverflow_errorにする。符号なしで負になる減算も拒否。途中例外は前の更新が残る。独自型とSOSのmerge内の算術は利用者が管理。旧DP入口にも新しい超過検査と逆変換の処理順が適用される。
- sos_dp(f)は部分集合ゼータの和。sos_dp(f,merge)は可換・結合的mergeによる全T⊆Sの集約。min/max/OR、単位元やT(0)を持たない型にも対応。mergeは引数を変更せず、Tへ代入可能な値を返す。各変換／SOSはO(n2^n)時間・O(1)追加空間、演算は正確にn2^n/2回。列挙は出力個数に比例、callback費用別。
- 図解は7モード。列挙64ビット・自由ビット8個まで、変換4ビット、BigIntでlong longの中間値を検査。次のmask、全ビット段階の表、読み取り元／更新先、注目する集合の途中の寄与と符号を表示。和・min/max/OR、mod12・法1、空集合除外・64ビットの境界を操作可能。

3.20 の補足:

- 独立カテゴリlinear_algebra/、入口all.hpp。C++17、namespace cp。8ヘッダ（内部2本・all含む）、5使用例。旧matrix/matrix_basic.hppのグローバル関数・誤綴りmatrirx_powと、3.18のmatrix_power_mod / matrix_power_applyは保持。3.18の説明から相互リンク。
- Matrix<T>は行優先vectorとrows/colsを保持し、0×mとn×0も区別。次元指定／initializer_list／vector<vector<T>>で構築、形違反invalid_argument、要素数の容量超過length_error。a(r,c)はassert、atはout_of_range、data()はconst。identityとtransposed、等値比較・コピー／move。move元は0×0、自分へのmoveも可。bool未対応。
- matrix_multiply / operator*はA(n,k)*B(k,m)、matrix_vector_multiplyは列ベクトル。T(0),*,+=が必要、積の可換性は不要。標準整数は各積と部分和を3.14の算術補助で検査、超過はoverflow_error。標準浮動小数点は非有限入力invalid_argument・非有限途中値overflow_error、有限の丸め・アンダーフローは検出しない。独自型は利用者管理。入力不変、次元違反invalid_argument。
- matrix_power(Matrix<T>,uint64 exponent)は正方行列、指数0でも形と有限性を検査。0乗はidentity、0×0も可。T(1)が必要。最後の不要な二乗を避け、LLONG_MAXの1乗も扱う。O(n³log(exponent+1)+n²)時間・O(n²)領域。mod型の積と累乗は合成数法・法1も可。
- gauss_jordan(A,b)は厳密な可換体向け。標準整数の除算はstatic_assertで拒否。浮動小数点はgauss_jordan(A,b,T epsilon=T(1e-10))、正の有限epsilonで絶対値≤epsilonを0と判定、列内最大絶対値pivot。小さい消去係数も省略しない。数値的な階数は入力スケール・丸め・悪条件に依存する。
- gauss_jordan_mod<Mod=998244353>(Matrix<int>,vector<int>)は素数法2〜2×10^9、3.18の算術補助で正規化し、pivotごとの逆元を1回計算。INT_MIN/MAXも可。合成数法はコンパイル時拒否。全版b.size()==A.rows()、入力不変。
- LinearSystemResultはstatus(no_solution/unique/multiple),rank,pivot_columns,reduced,particular,nullspace_basis,has_solution()。解があればx=particular+Σt[j]*basis[j]、自由列昇順。解なしは特解・基底を空にし、係数側のrank/pivot/簡約行列を返す。0×mは全変数が自由、n×0はbが全0なら空の一意解。基底を明示するためO(n(m+1)(r+1)+m(m−r))の体演算とO(n(m+1)+m(m−r)+m)領域。素数法の逆元費用はO(r log Mod)。
- XorBasis<UInt=uint64_t>はunsigned標準整数。insertは独立ならtrue、rank/empty/clear、contains、min_xor/max_xor(seed=0)、mergeで増えた次元、basisを提供。数値順は符号なし、空の選択で0を含む。最上位pivot降順、basisの値は挿入順依存。rank64でもシフト64を避ける。O(B)の挿入／問い合わせ、mergeはO(B²)、内部O(B)。
- 図解は5モード、各次元5・XOR入力12個まで。整数/modはBigInt、Gauss実数はNumber（表示7有効桁）。転置、内積、二分累乗のused+weight*remaining、行基本変形、特解・零空間、XORの最上位bitを操作する。

3.21 の補足:

- 独立カテゴリstring_algorithm/、入口all.hpp。C++17、namespace cp。10ヘッダ（内部1本・all含む）、8使用例。旧string/rolling_hash.hppのグローバルRollingHash<Str>とRunLengthEncoding.hppは変更せず保持。
- 文字列はstring_viewのバイト列。0始まり・半開区間[l,r)、辞書順／ハッシュ／Trieのbyteはunsigned char。UTF-8の見た目の文字単位へ変換しない。0を含む入力は明示した長さで渡す。入力長≤INT_MAX、超過length_error。Z/Prefix/KMP/SA/LCP/Manacherはvector<T>にも対応。等値には==、SAは<、組合せでは同値判定が一致する型が必要。
- z_algorithmは先頭と各suffixのLCP、z[0]=n、空は空。prefix_functionは各接頭辞の最長の真のborder、先頭0。kmp_search(text,pattern)は全出現位置を重複込みで昇順、空patternは0..n。区切りバイト不要。Z/PrefixはO(n)、KMPはO(n+m+k)時間、出力とprefixの領域。
- cp::RollingHashは2法1000000007/1000000009、Hash=array<uint32_t,2>。base既定911382323固定、257..1000000006のみ（不正invalid_argument）。byte+1を使い、先頭Hと冪だけを保持。get(l,r)、hash_string、concat(rawHash,rawHash,right_length)、equal(other,4境界)、lcp(other,4境界)／suffixの2始点。境界違反out_of_range、異なるbase比較は空でもinvalid_argument。concatのraw値にはbase/長さがないため呼出し側で揃え、right_length≤size()。非正規剰余は拒否。構築O(n)、区間/連結/等値O(1)、LCPはO(log(min長+1))。旧61ビット法とは型と値が異なる。
- 二重ハッシュでも衝突があり、equal/LCPは厳密ではない。固定baseで独立一様な衝突確率を保証しない。base257で0d 81 93 6e fc 1f 73 22と8個の00が同時衝突する例も検証。厳密照合にはKMP/SA/LCP等を使える。
- Trieは任意byteの疎な辺vectorとlower_bound、重複・空単語可。insert、erase1個(bool)、count、contains、count_prefix、size/empty、node_count、clear。root.passed=全体個数、terminal=その単語数。削除後の辺は保持し、clearで解放。move元は空rootに戻る。ノード数≤INT_MAX・個数≤SIZE_MAXを事前検査。クエリO(L log(σ+1))、挿入は新規辺の移動費用O(新規辺数*σ)、σ≤256。追加経路O(L)、再帰なし。
- suffix_arrayは空suffixを除く始点0..n−1。byte順／vectorは座標圧縮、倍増して順位の組を後半→前半の安定な計数ソート。O(n log(n+1))時間・O(n)領域。SA-ISではない。lcp_array(s,sa)は隣接suffixのLCP、長さmax(0,n−1)。saの長さ／順列を検査するが辞書順は前提。KasaiでO(n)、空も可。
- manacherはPalindromicRadii{odd,even}。odd中心iは[i−odd+1,i+odd)、even中心はiの直前で[i−even,i+even)。空は両方空、is_palindrome(l,r)はO(1)・空true・境界違反out_of_range。結果配列を変更せずに使う。奇数・偶数の両走査でO(n)、番兵バイト不要。
- 図解は8モード、S/T32バイト・pattern16バイト・Trie16操作60ノードまで。UTF-8/hexを切替え、00/FFも可。コピー・fallback・前計算と減算・各ノード個数・順位の組・LCP再利用・鏡像の半径を操作する。

3.22 の補足:

- 独立カテゴリcomputational_geometry/、入口all.hpp。17ヘッダ・12使用例。旧geometry/2D_geometry_template.hppは変更せず保持。cp::geometry::exactとfloatingでPoint/Vector・Line/Segmentを分離し、暗黙変換なし。
- exact::Pointはint64_t、WideはGCC/Clangの符号付き128ビット拡張。加減乗を組込み関数で検査し、座標差は広げてから計算。Pointの加減乗は結果座標もint64_t。全途中値が収まることが条件で、超過overflow_error。任意精度ではなく、int64_tの全座標組合せを保証しない。
- RationalはWideの分子・正の分母を約分して保持。積は交差約分、比較は連分数で不要な積を回避。最終値が収まっても途中超過は例外。0分母／0除算domain_error。非整数の交点・射影・切断はRationalPoint。再切断と面積も分数のまま。to_string(Wide)、str、approximate、conversion.hppのto_floatingで明示変換。
- floating::Pointは有限long double。非有限入力invalid_argument、非有限途中値overflow_error。Tolerance既定1e-10L・有限非負・絶対EPSで、尺度に依存し相対誤差保証はない。ソート／重複除去は数値の<と==、近似等値はalmost_equal。偏角はatan2の数値順。最近点対・直径の計量にはEPSを使わない。
- CCWは+1/−1/+2/−2/0の5分類。閉線分は端点・重なり・長さ0を扱う。直線は異なる二点が必要。line_intersectionはunique/parallel/coincidentとoptionalの点。squared_distanceはPoint/Line/Segmentの全組合せと対称順、exactではPoint同士Wide・他Rational、floatingはdistanceも提供。
- convex_hullは既定strict・CCW・最小(x,y)開始・閉点の再掲なし。keep_collinear=trueは辺上の異なる全点、全共線は辞書順。ConvexPolygonは点集合から凸包を構築し、containsはO(log h)。freeの凸包含と直径はstrict CCW凸包が前提で、毎回の検査を省く。
- Point in Polygonは単純多角形の周上順、CW/CCW・凹形可。Locationはoutside0/boundary1/inside2。面積はsigned_area2と非負polygon_area。凸切断は有向直線の左側の閉半平面。偏角は原点最初・+xから反時計回り、同方向は安定。
- 直径は凸包の添字を返すO(h)の回転キャリパー。空nullopt・一点{0,0,0}。最近点対は元入力の異なる添字をfirst<secondで返すO(n log n)、二点未満nullopt・重複は0。両方距離二乗で、同距離の辞書順最小は保証しない。
- 図解は12モード、20点・座標絶対値100万まで。BigInt有理数とNumberを切替。C++の128ビット上限やlong doubleの精度は再現しない。分数、境界、鎖のpop、面積の部分和、扇の二分探索、対蹠点、境界の帯を操作できる。

3.23 の補足:

- 独立カテゴリcombinatorial_optimization/、入口all.hpp。7ヘッダ（内部1本・共通入口含む）、5使用例・5ジャッジ形式。3.9の既存実装を変更せずBipartiteMatchingをforward includeで共用。新しい一般最大マッチングはGeneralMatching。
- hungarianはvector<vector<long long>>／vector<vector<optional<long long>>>を受け、全行へ異なる列を割り当てる最小費用。禁止セルはnullopt、有限INFなし。n>m・Hall条件不成立はnullopt、raggedはinvalid_argument。vectorの空は0×0。AssignmentProblem(0,m)なら0×mを保持。次元はINT_MAX未満。
- AssignmentResultはcost(long long),row_to_column,column_to_row(未使用−1),row_potential,column_potential(128ビット)。最小化はu+v≤cost・v≤0、最大化はu+v≥weight・v≥0、選択辺は等号、未使用列v=0、Σu+Σv=cost。コピーの独立した結果。O(n²m+n+m)時間・O(n+m)作業領域。
- AssignmentProblem(n,m)は全禁止で構築、set_cost/forbid/cost/rows/columns、solve(AssignmentObjective::minimize/maximize)。毎回再計算し全行を必ず埋める。最大化でも負重みだから未割当にはしない。内部Hungarianを共有し、最大化は128ビットへ広げてから符号反転する。
- GeneralMatchingは無向の重みなしBlossom最大組数。BFS・共通祖先・base縮約・元頂点のparentによる復元を反復実装。自己ループ無視、平行辺は解くときに線形除去。O(V³+E)時間・O(V+E)領域。GeneralMatchingResultはcardinality,mate(-1未対応),pairs(小さい端点先・昇順)。辺追記・再計算可。同数の辞書順最小は保証しない。
- MinCostFlow<Cap=long long,Cost=long long>はCap整数64ビット以下bool以外（unsigned可）、Cost符号付き整数64ビット以下。add_edge返値ID、flow/slopeは今回の追加流量・追加費用をResult{flow,cost}で返す。flowは折れ点を保存しない。get_edge/edgesで元辺{from,to,capacity,flow,cost}、reset_flowで辺とIDを保持して送流を0へ戻す。
- 最初のflow/slope（limit0含む）からresetまでs,tと辺集合を固定。同一s,tへの追加送流は可、変更／追記はlogic_error。limit0では探索・負閉路検査をしない。初回正送流は必要なら全成分へ仮想始点を置くBellman–Fordで負辺を処理し、正容量の負閉路は非連結成分もdomain_errorで拒否。循環流改善は含めない。
- 以後はpotentialと非負reduced costのDijkstra。初期化O(V+VE)、各呼出しA増加と到達不能確認でO((A+1)(V+E)log(V+E+1))、A≤F。slopeは(0,0)から同じ単位費用を統合。負費用なのでcostは減る場合がある。元辺逆向きの登録・自己ループ・平行辺・容量0も可。
- 費用のある新実装はGCC/Clangの検査付き__int128、公開名OptimizationWideとoptimization_to_string。全途中値が128ビット、返値費用はlong long/Costに収まる条件。フローは各増加後の今回費用もCostに収まること。超過overflow_error、現在増加は算術検査後に反映、同呼出しの前の増加は巻き戻さない。新APIの不正な次元・番号・容量・端点等は例外。二部版のみ既存assert条件。
- 図解5モード。BigInt費用、行列5×6・一般10頂点／二部各6頂点24辺・フロー8頂点16辺容量9上限20まで。縮約は元の位置を保持し同baseのリングで表示。双対とslack、交互路・逆辺、全成分の負閉路、追加費用と折れ点を追える。

3.24 の補足:

- 独立カテゴリgame_theory/、入口all.hpp。5ヘッダ・4使用例・4ジャッジ形式。標準C++17・namespace cp。Grundyは既存GraphのDirected版にも対応し、費用を無視。既存Graph／トポロジカルソートの実装は変更しない。
- nim_sum / nim_winning / nim_winning_moveはvector<Int>、bool以外の64ビット以下の標準整数型。負値invalid_argument、実数／boolコンパイル時拒否。全山を検査してuint64_tのXORを返し、総石数は加算しない。空・全0は通常Nimで負け。O(n)時間・O(1)領域。
- NimMoveはpile(size_t元添字),remaining,removed(いずれもuint64_t)。normalはtarget=pile XOR sumが元より小さい最初の山を選びXOR0へ。負けはnullopt、入力不変。
- misere_nim_winning / misere_nim_winning_moveは最後を取ると負け。2個以上の山がなければ1の山の偶奇（偶数勝ち）、一つなら1の山が偶数個のとき大山を1へ・奇数個なら0へ、それ以上は通常Nim。空・全0は手番側の勝ちという規約だが合法手がなくnullopt。通常Grundy XORの理論をmisere全般へ流用しない。
- grundy_mexは非負int列からmex、空0・重複可・INT_MAXの値も可。O(k)時間と領域、入力長≤INT_MAX、超過length_error。負値invalid_argument。
- calculate_grundy(moves)は隣接リストからGrundyResult{values,topological_order}を返す。全成分をsize_t入次数のKahn法で検査し逆順にmex。seenの印に頂点番号を使いO(V+E)時間・O(V)作業領域。通常ルールの公平な有限完全情報ゲーム、閉路・自己ループはinvalid_argument、範囲外はout_of_range、V≤INT_MAX。重複辺・孤立点・空可。Graph版は隣接コピーO(V+E)追加領域。
- DAGGameはmoves／Grundyを保持し、size/transitions/grundy/topological_orderをconst取得、grundy(v)/winning(v)、move_to_grundy(v,target)/winning_move(v)の任意目標／0への最初の移動先を返す。負target invalid_argument、頂点範囲外out_of_range。
- DAGGameのnim_sum(positions)/winning(positions)は独立なk成分のXOR。winning_moveはDAGGameMove{component,from,to}で、一成分を小さい目標Grundyへ移しXOR0にする。O(k+選択した頂点のoutdegree)。重複頂点でも独立な駒として数え、空の和は負け。相互の占有制限は扱わない。再帰なし。
- 図解4モード。山12個uint64 BigInt、DAG10局面45辺・独立な駒12個まで。16ビット表示区間切替、Misere終盤、Kahn/mex、独立和と勝ち手。合法手／勝ち手を適用して次手番の局面を再解析できる。閉路の残りは閉路そのものと限らず、そこから先の頂点も含むと明示する。

3.25 の補足:

- 独立カテゴリheuristic_search/、入口all.hpp。7ヘッダ・5使用例、標準C++17・namespace cp・外部依存なし。既存のstring/rolling_hash.hppとgeometry/2D_geometry_template.hpp内の乱数は変更しない。オンラインジャッジのPROBLEMは仮置きせずローカルquery_driverで検証。
- RandomはVignaのpublic domain SplitMix64をクラス化し出典を明記。seed省略0、全uint64 seed、URBG。rawとrealは1生成、realは上位53ビットの[0,1)。uniform(bound)は[0,bound)、threshold=2^64 mod bound未満を棄却。integer(lo,hi)は両端を含み、bool以外64ビット以下の整数型・符号付き負値／全域／uint64全域可。全域はraw、符号復元は範囲外signedキャストに依存しない。shuffleはFisher–Yates。乱数の大域状態／自動seedなし。
- BasicTimer<Clock>はis_steadyなClock、Timerはsteady_clock。単位は秒、構築時から実時間を測る。elapsed/limit/remaining/expired/progress、reset()/reset(seconds)。0秒はexpired=true・progress1、+∞はexpired=false・remaining∞・progress0。負・NaN・−∞拒否。模擬Clockで待機なしの境界検証。
- 共通SearchGoalはminimize/maximize、SearchOptionsはmax_iterations=1000とgoalとstd::function<bool()> stop。Stateはコピー／ムーブ可能でコピーの独立性が必要、デフォルト構築不要。evaluate(const State&)は純粋、bool以外の64ビット以下の算術型、浮動値は有限のみ。整数比較を保つ。初期評価1回は予算0／stopより先。
- SearchResultはbest/current_state、best/current_score、iterations/evaluations/accepted、SearchTermination。terminationはiteration_limit/depth_limit/local_optimum/exhausted/stopped。完了した反復／走査だけを数え、局所探索の改善なし完了走査も1回。同点最良は先を保持。評価回数超過overflow_error、不正値・設定invalid_argument。NDEBUGでも検査、コールバック等の例外は伝播、利用側副作用や乱数は巻き戻さない。
- simulated_annealingはpropose(const State&,Random&)が独立候補を値で返す。改善・同点必採用で採否乱数なし、悪化はexp(−loss/T)とreal1回。整数悪化幅は符号なし減算後long doubleへ変換し、UINT64_MAX付近の差1・符号両端も正しく扱う。現在／最良を別保持。AnnealingOptionsは正有限の開始／終了温度、linear/geometric、任意のprogress()∈[0,1]。既定はstep/(I−1)、I=1なら開始温度。Timer.progressとstopを両方設定すれば時間冷却。stopは反復先頭だけで、コールバック中は中断不可。
- local_searchはneighbors(const State&)が値の範囲（vector推奨）を返す。first_improvement／best_improvement、既定best。厳密改善だけ採用しbest=current。同点巡回しない。stopは走査先頭と各候補評価前、途中の最良改善走査は捨てて前の状態を返す。評価数には途中分も含む。I走査・B近傍でO(IB)評価、O(B)状態領域。
- beam_searchはexpand(const State&)が値の子範囲を返す。width≥1・max_depth、安定ソートで親順／子順の同点を維持、重複を統合しない。BeamSearchResultはfrontier(ScoredSearchState{state,score}),depth,evaluations,termination、best()は先頭const参照。異なる深さを比較せず最後の完成層を返す。全子空なら前層をexhaustedで返し、一部終端だけなら次層から消える。stopは親展開前と子評価前、部分層は捨て評価数は保持。通常O(Σ C log C)、状態領域O(W+C)で候補全体も保持。ソート途中には中断不可。
- 図解5モード。乱数BigInt・模擬時計、21位置の左右近傍、温度と採否、現在／最良、安定同点・重複保持と幅削減、局所最適。乱数16個、SA/局所100反復、Beam幅6深さ8、時計20更新まで。探索の品質／大域最適を保証しない。

## 検証方法

3.21〜3.25のB追加分：`python3 verify/run_b_21_25.py`（`--sanitize` / `--links-only`対応）/ `make verify-b-21-25`。図解は `node verify/visualizations/b_21_25_model_test.js` と `b_21_25_browser_test.mjs`。

3.16〜3.20のB追加分：`python3 verify/run_b_16_20.py`（`--sanitize` / `--links-only`対応）/ `make verify-b-16-20`。図解は `node verify/visualizations/b_16_20_model_test.js` と `b_16_20_browser_test.mjs`。

3.21〜3.25のA追加分: `python3 verify/run_final_a.py`（`--sanitize`対応）/ `make verify-final-a`。図解は `node verify/visualizations/final_a_model_test.js` と `final_a_browser_test.mjs`。

3.16〜3.20のA追加分: `python3 verify/run_transform_a.py`（`--sanitize`対応）/ `make verify-transform-a`。図解は `node verify/visualizations/transform_a_model_test.js` と `transform_a_browser_test.mjs`。

ライブラリのルートで実行します。Python 検証に追加 pip パッケージは不要です。

```sh
python3 verify/run_graph_a.py
python3 verify/run_graph_a.py --sanitize
python3 verify/run_a_priority.py
python3 verify/run_a_priority.py --sanitize
python3 verify/run_graph_tree.py
python3 verify/run_graph_tree.py --sanitize --stress
python3 verify/run_union_find.py
python3 verify/run_union_find.py --sanitize
python3 verify/run_range_query.py
python3 verify/run_range_query.py --sanitize
python3 verify/run_static_query.py
python3 verify/run_static_query.py --sanitize
python3 verify/run_ordered_structure.py
python3 verify/run_ordered_structure.py --sanitize
python3 verify/run_offline_query.py
python3 verify/run_offline_query.py --sanitize
python3 verify/run_graph_basic.py
python3 verify/run_graph_basic.py --sanitize
python3 verify/run_graph_connectivity.py
python3 verify/run_graph_connectivity.py --sanitize
python3 verify/run_graph_flow.py
python3 verify/run_graph_flow.py --sanitize
python3 verify/run_graph_combinatorics.py
python3 verify/run_graph_combinatorics.py --sanitize
python3 verify/run_tree.py
python3 verify/run_tree.py --sanitize
python3 verify/run_dp_optimization.py
python3 verify/run_dp_optimization.py --sanitize
python3 verify/run_number_theory_basic.py
python3 verify/run_number_theory_basic.py --sanitize
python3 verify/run_number_theory_convolution.py
python3 verify/run_number_theory_convolution.py --sanitize
python3 verify/run_combinatorics.py
python3 verify/run_combinatorics.py --sanitize
python3 verify/run_convolution.py
python3 verify/run_convolution.py --sanitize
python3 verify/run_fps.py
python3 verify/run_fps.py --sanitize
python3 verify/run_linear_recurrence.py
python3 verify/run_linear_recurrence.py --sanitize
python3 verify/run_set_function.py
python3 verify/run_set_function.py --sanitize
python3 verify/run_linear_algebra.py
python3 verify/run_linear_algebra.py --sanitize
python3 verify/run_string_algorithm.py
python3 verify/run_string_algorithm.py --sanitize
python3 verify/run_computational_geometry.py
python3 verify/run_computational_geometry.py --sanitize
python3 verify/run_combinatorial_optimization.py
python3 verify/run_combinatorial_optimization.py --sanitize
python3 verify/run_game_theory.py
python3 verify/run_game_theory.py --sanitize
python3 verify/run_heuristic_search.py
python3 verify/run_heuristic_search.py --sanitize
make verify-visualizations
```

`--compiler clang++` 等で切り替えられます。C++17 と警告エラーを使い、単独ヘッダ・複数翻訳単位・使用例・素朴解比較・ジャッジ入出力を確認します。ASan/UBSan も実行します。

3.3 追加時には通常検証・ASan/UBSan・両カテゴリの図解モデル検証とブラウザ検証が成功しました。3.3 の説明・図解・索引のローカルリンク 115 件も確認しました。

3.4 追加時にも通常検証・ASan/UBSan・全カテゴリの図解モデル・5 モードのブラウザ検証が成功しました。3.4 の説明・図解・索引のローカルリンク 132 件と、5 つの documentation_of の参照先、Markdown の usage 5 本のコンパイル・実行を確認しています。

3.5 追加時も通常検証・ASan/UBSan・全カテゴリの図解モデル・5 モードのブラウザ検証が成功しました。説明・図解・索引のローカルリンク 130 件、3 つの documentation_of の参照先、Markdown の usage 3 本のコンパイル・実行を確認しています。

3.6 追加時も通常検証・ASan/UBSan・全カテゴリの図解モデル・4 モードのブラウザ検証が成功しました。説明・図解・索引のローカルリンク 164 件、6 つの documentation_of の参照先、Markdown の usage 5 本のコンパイル・実行を確認しています。デスクトップ全モードと 390px 表示の画像も確認しました。

3.7 追加時も通常検証・ASan/UBSan・全カテゴリの図解モデル・11 モードのブラウザ検証が成功しました。Bellman–Ford の改善後に既存グラフ／木／旧 API の ASan/UBSan と 20 万頂点の回帰検証も実行済みです。説明・図解・索引のローカルリンク 265 件、16 の documentation_of、Markdown の usage 15 本をコンパイル・実行し、デスクトップと 390px 表示の画像も確認しました。

3.8 追加時も通常検証・ASan/UBSan・全カテゴリの図解モデル・7 モードのブラウザ検証が成功しました。既存グラフ／木／旧 API／二重連結分解の通常・ASan/UBSan・20 万頂点の回帰検証も実行済みです。説明・図解・索引のローカルリンク 242 件、9 の documentation_of、Markdown の usage 8 本をコンパイル・実行しました。デスクトップ各モード・仮削除・不等式の比較・390px 表示の画像も確認しています。

3.9 追加時も通常検証・ASan/UBSan（debug/release）・全カテゴリの図解モデル・3 モードのブラウザ検証が成功しました。20 万頂点のフロー経路と 20 万左頂点の交互路も検証済みです。説明・図解・索引のローカルリンク 207 件、4 の documentation_of、Markdown の usage 3 本をコンパイル・実行しました。逆辺・交互路・最小カットの画像と 390px の表示を確認し、交差辺のラベルを調整後にもブラウザ検証を実行しています。

3.10 追加時も通常検証・ASan/UBSan（debug/release）・全カテゴリの図解モデル・6 モードのブラウザ検証が成功しました。コールバックの const 参照の契約を反映後も再検証しています。説明・図解・索引のローカルリンク 248 件、8 の documentation_of、Markdown の usage 7 本をコンパイル・実行しました。枝刈り・最小彩色・全域木・390px 表示に加え、DP 表と行交換直後の余因子／元ラプラシアンの画像も確認しています。既存カテゴリとの混在 include と複数翻訳単位でのリンクも通っています。

3.11 追加時も通常検証・ASan/UBSan（debug/release）・全カテゴリの図解モデル・7モードのブラウザ検証が成功しました。既存グラフ／木／旧APIの通常・ASan/UBSan・20万頂点の回帰検証も成功。説明・図解・索引のローカルリンク290件、7つのdocumentation_of、Markdownのusage7本をコンパイル・実行しました。根変更・森・空・復路・全方位の除外表・巨大kの遷移を操作し、デスクトップ各モードと390pxの画像も確認しています。この環境では既存のNode/ChromiumがPATHになかったため公式バイナリを一時展開しましたが、/tmpのファイルは永続依存にしていません。

3.12 追加時も通常検証・ASan/UBSan（debug/release）・全カテゴリの図解モデル・12モードのブラウザ検証が成功しました。ローカルリンク343件、12のdocumentation_of、Markdownのusage11本をコンパイル・実行しました。0-1／完全の走査方向、個数制限の期限切れ、重さ0、64ビット語、桁のtight/started、4種類の部分集合操作、復元と障害物を操作し、デスクトップ各モード・状態表・390pxの画像も確認しています。以前の木カテゴリの未コミット変更を保持し、新規カテゴリと共有索引だけを追加更新しました。

3.13 追加時も通常検証・ASan/UBSan（debug/release）・全カテゴリの図解モデル・8モードのブラウザ検証が成功しました。説明・図解・索引のローカルリンク315件、8つのdocumentation_of、Markdownのusage8本をコンパイル・実行しました。負数・0・1・LCM超過・逆元なし・63ビットの指数・篩の除去・SPFの分解経路を操作し、デスクトップ各モードと390pxの画像を確認しています。SVG内の文字と箱の範囲も検査します。3.11/3.12の未コミット変更と旧number_theory/を保持しました。

3.14 追加時も通常検証・ASan/UBSan（debug/release）・全カテゴリの図解モデル・7モードのブラウザ検証が成功しました。ローカルリンク327件、7つのdocumentation_of、Markdownのusage7本をコンパイル・実行しました。素数の冪・減算の向き・3段階の畳み込み・合成数法／法1・0拡張・切り捨て・添字0・異なる長さ・64ビットの係数を操作し、デスクトップ各モード・途中の表・390pxの画像を確認しています。SVGの文字・箱の描画範囲も検査します。3.11〜3.13の未コミット変更を保持しました。

3.15 追加時も通常検証・ASan/UBSan（debug/release）・全カテゴリの図解モデル・7モードのブラウザ検証が成功しました。ローカルリンク338件、6つのdocumentation_of、Markdownのusage6本をコンパイル・実行しました。逆階乗の方向・逆元なし・Pascalと合成数法／法1・順序の区別・括弧列の反射・星と仕切り・空集合・負の途中係数・64ビット法・再生と入力エラーを操作し、SVGの描画範囲と390pxの横幅を検査しました。逆階乗・反射・負の係数・モバイルの画像も確認しています。オンラインジャッジでのACは未確認です。

3.16 追加時も通常検証・ASan/UBSan（debug/release）・全カテゴリの図解モデル・6モードのブラウザ検証が成功しました。説明・図解・索引のローカルリンク353件、5つのdocumentation_of、Markdownのusage5本をコンパイル・実行しました。ビット反転・バタフライ・単位根・0埋め・各点積・逆変換・3法CRTとLLONG_MIN・実数／複素数の積・再生・拒否条件を操作し、SVG全体の描画範囲と390pxの幅を検査。単位根・CRTの極値・モバイルの画像も確認しています。65,537×65,537の全1列のdouble FFTは、この環境で最大絶対誤差約1.61×10^-7でした（一般の誤差保証ではない）。3.15の未コミット変更を保持しました。オンラインACは未確認です。

3.17 追加時も通常検証・ASan/UBSan/LeakSanitizer（debug/release）・全カテゴリの図解モデル・12モードのブラウザ検証が成功しました。説明・図解・索引のローカルリンク467件、11のdocumentation_of、Markdownのusage11本をコンパイル・実行しました。先頭0と次数の移動、Newton法の補正・確定精度、平方根なし、除算の途中、積木・剰余・補間の重み、Taylor積、Bostan–Moriの偶奇とuint64最大値、再生・入力拒否を操作し、SVGの描画範囲と390px表示を検査。精度倍増・積木・モバイルの画像も確認しました。サンドボックス内ではLeakSanitizerがptrace制約、Chromiumがソケット制約で停止したため、承認手順でサンドボックス外実行して全検査を通しました。3.15・3.16の未コミット変更を保持しています。オンラインACは未確認です。

3.18 追加時も通常検証・ASan/UBSan/LeakSanitizer（debug/release）・全カテゴリの図解モデル・4モードのブラウザ検証が成功しました。説明・図解・索引のローカルリンク452件、4のdocumentation_of、Markdownのusage4本をコンパイル・実行しました。列状態と行列の二乗、定数1を含む状態・合成数法／法1、Kitamasaの高次還元、BMの不一致・次数増加・遅い非0・末尾0、母関数P/Qへの変換と偶奇、uint64最大値、入力拒否・再生を操作しました。SVGの描画範囲と390pxの幅、行列・還元・不一致・母関数・モバイルの画像も確認しています。環境のptrace／ソケット制限を避け、サニタイザとChromiumは承認手順でサンドボックス外実行。3.15〜3.17の未コミット変更を保持しました。オンラインACは未確認です。

3.19 追加時も通常検証・ASan/UBSan/LeakSanitizer（debug/release）、旧DPカテゴリの通常・サニタイザ回帰、全カテゴリの図解モデル、7モードのブラウザ検証が成功しました。説明・図解・索引のローカルリンク454件、7つのdocumentation_of、Markdownのusage7本をコンパイル・実行しました。列挙順と次のmask、64ビットの終端、4変換の方向と逆のビット順、寄与と交互和、汎用SOS、合成数法・法1、入力拒否・再生、SVGの描画範囲と390pxの横幅を確認。変換途中・64ビット列挙・モバイルの画像も確認しました。サニタイザとChromiumは承認手順でサンドボックス外実行。開始時に3.15〜3.18をbbd20a6へコミットし、3.19は未コミットで残しています。オンラインACは未確認です。

3.20 追加時も通常検証・ASan/UBSan/LeakSanitizer（debug/release）、全カテゴリの図解モデル、5モードのブラウザ検証が成功しました。説明・図解・索引のローカルリンク463件、5つのdocumentation_of、Markdownのusage5本をコンパイル・実行しました。長方形の転置と内積、二分累乗の不変条件・uint64最大指数、行交換・正規化・消去、特解と零空間・矛盾、素数法、64ビットXOR・従属・所属判定、再生と拒否条件を操作。SVGの描画範囲、390px表示と、途中状態・解空間・最上位bit・モバイルの画像も確認しました。サニタイザとChromiumは承認手順でサンドボックス外実行。3.19の既存変更を保持し、3.20も未コミットです。オンラインACは未確認です。

3.21 追加時も通常検証・ASan/UBSan/LeakSanitizer（debug/release）、全カテゴリの図解モデル、8モードのブラウザ検証が成功しました。説明・図解・索引のローカルリンク491件、8つのdocumentation_of、Markdownのusage8本をコンパイル・実行しました。Zの区間コピー、borderへの戻りと重複一致、先頭ハッシュの減算とLCPの二分探索、Trieの個数・削除、suffixの順位の組とLCPの持越し、奇数・偶数の鏡像、空・00/FF・UTF-8、再生・入力拒否を操作。SVGの描画範囲と390pxの幅、途中状態・Trie・回文・モバイルの画像も確認しました。base257の既知の二重衝突を通常・サニタイザの単体検証にも追加。サニタイザとChromiumは承認手順でサンドボックス外実行。3.19・3.20の既存変更を保持し、3.21も未コミットです。オンラインACは未確認です。

3.22 追加時も通常検証・ASan/UBSan/LeakSanitizer（debug/release）、全21カテゴリの図解モデル、12モードのブラウザ検証が成功しました。説明・図解・索引のローカルリンク550件、12のdocumentation_of、Markdownのusage12本をコンパイル・実行しました。整数の分数／浮動小数点のEPS、点に退化した線分、平行・同一直線、射影、凸包のpop、面積・winding・扇形二分探索、有理切断、対蹠点と境界の帯、空・一点・重複、再生・入力拒否を操作。SVGの描画範囲と390px表示、切断・凸包途中・モバイルの画像を確認しました。長い結果のモバイル折り返しも修正。サニタイザとChromiumは承認手順でサンドボックス外実行。3.19〜3.21の既存変更を保持し、3.22も未コミットです。オンラインACは未確認です。

3.23 追加時も通常検証・ASan/UBSan/LeakSanitizer（debug/release）、全22カテゴリの図解モデル、5モードのブラウザ検証が成功しました。説明・図解・索引のローカルリンク521件、5つのdocumentation_of、Markdownのusage5本をコンパイル・実行しました。Hungarianのpotential/slack、長方形・禁止辺・最小／最大・64ビット費用、Blossom・複数奇数閉路、二部交互路、逆辺での送流組替え、負費用・全成分の負閉路拒否・折れ線・不足・流量0、空・入力拒否・再生を操作。SVG描画範囲、390px幅と図の横スクロール、縮約・slack・逆辺・モバイルの画像を確認しました。最小費用流の頂点配置とモード別の破線凡例も調整。サニタイザとChromiumは承認手順でサンドボックス外実行。3.19〜3.22の既存変更を保持し、3.23も未コミットです。オンラインACは未確認です。

3.24 追加時も通常検証・ASan/UBSan/LeakSanitizer（debug/release）、全23カテゴリの図解モデル、4モードのブラウザ検証が成功しました。説明・図解・索引のローカルリンク514件、4つのdocumentation_of、Markdownのusage4本をコンパイル・実行しました。64ビットXORと表示区間、Misèreの偶奇・終盤・最後の一個・空、Kahn/mex・独立和、勝ち手と任意合法手の適用、重複する駒／辺・到達不能成分の閉路、入力拒否・再生を操作。SVG描画範囲と390px幅・図の横スクロール、上位ビット・mex・勝ち手・モバイルの画像も確認しました。サニタイザとChromiumは承認手順でサンドボックス外実行。3.19〜3.23の既存変更を保持し、3.24も未コミットです。オンラインACは未確認です。

3.25 追加時も通常検証・ASan/UBSan/LeakSanitizer（debug/release）、全24カテゴリの図解モデル、5モードのブラウザ検証が成功しました。説明・図解・索引のローカルリンク521件、5つのdocumentation_of、Markdownのusage5本をコンパイル・実行。64ビット乱数と棄却、0秒／期限ちょうど、SAの悪化採用・棄却・現在／最良・再現・両目的／冷却、Beamの同点・重複・削減・枯渇、局所探索の両モード・平坦、入力拒否・再生を操作しました。SVG範囲と390px幅、悪化採用・候補削減・モバイルの画像も確認。サニタイザとChromiumは承認手順でサンドボックス外実行。3.19〜3.24の既存変更を保持し、3.25も未コミットです。オンライン提出・スコア改善は未確認です。

- グラフ・木: 1,920 ランダムケース、旧 API、深いパス。`--stress` は 20 万頂点。
- 3.2: 併合／制約履歴の全探索、1,050 ランダム系列、20 万要素、3 ジャッジ形式。
- 3.3: 720 ランダム系列・154,400 操作、20 万要素、5 ジャッジ形式を各 1,200 クエリで比較。debug/release 両方で検査する。
- 3.4: 三値配列 9,841 個の全探索、240 FIFO 履歴・120,000 操作、20 万要素、比較／集約回数の線形上限、5 ジャッジ用プログラム。debug/release 両方。
- 3.5: 486 頻度状態の全探索、400 履歴・152,000 更新操作、昇降順・異種キー・独自比較器・LLONG_MAX、20 万座標、4 ジャッジ用プログラム。debug/release 両方。
- 3.6: 1,093 三値配列の全区間、M=0..80 の全成立境界、200 グラフ履歴、300 点更新履歴・48,000 操作、625 格子長方形、極値座標、独自型、20 万要素／状態／操作・50 万 CDQ イベント。6 単独ヘッダ・4 使用例・4 ジャッジ形式を各 600 クエリで比較。debug/release 両方。
- 3.7: 16 ヘッダ・4 使用例・10 ジャッジ形式／250 ケース。600 グラフを歩道 DP・再帰 DFS・順列 DAG・辺部分集合 MSF と比較し、全復元経路の始終点・向き・重みも確認。20 万頂点と 1 辺の負閉路。既存 graph/tree・旧 API の回帰検証も実行。
- 3.8: 9 ヘッダ・4 使用例・5 ジャッジ形式／240 ケース。531 有向グラフ・760 無向多重グラフの全探索、600 ランダムグラフ、20 万頂点。相互到達性、縮約の重複と元辺、辺／頂点削除、low と部分木、全彩色を検証。既存木・旧 API・二重連結分解も回帰検証。
- 3.9: 4 ヘッダ・3 使用例・3 ジャッジ形式／192 ケース。738 容量割当・689 二部グラフの全探索、700 組のランダム入力、全カット列挙・部分集合 DP。流量保存・残余到達・各辺の上限・リセット・容量型の極値／超過、20 万頂点のフロー経路と 20 万左頂点の交互路。debug/release 両方。
- 3.10: 9 ヘッダ（内部補助含む）・4 使用例・6 ジャッジ形式／288 ケース。1,100 単純グラフの全探索、240 ランダム構造、240 多重グラフ／行列。全頂点／辺部分集合・色割当・行列式の全順列と比較。全余因子、負の重み・整数極値・素数2と最大int素数、64ビット・22頂点の境界、Cayley、20万頂点の疎グラフ。debug/release 両方。
- 3.11: treeの10ヘッダ・使用例7本・7ジャッジ形式／336ケース。339個の非空な小さな森を全根で、300ランダム入力を全頂点対で、700遷移表を全探索。BFSパス・再帰DFS・全根DP・全頂点部分集合と比較。非可換／S≠R／送受信方向、uint64境界、距離超過、20万頂点のパス・星型・遷移の閉路と演算回数の線形上限。debug/release両方。
- 3.12: 14ヘッダ・11使用例・12ジャッジ形式／576ケース。9,841三値配列、3,969LCS組、225編集グラフ、6,175品物列＋400ランダム入力、200隣接結合、桁和の全上限0〜1000・法1〜9、300TSP全順列、5,058二値グリッドを独立解と比較。復元・0重さ・負値・整数極値／超過・64ビット境界、20万要素／容量、10万桁、18頂点TSP、160要素区間DP、長さ2,000文字列。debug/release両方。
- 3.13: 9ヘッダ・8使用例・9ジャッジ形式／432ケース。40,401符号付き小整数対、0〜10,000の素数・SPF、2〜10,000の約数・分解を全走査と比較。Python任意精度20,630ケースでGCD・LCM・Bézout係数・mod積／冪／逆元、連続Fibonacciと64ビット極値を検証。LCMのULLONG_MAXちょうど・超過、合成数法・法1、既知のLLONG_MAXの分解、10^12の約数、200万の篩。debug/release両方。
- 3.14: 10ヘッダ・7使用例・3ジャッジ形式／288ケース。10,242列の4変換、8,586組の畳み込みを直接の約数／倍数走査・μ・全添字対と比較。符号付き整数・998244353・法12・非可換行列、空／長さ違い／添字0／上限、8ビット整数の393,216演算を独立な広い整数型と照合。100万上限の全出力をτ・φ・素因数の指数による式で確認し、素数ごとの更新回数とDirichletの調和級数の回数も検証。debug/release両方。
- 3.15: 8ヘッダ・6使用例・6ジャッジ形式／384ケース。361個の有効な階乗表と889個の逆元なし、69,905集合族、Python任意精度186ブロック・6,531出力を検証。部分集合・順序付き選択・括弧列・弱合成の全列挙、旧実装との比較、法1・合成数法・64ビット極値・途中の整数超過、200万要素の階乗表・1500行のPascal表・22集合の包除を確認。debug/release両方。
- 3.16: 7ヘッダ・5使用例・5ジャッジ形式／360ケース。14,641配列対の全探索、8法／根の直接有限体DFT、複素DFT／逆／共役対称性、400整数積・300複素数積、Python任意精度858組・2,838出力。2^20点NTT・10万×10万のmod／整数積・65,537×65,537の実数FFT、全整数境界・CRTの符号・保守的上限・NaN/Inf・非有限途中値を検証。debug/release両方。
- 3.17: 14ヘッダ・11使用例・10ジャッジ形式／480ケース。1,093三値級数を9精度で、240多項式・300漸化式、Python任意精度4,352クエリを係数漸化式・二分累乗・Horner・直接Lagrange・特性多項式剰余と比較。65,536項のNewton／shift／除算・8,192点の積木・4,096次とuint64の漸化式、各境界・拒否条件。debug/release両方。
- 3.18: 6ヘッダ・4使用例・5ジャッジ形式／320ケース。BMの9,233有限列を独立Gaussianの最小可解次数と比較。600漸化式・160任意行列・Python任意精度3,504クエリ、Kitamasaの全重み、法1・合成数・非NTT素数・intとuint64極値を検証。192次Kitamasa・2,048観測／次数1,024のBM・48×48密行列・16,384次Bostan–Moriと2^63項。debug/release両方。
- 3.19: 9ヘッダ・7使用例・4ジャッジ形式／256ケース。2,047基準mask・177,146組の列挙、6,654三値表・180ランダム表、Python任意精度2,428クエリを集合の定義から独立比較。全int8加減算262,144組、符号付き／なしの極値・途中超過、mod12・複素数・単位元不要の型、2^20項の全出力と更新回数を検証。旧DPの通常・サニタイザ回帰も実施。debug/release両方。
- 3.20: 8ヘッダ・5使用例・4ジャッジ形式／256ケース。500長方形積・1,950反復累乗比較、有限体5,915系の全解空間、実数160系、有理数・独自mod型・複素数、65,536 int8積、37,449 XOR系列・100全幅系列。Python任意精度1,361クエリを全小行列式の順列和・内積・部分集合と比較。128変数の消去・256自由変数の基底・32×32巡回行列のuint64最大指数・64個のpivot・10万挿入、空次元・数値超過と非有限値を確認。debug/release両方。
- 3.21: 10ヘッダ・8使用例・8ジャッジ形式／512ケース。9,841三値文字列・180全byte列、3,937 KMP組、120 Trie履歴48,000操作、300ハッシュ組6,000区間クエリ。Python任意精度4,126クエリ（Trie2,880操作）をprefix/border/全suffix/回文/多項式/辞書から独立比較。20万文字の同一・交互列／深いTrie、比較回数の線形上限、汎用整数極値・00/FF・空・重複・base境界・既知の二重衝突と入力拒否を確認。debug/release両方。
- 3.22: 17ヘッダ・12使用例・12ジャッジ形式768ケース、Python Fraction 2,244ケース。18,496有理数組・729CCW・6,561線分組、512点集合と12,800包含、300切断・600点群をgift wrapping／有理射線／全点対と比較。128ビット境界・退化・拒否条件、20万点の最近点対と20,001頂点のキャリパーの演算回数も検証。debug/release両方。
- 3.23: 7ヘッダ・5使用例・5ジャッジ形式320ケース、Python任意精度1,099ケース。禁止辺行列4,458種類＋800割当・全33,868グラフ＋1,400多重グラフ・100二部比較、1,130ネットワークの全整数フロー。双対証明、逆辺・slope・追加送流・全成分の負閉路・整数極値と拒否、350×500割当・500頂点密グラフ・20万頂点送流。debug/release両方。
- 3.24: 5ヘッダ・4使用例・4ジャッジ形式256入力＋10万石。Python1,028ケース。Nim／Misereの19,531山列、21,845 mex列、全1,100順序付きDAG＋350多重DAGの全単体／3成分積ゲームを独立比較。終局・勝ち手・任意target・Graph互換・64ビット・閉路／不正値拒否、20万山／頂点／成分／mex、1,500密DAG・100万重複辺。debug/release両方。
- 3.25: 7ヘッダ・不正型拒否・逆順混在include・複数翻訳単位・5使用例。8整数型と全64ビット範囲・棄却・real格子／shuffle、模擬Clockの境界／reset／無限予算。24,060全列局所探索・2,000 Beam・800 SAを別手順と照合、Python任意精度／探索1,550ケース。部分走査／層中断・現在／最良・乱数消費・差1と符号両端・非有限入力・非デフォルト状態、20万回改善と幅200深さ200。debug/release両方。
- 図解: 3.2 は独立 BFS、3.3 / 3.4 は素朴配列・列の集約によりモデルを検証。3.4 は 189,940 操作、移し替え途中も確認。
- 3.5 の図解: 200 入力・133,200 操作を素朴配列・辞書と照合。失敗時の状態維持と BIT ブロックの値も確認。
- 3.6 の図解: 647 バッチ・53,519 スナップショットを素朴配列・BFS・時刻付き点列と比較。全 BIT ブロック、探索範囲、時刻順、巻き戻し後の 0 を確認。

- 3.7 の図解: 1,042 バッチ・11,140 スナップショットを独立オラクルと比較。距離と負閉路の影響、全成分 DFS、閉路の向き、復元経路、MST の最小性を確認。

- 3.8 の図解: 1,559 ケース・37,101 スナップショットを独立オラクルと照合。SCC、縮約 DAG、根の特例、平行辺、仮削除・彩色と衝突を確認。
- 3.9 の図解: 1,479 バッチ・20,355 スナップショットを全カット列挙・右部分集合 DP と照合。全フレームの流量保存・容量上限・左右の対応、上限停止・逆辺・交互路・カット元辺 ID を確認。
- 3.10 の図解: 1,809 バッチ・34,537 スナップショット。全列挙・極大性・最適性・枝刈り上界・全DP状態・重み付き木・各行消去・行交換・0 mod p・スナップショット独立性を確認。

- 3.11の図解: 1720バッチ・34238スナップショット。全有向メッセージ・prefix/suffix、部分木と往復列、独立集合の全状態、64ビットの閉路走査、スナップショット独立性を確認。

- 3.12の図解: 1,784入力・19,679スナップショット。更新途中の表、候補キュー、十進接頭辞、集合変換・巡回路、語境界・復元列・長方形・スナップショット独立性。

- 3.13の図解: 8,529入力・58,563スナップショット。約数／素数／分解の全走査、反復乗算・剰余周期、恒等式を独立オラクルとする。全係数・SPFの一度だけの生成・篩の候補・約数対・整数境界・スナップショット独立性を確認。

- 3.14の図解: 4,082入力・70,761スナップショット。素因数の集合を制限した定義式で各更新途中の行を独立に計算。μ・添字対・各素数とkの方向、0・空の正の範囲・長さ違い・合成数法・出力の打ち切り・整数境界・スナップショット独立性を確認。

- 3.15の図解: 3,011入力・32,543スナップショット、96個の拒否条件。部分集合・順序付き選択・括弧列の列挙、弱合成のDP、要素ごとの包除係数を独立オラクルとする。階乗・逆階乗の全エントリ、Pascalの各行、空集合の呼び出し、負の途中係数、合成数法・法1・64ビット法・スナップショット独立性を確認。

- 3.16の図解: 1,017入力・73,452スナップショット、17個の拒否条件。有限体／複素数の直接DFT、ビット反転に対応する各部分列のDFT、素朴な多項式積とCRTを独立比較。各バタフライ・全段階・逆の正規化・0埋め・極値・スナップショット独立性を確認。

- 3.17の図解: 4,116入力・26,757スナップショット、15拒否条件。log/expの級数定義、平方根候補全走査、各Newton精度・除算恒等式・全積木、Bostan–Moriの偶奇と縮約係数を独立に比較。

- 3.18の図解: 3,446入力・55,104スナップショット、15拒否条件。逐次状態遷移・単位初期値の数列・各接頭辞のGaussian最短次数を独立オラクルとする。全還元・BM不一致・母関数の高次消滅・偶奇抽出・uint64最大値・スナップショット独立性を確認。

- 3.19の図解: 2,853入力・50,838スナップショット、15拒否条件。処理済みビットを制限した集合の定義を独立オラクルに、全中間表・4方向と逆のビット順・寄与と符号、SOSの4演算、列挙順と64ビット境界、スナップショット独立性を検証。

- 3.20の図解: 2,413入力・27,172スナップショット、17拒否条件。直接の転置・内積の接頭辞・反復累乗、行の同値性、有限体の全解、XORの部分集合・問い合わせの剰余類を独立比較。空次元・自由列・基底の全出力・uint64境界とスナップショット独立性を確認。

- 3.21の図解: 3,096入力・51,386スナップショット、16拒否条件。直接のprefix/border/全一致/suffix/半径、多項式の和と辞書を独立比較。既知区間の再利用・fallback後の状態・順位と各組・LCPの持越し・Trie各ノード・00/FF/UTF-8・スナップショット独立性を確認。
- 3.22の図解: 5,016入力・38,475スナップショット、13拒否条件。独立なgift wrapping／直線方程式／射線／切断頂点の列挙／全点対を比較し、途中の凸な鎖・面積の部分和・分数・EPSと近接距離・スナップショット独立性も確認。
- 3.23の図解: 2,782入力・27,955スナップショット・16拒否条件・719回のBlossom縮約。全単射・頂点部分集合・全整数フローを独立比較。双対・slack・mate/base・流量保存・逆辺経路・slope・スナップショット独立性も検証。
- 3.24の図解: 5,366入力・93,244スナップショット、3,057合法勝ち手、19拒否条件。完全ゲーム木・集合mex・積ゲーム・到達閉包の閉路判定から独立比較。XOR接頭辞・入次数・mexの記録・独立和・勝ち手の実適用とスナップショット独立性を確認。
- 3.25の図解: 26,367入力・110,981スナップショット・25拒否条件。64ビット定義・時刻計算・独立近傍歩行／採否・候補逐次選択と照合。SA両目的／冷却、採否・乱数消費・最良保持、Beam安定同点／重複／候補枯渇、局所探索の平坦・両モード、フレーム独立性を確認。

ブラウザ操作の検証には Node.js 22 以降と Chromium が必要です。npm パッケージは不要です。

```sh
node verify/visualizations/browser_test.mjs --browser /path/to/chromium
node verify/visualizations/range_query_browser_test.mjs --browser /path/to/chromium
node verify/visualizations/static_query_browser_test.mjs --browser /path/to/chromium
node verify/visualizations/ordered_structure_browser_test.mjs --browser /path/to/chromium
node verify/visualizations/offline_query_browser_test.mjs --browser /path/to/chromium
node verify/visualizations/graph_basic_browser_test.mjs --browser /path/to/chromium
node verify/visualizations/graph_connectivity_browser_test.mjs --browser /path/to/chromium
node verify/visualizations/graph_flow_browser_test.mjs --browser /path/to/chromium
node verify/visualizations/graph_combinatorics_browser_test.mjs --browser /path/to/chromium
node verify/visualizations/tree_browser_test.mjs --browser /path/to/chromium
node verify/visualizations/dp_optimization_browser_test.mjs --browser /path/to/chromium
node verify/visualizations/number_theory_basic_browser_test.mjs --browser /path/to/chromium
node verify/visualizations/number_theory_convolution_browser_test.mjs --browser /path/to/chromium
node verify/visualizations/combinatorics_browser_test.mjs --browser /path/to/chromium
node verify/visualizations/convolution_browser_test.mjs --browser /path/to/chromium
node verify/visualizations/fps_browser_test.mjs --browser /path/to/chromium
node verify/visualizations/linear_recurrence_browser_test.mjs --browser /path/to/chromium
node verify/visualizations/set_function_browser_test.mjs --browser /path/to/chromium
node verify/visualizations/linear_algebra_browser_test.mjs --browser /path/to/chromium
node verify/visualizations/string_algorithm_browser_test.mjs --browser /path/to/chromium
node verify/visualizations/computational_geometry_browser_test.mjs --browser /path/to/chromium
node verify/visualizations/combinatorial_optimization_browser_test.mjs --browser /path/to/chromium
node verify/visualizations/game_theory_browser_test.mjs --browser /path/to/chromium
node verify/visualizations/heuristic_search_browser_test.mjs --browser /path/to/chromium
```

`--screenshot /tmp/example.png` でデスクトップ・モバイルの画像を保存できます。全モードの操作・再生・入力エラー・ローカルリンク・390px の横幅を確認します。

この環境では、サンドボックス内の LeakSanitizer が ptrace 制限、Chromium がソケット制限で失敗する場合がありました。その場合はエラーを確認し、環境の承認手順で同じチェックをサンドボックス外で実行します。検証機能を無効にして成功扱いにしません。以前のセッションで作った `/tmp/` のブラウザ補助ファイルは永続依存にしません。

## 図解の追加・確認

現在はローカルで直接開ける HTML/CSS/JavaScript/SVG を使っています。外部通信・CDN・npm 導入なしで動作します。共有 CSS は `visualizations/assets/style.css` です。モデルと画面コードを分け、モデルは Node.js でも実行できます。

入力変更、操作の実行、前後のステップ、再生、サンプル操作列を基本とします。どのノードが参照／変更されたか、何を保持するか、各ステップの理由が分かるようにします。内部状態を簡略化する場合は、その範囲を明記してください。

モデルの期待値は内部状態から再計算するだけで済ませず、独立した素朴解と比較します。更新途中の一時的に不整合な状態から別操作を始めないようにし、操作完了まで進める等の扱いを設けます。スマホではページ全体の横はみ出しを避け、必要なら図の領域だけをスクロールさせます。

## 参考にするライブラリ

必要な分野の一次資料を選んで実際に読み、`references.md` に参考箇所と設計の違いを残します。すべてのライブラリを毎回調べる必要はありません。

[ACL](https://github.com/atcoder/ac-library)、[Luzhiled](https://ei1333.github.io/library/)、[Nyaan](https://nyaannyaan.github.io/library/)、[maspypy](https://maspypy.github.io/library/)、[suisen](https://suisen-cp.github.io/cp-library-cpp/)、[hitonanode](https://hitonanode.github.io/cplib-cpp/)、[sotanishy](https://sotanishy.github.io/cp-library-cpp/)、[noshi91](https://noshi91.github.io/Library/)、[kmyk](https://kmyk.github.io/competitive-programming-library/)、[KACTL](https://github.com/kth-competitive-programming/kactl)、[CP-Algorithms](https://cp-algorithms.com/)、[Library Checker](https://judge.yosupo.jp/)。

コードを取り込む場合はライセンスと出典を確認して保持します。説明文はこの API に合わせて書きます。

## 新しいセッションへの依頼テンプレート

A全169掲載箇所、B全168掲載箇所が完了しています。次の対象を依頼する場合はカテゴリ・ランクを明示し、次の文面を使えます。

追加候補や元Cを進める場合は、[統合一覧](docs/remaining_work.md)のIDを指定すると重複や既存機能の再実装を避けられます。「要仕様化」は対象操作・条件を確認してから実装します。

```text
cp_library/HANDOFF.md を読んで、現在の変更とブランチを確認してから続けてください。
docs/implementation_plan.md を読み、既存の変更を保持して、下で指定する対象の不足分を
カテゴリ順に進めてください。実装済み項目は再実装しないでください。

対象を指定:
### 3.x カテゴリ名
- B: 実装してほしい項目...

3.xx ごとに独立した実装フォルダにしてください。
Markdown の説明・usage・計算量・問題例に加え、docs/ とは別の
visualizations/ に操作できる図解も用意し、各索引からリンクしてください。
適切な検証を実行し、完了後に docs/implementation_plan.md のチェック・参照先と
HANDOFF.md の現状も更新してください。
```
