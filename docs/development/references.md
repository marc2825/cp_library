# 開発・検証基盤の参考資料

[一覧](README.md) / [図解](../../visualizations/development/index.html)

2026-10-19確認。公開API・CLIを確認し、このリポジトリ向けの小規模なローカルツールを実装しています。

- [competitive-verifier reference](https://competitive-verifier.github.io/competitive-verifier/document.html): PROBLEM/STANDALONE、C++設定、依存解決、documentation_of。固定版4.1.2は実際に/tmpの仮想環境へ導入しCLIのhelpと動作を確認。
- [competitive-verifier 4.1.2](https://pypi.org/project/competitive-verifier/4.1.2/): 使用するツールの配布版。[requirements](../../.competitive-verifier/requirements.txt)に解決した推移依存も記録。
- [Python subprocess](https://docs.python.org/3/library/subprocess.html): argv、cwd、start_new_session、timeoutとプロセス終了。
- [GCC Once-Only Headers](https://gcc.gnu.org/onlinedocs/cpp/Once-Only-Headers.html): include guardの意味。バンドラは独自の限定的な展開器であり、GCCプリプロセッサの代替実装ではない。
- [GitHub Actions artifacts](https://docs.github.com/en/actions/how-tos/writing-workflows/choosing-what-your-workflow-does/storing-and-sharing-data-from-a-workflow): 検証結果・生成文書の保存。

生成器は既存の[Random](../../heuristic_search/random.hpp)と同じSplitMix64の定数を使うPython開発用APIです。頂点のshuffle、先行頂点への親接続、全候補からの辺選択、放物線上の凸多角形という単純な構成を使い、出力の性質を独立検証しています。
