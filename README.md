# procon Library

This repository hosts a competitive programming library that aggregates reusable data structures,
number theoretic utilities, graph routines, and other building blocks collected from actual
contest use.  The goal is to publish the library as a public resource together with thorough
documentation and automated verification.

## Highlights
- Modular components grouped by topic under `cp_library/`
- Header-only style for easy copy & paste into submissions
- Ready for automated verification with [online-judge-tools](https://github.com/online-judge-tools/oj)
- GitHub Pages site (see `docs/`) for human-readable documentation and examples

## Directory Layout
```
cp_library/
  Combination/          # Precomputed factorials, binomial coefficients, etc.
  DP/                   # Dynamic programming helpers (e.g. TSP DP templates)
  data_structure/       # Union-find, bit Fenwick trees, segment trees, ...
  geometry/             # 2D geometry primitives and algorithms
  graph_theory/         # Graph traversal, flow, shortest path
  matrix/               # Matrix utilities and linear algebra helpers
  number_theory/        # Modular arithmetic, primality, convolutions
  string/               # String algorithms and hashing helpers
  misc/                 # Utility helpers that do not fit other categories
  test/                 # Auto-generated test assets (ignored by Git)
  test.sh               # Test runner script (see below)
```

Additional sample solutions (`main.cpp`, `naive.cpp`, etc.) live in the project root and are
not part of the published library.

## Getting Started
1. Install a modern C++ compiler (GCC 10+ or Clang 11+ recommended).
2. Install [online-judge-tools](https://github.com/online-judge-tools/oj) (OJ) locally.
   ```bash
   pip install -U online-judge-tools
   ```
3. Ensure you can compile a simple program:
   ```bash
   g++ -std=c++20 -O2 -pipe -Wall -Wextra sample.cpp
   ```
4. Copy the needed headers (or include directly via `-I cp_library`).

## Running the Test Suite
We follow the workflow proposed in the ["How to run CI on your library" guide](https://online-judge-tools.readthedocs.io/en/develop/run-ci-on-your-library.html).

- Place verification code under a file with suffix `.test.cpp`.
- Specify the source problem at the top of each file:
  ```cpp
  #define PROBLEM "https://judge.yosupo.jp/problem/unionfind"
  ```
- Run the automated checker:
  ```bash
  bash test.sh                # run all .test.cpp files
  bash test.sh path/to/foo.test.cpp   # run a single test file
  ```

The runner compiles each test, downloads sample/system cases via `oj`, and executes them locally.
Artifacts are stored under `test/<hash>/`, so the repository must ignore that directory (see
`.gitignore`).

## Continuous Integration
A ready-to-use GitHub Actions workflow lives under `.github/workflows/oj-ci.yml`.  It
executes `test.sh` on Ubuntu with both GCC and Clang so every pull request is automatically
verified.  The workflow installs online-judge-tools and caches downloaded test cases between
runs.

Badges/examples can be added once the repository is public.  For now, keep CI green by ensuring
`test.sh` succeeds before pushing.

## Documentation Site
Static documentation is served from the `docs/` directory via GitHub Pages (Pages → Source →
"Deploy from branch" with `/docs`).  The landing page (`docs/index.md`) mirrors the README and
adds extra usage and contribution guides.  Feel free to extend it with component-specific
reference pages written in Markdown.

## Contribution Guidelines
- Follow the existing header-only layout; keep each algorithm self-contained.
- For new modules, include at least one `.test.cpp` verifying AC on a public judge.
- Run `bash test.sh` locally before opening a pull request.
- Document tricky implementations briefly in comments to aid readers.

## License
Decide on a license (e.g., MIT/CC0) before publishing publicly.  Until then, keep the
repository private or restrict distribution.
