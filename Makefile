# https://triple-four.hatenablog.com/entry/20210623/1624458051#-Wall-%E3%81%A8--Wextra--%E3%81%84%E3%82%8D%E3%81%84%E3%82%8D%E3%81%AA%E8%AD%A6%E5%91%8A%E3%82%92%E3%81%BE%E3%81%A8%E3%82%81%E3%81%A6%E6%9C%89%E5%8A%B9%E5%8C%96
# https://img.atcoder.jp/file/language-update/language-list.html
# aliasコマンド で .bashrc に定義もしている
# alias g+='g++ -std=gnu++2a -O2 -g -mtune=native -march=native -fconstexpr-depth=2147483647 -fconstexpr-loop-limit=2147483647 -fconstexpr-ops-limit=2147483647 -Wall -Wextra -Wfloat-equal -ftrapv -fstack-protector-all -fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover -DMARC_LOCAL -o a.out'
# alias ainout='./a.out < input.txt > output.txt'
# ./bashrc ファイルにも定義しておく（aliasコマンドでの定義はOSの再起動で消えるので）


CC = g++
#CC = clang++
#CC = clang++ -lstdc++
VERSION = -std=gnu++2a
WARNINGS = -Wall -Wextra -Wfloat-equal
FLAGS = -ftrapv -fstack-protector-all -fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover
CFLAGS = -O2 -g -mtune=native -march=native -fconstexpr-depth=2147483647 -fconstexpr-loop-limit=2147483647 -fconstexpr-ops-limit=2147483647
DEFINE = -DMARC_LOCAL
INCDIR = -I .
TARGET = a.out


main: main.cpp
	$(CC) $(VERSION) $(WARNINGS) $(DEFINE) -o $(TARGET) $<

main2: main.cpp
	$(CC) $(VERSION) $(CFLAGS) $(WARNINGS) $(FLAGS) $(DEFINE) -o $(TARGET) $<

mainat: main.cpp
	$(CC) $(VERSION) $(WARNINGS) $(DEFINE) -DATCODER $(INCDIR) -o $(TARGET) $<

mainat2: main.cpp
	$(CC) $(VERSION) $(CFLAGS) $(WARNINGS) $(FLAGS) $(DEFINE) -DATCODER $(INCDIR) -o $(TARGET) $<

test: test.cpp
	$(CC) $(VERSION) $(WARNINGS) $(DEFINE) -o $(TARGET) $<

test2: test.cpp
	$(CC) $(VERSION) $(CFLAGS) $(WARNINGS) $(FLAGS) $(DEFINE) -o $(TARGET) $<

testat: test.cpp
	$(CC) $(VERSION) $(WARNINGS) $(DEFINE) -DATCODER $(INCDIR) -o $(TARGET) $<

testat2: test.cpp
	$(CC) $(VERSION) $(CFLAGS) $(WARNINGS) $(FLAGS) $(DEFINE) -DATCODER $(INCDIR) -o $(TARGET) $<

naive: naive.cpp
	$(CC) $(VERSION) $(WARNINGS) $(DEFINE) -o $(TARGET) $<

naive2: naive.cpp
	$(CC) $(VERSION) $(CFLAGS) $(WARNINGS) $(FLAGS) $(DEFINE) -o $(TARGET) $<

naiveat: naive.cpp
	$(CC) $(VERSION) $(WARNINGS) $(DEFINE) -DATCODER $(INCDIR) -o $(TARGET) $<

naiveat2: naive.cpp
	$(CC) $(VERSION) $(CFLAGS) $(WARNINGS) $(FLAGS) $(DEFINE) -DATCODER $(INCDIR) -o $(TARGET) $<

libcheck: libcheck.cpp
	$(CC) $(VERSION) $(CFLAGS) $(WARNINGS) $(FLAGS) $(DEFINE) -o $(TARGET) $<

poj: poj.cpp
	$(CC) $(VERSION) $(WARNINGS) $(DEFINE) -o $(TARGET) $<


.PHONY: clean
clean:
	rm -f $(TARGET) *.o

.PHONY: verify-graph-tree
verify-graph-tree:
	python3 verify/run_graph_tree.py

.PHONY: verify-provenance verification-report verify-development-records verify-stdlib-debug
verify-provenance:
	python3 -m tools.cp_tools.provenance

verification-report:
	python3 -m tools.cp_tools.catalog

verify-development-records:
	python3 verify/run_development_records.py

verify-stdlib-debug:
	python3 -m tools.cp_tools.suite --stdlib-debug --runners union_find range_query

.PHONY: verify-union-find
verify-union-find:
	python3 verify/run_union_find.py

.PHONY: verify-visualizations
verify-visualizations:
	node verify/visualizations/linear_algebra_extensions_model_test.js
	node verify/visualizations/string_extensions_model_test.js
	node verify/visualizations/geometry_extensions_model_test.js
	node verify/visualizations/polynomial_extensions_model_test.js
	node verify/visualizations/combinatorics_extensions_model_test.js
	node verify/visualizations/number_theory_extensions_model_test.js
	node verify/visualizations/dp_extensions_model_test.js
	node verify/visualizations/tree_extensions_model_test.js
	node verify/visualizations/graph_extensions_model_test.js
	node verify/visualizations/graph_advanced_model_test.js
	node verify/visualizations/data_structure_extensions_model_test.js
	node verify/visualizations/data_structure_advanced_model_test.js
	node verify/visualizations/utility_search_model_test.js
	node verify/visualizations/development_runtime_model_test.js
	node verify/visualizations/development_model_test.js
	node verify/visualizations/foundation_tools_model_test.js
	node verify/visualizations/big_integer_model_test.js
	node verify/visualizations/foundation_structures_model_test.js
	node verify/visualizations/foundation_model_test.js
	node verify/visualizations/b_21_25_model_test.js
	node verify/visualizations/b_16_20_model_test.js
	node verify/visualizations/b_12_15_model_test.js
	node verify/visualizations/b_7_11_model_test.js
	node verify/visualizations/b_1_6_model_test.js
	node verify/visualizations/final_a_model_test.js
	node verify/visualizations/transform_a_model_test.js
	node verify/visualizations/math_a_model_test.js
	node verify/visualizations/graph_a_model_test.js
	node verify/visualizations/a_priority_model_test.js
	node verify/visualizations/advanced_model_test.js
	node verify/visualizations/union_find_model_test.js
	node verify/visualizations/range_query_model_test.js
	node verify/visualizations/static_query_model_test.js
	node verify/visualizations/ordered_structure_model_test.js
	node verify/visualizations/offline_query_model_test.js
	node verify/visualizations/graph_basic_model_test.js
	node verify/visualizations/graph_connectivity_model_test.js
	node verify/visualizations/graph_flow_model_test.js
	node verify/visualizations/graph_combinatorics_model_test.js
	node verify/visualizations/tree_model_test.js
	node verify/visualizations/dp_optimization_model_test.js
	node verify/visualizations/number_theory_basic_model_test.js
	node verify/visualizations/number_theory_convolution_model_test.js
	node verify/visualizations/combinatorics_model_test.js
	node verify/visualizations/convolution_model_test.js
	node verify/visualizations/fps_model_test.js
	node verify/visualizations/linear_recurrence_model_test.js
	node verify/visualizations/set_function_model_test.js
	node verify/visualizations/linear_algebra_model_test.js
	node verify/visualizations/string_algorithm_model_test.js
	node verify/visualizations/computational_geometry_model_test.js
	node verify/visualizations/combinatorial_optimization_model_test.js
	node verify/visualizations/game_theory_model_test.js
	node verify/visualizations/heuristic_search_model_test.js

.PHONY: verify-range-query
verify-range-query:
	python3 verify/run_range_query.py

.PHONY: verify-static-query
verify-static-query:
	python3 verify/run_static_query.py

.PHONY: verify-ordered-structure
verify-ordered-structure:
	python3 verify/run_ordered_structure.py

.PHONY: verify-offline-query
verify-offline-query:
	python3 verify/run_offline_query.py

.PHONY: verify-graph-basic
verify-graph-basic:
	python3 verify/run_graph_basic.py

.PHONY: verify-graph-connectivity
verify-graph-connectivity:
	python3 verify/run_graph_connectivity.py

.PHONY: verify-graph-flow
verify-graph-flow:
	python3 verify/run_graph_flow.py

.PHONY: verify-graph-combinatorics
verify-graph-combinatorics:
	python3 verify/run_graph_combinatorics.py

.PHONY: verify-tree
verify-tree:
	python3 verify/run_tree.py

.PHONY: verify-dp-optimization
verify-dp-optimization:
	python3 verify/run_dp_optimization.py

.PHONY: verify-number-theory-basic
verify-number-theory-basic:
	python3 verify/run_number_theory_basic.py

.PHONY: verify-number-theory-convolution
verify-number-theory-convolution:
	python3 verify/run_number_theory_convolution.py

.PHONY: verify-combinatorics
verify-combinatorics:
	python3 verify/run_combinatorics.py

.PHONY: verify-convolution
verify-convolution:
	python3 verify/run_convolution.py

.PHONY: verify-fps
verify-fps:
	python3 verify/run_fps.py

.PHONY: verify-linear-recurrence
verify-linear-recurrence:
	python3 verify/run_linear_recurrence.py

.PHONY: verify-set-function
verify-set-function:
	python3 verify/run_set_function.py

.PHONY: verify-linear-algebra
verify-linear-algebra:
	python3 verify/run_linear_algebra.py

.PHONY: verify-string-algorithm
verify-string-algorithm:
	python3 verify/run_string_algorithm.py

.PHONY: verify-computational-geometry
verify-computational-geometry:
	python3 verify/run_computational_geometry.py

.PHONY: verify-combinatorial-optimization
verify-combinatorial-optimization:
	python3 verify/run_combinatorial_optimization.py

.PHONY: verify-game-theory
verify-game-theory:
	python3 verify/run_game_theory.py

.PHONY: verify-heuristic-search
verify-heuristic-search:
	python3 verify/run_heuristic_search.py

.PHONY: verify-advanced
verify-advanced:
	python3 verify/run_advanced.py

.PHONY: verify-a-priority verify-basic-utility
verify-a-priority:
	python3 verify/run_a_priority.py

verify-basic-utility: verify-a-priority

.PHONY: verify-graph-a
verify-graph-a:
	python3 verify/run_graph_a.py

.PHONY: verify-math-a
verify-math-a:
	python3 verify/run_math_a.py

.PHONY: verify-transform-a
verify-transform-a:
	python3 verify/run_transform_a.py

.PHONY: verify-final-a
verify-final-a:
	python3 verify/run_final_a.py

.PHONY: verify-b-1-6
verify-b-1-6:
	python3 verify/run_b_1_6.py

.PHONY: verify-b-7-11
verify-b-7-11:
	python3 verify/run_b_7_11.py

.PHONY: verify-b-12-15
verify-b-12-15:
	python3 verify/run_b_12_15.py

.PHONY: verify-b-16-20
verify-b-16-20:
	python3 verify/run_b_16_20.py

.PHONY: verify-b-21-25
verify-b-21-25:
	python3 verify/run_b_21_25.py

.PHONY: verify-foundation
verify-foundation:
	python3 verify/run_foundation.py

.PHONY: verify-foundation-structures
verify-foundation-structures:
	python3 verify/run_foundation_structures.py

.PHONY: verify-big-integer
verify-big-integer:
	python3 verify/run_big_integer.py

.PHONY: verify-foundation-tools
verify-foundation-tools:
	python3 verify/run_foundation_tools.py

.PHONY: verify-development verify-ci-offline verify-ci
verify-development:
	python3 verify/run_development.py

verify-ci-offline:
	python3 tools/verify_ci.py --offline

verify-ci:
	python3 tools/verify_ci.py

.PHONY: verify-development-runtime
verify-development-runtime:
	python3 verify/run_development_runtime.py

.PHONY: verify-utility-search
verify-utility-search:
	python3 verify/run_utility_search.py

.PHONY: verify-data-structure-extensions
verify-data-structure-extensions:
	python3 verify/run_data_structure_extensions.py

.PHONY: verify-data-structure-advanced
verify-data-structure-advanced:
	python3 verify/run_data_structure_advanced.py

.PHONY: verify-tree-extensions
verify-tree-extensions:
	python3 verify/run_tree_extensions.py

.PHONY: verify-graph-extensions
verify-graph-extensions:
	python3 verify/run_graph_extensions.py

.PHONY: verify-graph-advanced
verify-graph-advanced:
	python3 verify/run_graph_advanced.py

.PHONY: verify-dp-extensions
verify-dp-extensions:
	python3 verify/run_dp_extensions.py

.PHONY: verify-number-theory-extensions
verify-number-theory-extensions:
	python3 verify/run_number_theory_extensions.py

.PHONY: verify-combinatorics-extensions
verify-combinatorics-extensions:
	python3 verify/run_combinatorics_extensions.py

.PHONY: verify-polynomial-extensions
verify-polynomial-extensions:
	python3 verify/run_polynomial_extensions.py

.PHONY: verify-linear-algebra-extensions
verify-linear-algebra-extensions:
	python3 verify/run_linear_algebra_extensions.py

.PHONY: verify-string-extensions
verify-string-extensions:
	python3 verify/run_string_extensions.py

.PHONY: verify-geometry-extensions
verify-geometry-extensions:
	python3 verify/run_geometry_extensions.py
	node verify/visualizations/geometry_extensions_model_test.js
