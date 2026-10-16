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

.PHONY: verify-union-find
verify-union-find:
	python3 verify/run_union_find.py

.PHONY: verify-visualizations
verify-visualizations:
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
