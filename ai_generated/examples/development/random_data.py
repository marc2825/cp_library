from tools.cp_tools.random_data import Random, forest, undirected_graph, dag, simple_polygon

rng = Random(42)
print(forest(rng, 7, 3).text())
print(undirected_graph(rng, 8, 10, components=2).text(one_based=True))
print(dag(rng, 5, 7).edges)
print(simple_polygon(rng, 6, bound=20))
