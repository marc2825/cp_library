"""JSON test specifications, dependency-ordered generation, aggregate budgets, CLI.

    python tools/cplib.py gen --config examples/development/generation/arrays.json
"""
import argparse
import ast
import json
import math
import operator
from pathlib import Path
import re
import sys

from . import generators as g
from . import structures as s
from .formatting import format_case, format_cases
from .random_data import Random, forest, simple_polygon

__all__ = ["expression", "validate_config", "generate_cases", "generate_text", "main"]

_BINARY = {ast.Add: operator.add, ast.Sub: operator.sub, ast.Mult: operator.mul,
           ast.FloorDiv: operator.floordiv, ast.Mod: operator.mod}
_COMPARE = {ast.Lt: operator.lt, ast.LtE: operator.le, ast.Gt: operator.gt,
            ast.GtE: operator.ge, ast.Eq: operator.eq, ast.NotEq: operator.ne}
_CALLS = {"gcd": math.gcd, "abs": abs, "min": min, "max": max, "len": len, "sum": sum}


def _expression_tree(source, names):
    if not isinstance(source, str) or len(source) > 2000:
        raise ValueError("expression must be a string of at most 2000 characters")
    try:
        node = ast.parse(source.removeprefix("="), mode="eval").body
    except (SyntaxError, RecursionError) as exc:
        raise ValueError(f"invalid expression: {source}") from exc

    def check(n):
        if isinstance(n, ast.Constant) and type(n.value) in {int, bool}:
            return
        if isinstance(n, ast.Name) and n.id in names:
            return
        if isinstance(n, ast.BinOp) and type(n.op) in _BINARY:
            check(n.left)
            check(n.right)
            return
        if isinstance(n, ast.UnaryOp) and isinstance(n.op, (ast.USub, ast.UAdd, ast.Not)):
            check(n.operand)
            return
        if isinstance(n, ast.BoolOp) and isinstance(n.op, (ast.And, ast.Or)):
            for value in n.values:
                check(value)
            return
        if isinstance(n, ast.Compare) and all(type(op) in _COMPARE for op in n.ops):
            for value in [n.left, *n.comparators]:
                check(value)
            return
        if isinstance(n, ast.Call) and isinstance(n.func, ast.Name) and n.func.id in _CALLS and not n.keywords:
            for arg in n.args:
                check(arg)
            return
        raise ValueError(f"unsupported expression or unknown/forward reference: {source}")
    try:
        check(node)
    except RecursionError as exc:
        raise ValueError("expression is too deeply nested") from exc
    return node


def expression(source, values):
    """Evaluate a restricted AST. No eval, attributes, subscripts or arbitrary calls."""
    node = _expression_tree(source, values)

    def visit(n):
        if isinstance(n, ast.Constant):
            return n.value
        if isinstance(n, ast.Name):
            return values[n.id]
        if isinstance(n, ast.BinOp):
            return _BINARY[type(n.op)](visit(n.left), visit(n.right))
        if isinstance(n, ast.UnaryOp):
            return {ast.USub: operator.neg, ast.UAdd: operator.pos, ast.Not: operator.not_}[type(n.op)](visit(n.operand))
        if isinstance(n, ast.BoolOp):
            return all(visit(x) for x in n.values) if isinstance(n.op, ast.And) else any(visit(x) for x in n.values)
        if isinstance(n, ast.Compare):
            left = visit(n.left)
            for op, right in zip(n.ops, n.comparators):
                value = visit(right)
                if not _COMPARE[type(op)](left, value):
                    return False
                left = value
            return True
        return _CALLS[n.func.id](*(visit(x) for x in n.args))
    try:
        return visit(node)
    except (ArithmeticError, TypeError, RecursionError) as exc:
        raise ValueError(f"expression failed: {source}: {exc}") from exc


_OPTIONS = {
    "int": {"min", "max", "mode", "multiple", "zero_weight"},
    "float": {"min", "max"}, "fixed": {"min", "max", "digits"},
    "bool": {"true_weight", "false_weight"}, "char": {"alphabet"},
    "choice": {"values", "weights"}, "const": {"value"},
    "int_array": {"length", "min", "max", "mode", "unique", "distinct_count", "total", "sum_min", "sum_max"},
    "tuple": {"ranges", "distinct"},
    "ordered_tuple": {"length", "min", "max", "strict"},
    "interval": {"min", "max", "half_open", "length"},
    "intervals": {"count", "min", "max", "half_open", "length"},
    "string": {"length", "alphabet", "mode", "unique", "counts", "distinct_count"},
    "brackets": {"pairs", "mode"},
    "matrix": {"height", "width", "min", "max", "mode", "ones", "row_sums"},
    "grid": {"height", "width", "walls", "connectivity", "start_goal"},
    "tree": {"n", "mode"}, "forest": {"n", "components"},
    "graph": {"n", "m", "directed", "connected", "components", "loops", "multi", "weight_min", "weight_max"},
    "dag": {"n", "m"}, "bipartite": {"left", "right", "m"},
    "queries": {"length", "n", "kind", "min", "max"},
    "polygon": {"n", "bound"},
}
_NUMERIC = {"min", "max", "length", "count", "pairs", "height", "width", "walls", "n", "m", "components",
            "left", "right", "bound", "total", "sum_min", "sum_max", "ones", "distinct_count", "multiple",
            "zero_weight", "digits", "true_weight", "false_weight", "weight_min", "weight_max"}
_BOOLEAN = {"unique", "distinct", "strict", "half_open", "directed", "connected", "loops", "multi", "start_goal"}
_REQUIRED = {"choice": {"values"}, "const": {"value"}, "tuple": {"ranges"},
             "int_array": {"length"}, "ordered_tuple": {"length"}, "string": {"length"},
             "brackets": {"pairs"}, "matrix": {"height", "width"}, "grid": {"height", "width"},
             "tree": {"n"}, "forest": {"n", "components"}, "graph": {"n", "m"},
             "dag": {"n", "m"}, "bipartite": {"left", "right", "m"},
             "queries": {"length", "n"}, "intervals": {"count"}, "polygon": {"n"}}


def _resolve(value, context):
    if isinstance(value, str) and value.startswith("="):
        return expression(value, context)
    if isinstance(value, list):
        return [_resolve(x, context) for x in value]
    return value


def _options(spec, context):
    result = {}
    for key, value in spec.items():
        if key in {"name", "type"}:
            continue
        if key.endswith("_ref"):
            result[key[:-4]] = expression(value, context)
        else:
            result[key] = _resolve(value, context) if key in _NUMERIC | {"ranges", "row_sums"} else value
    return result


def _generate(rng, kind, opt):
    opt = dict(opt)
    if kind in {"int", "float", "fixed", "int_array", "ordered_tuple", "interval", "intervals", "matrix", "queries"}:
        opt["low"], opt["high"] = opt.pop("min", 0), opt.pop("max", 100)
    if kind in {"int_array", "string"}:
        opt["n"] = opt.pop("length")
    if kind == "ordered_tuple":
        opt["count"] = opt.pop("length")
    if kind == "queries":
        opt["count"] = opt.pop("length")
    if kind == "int_array" and any(k in opt for k in ("total", "sum_min", "sum_max")):
        if set(opt) - {"n", "low", "high", "total", "sum_min", "sum_max"}:
            raise ValueError("sum arrays cannot combine with other array modes/constraints")
        return g.random_array_with_sum(rng, **opt)
    functions = {"int": g.random_int, "float": g.random_float, "fixed": g.random_fixed,
                 "bool": g.random_bool, "char": g.random_char, "int_array": g.random_array,
                 "tuple": g.random_int_tuple, "ordered_tuple": g.random_ordered_tuple,
                 "interval": g.random_interval, "intervals": s.random_intervals,
                 "string": g.random_string, "brackets": g.random_brackets,
                 "matrix": s.random_matrix, "grid": s.random_grid, "tree": s.random_tree,
                 "forest": forest, "graph": s.random_graph, "dag": s.random_dag,
                 "bipartite": s.random_bipartite_graph, "queries": s.random_queries, "polygon": simple_polygon}
    if kind == "const":
        return opt["value"]
    if kind == "choice":
        return rng.weighted_choice(opt["values"], opt["weights"]) if "weights" in opt else rng.choice(opt["values"])
    weights = None
    if kind == "graph" and ("weight_min" in opt or "weight_max" in opt):
        weights = opt.pop("weight_min", 0), opt.pop("weight_max", 100)
    value = functions[kind](rng, **opt)
    return s.weighted_edges(rng, value, *weights) if weights is not None else value


def validate_config(config):
    """Validate schema, references and static constraints without sampling."""
    if not isinstance(config, dict) or set(config) - {"seed", "cases", "variables", "output", "constraints", "attempts"}:
        raise ValueError("invalid configuration keys")
    if type(config.get("seed", 0)) is not int:
        raise ValueError("seed must be an integer")
    if type(config.get("attempts", 100)) is not int or not 1 <= config.get("attempts", 100) <= 100000:
        raise ValueError("attempts must be between 1 and 100000")
    variables = config.get("variables")
    if not isinstance(variables, list):
        raise ValueError("variables must be an array")
    known, constant, specs = set(), {}, {}

    def check_refs(value):
        if isinstance(value, str) and value.startswith("="):
            _expression_tree(value, known)
        elif isinstance(value, list):
            for v in value:
                check_refs(v)

    for spec in variables:
        if not isinstance(spec, dict) or not isinstance(spec.get("name"), str) or not re.fullmatch(r"[A-Za-z_][A-Za-z_0-9]*", spec["name"]):
            raise ValueError("variable needs an identifier name")
        name, kind = spec["name"], spec.get("type")
        if name in known or name in _CALLS:
            raise ValueError(f"duplicate or reserved variable: {name}")
        if not isinstance(kind, str) or kind not in _OPTIONS:
            raise ValueError(f"unsupported variable type: {kind}")
        keys = {k.removesuffix("_ref") for k in spec if k not in {"name", "type"}}
        if keys - _OPTIONS[kind] or not _REQUIRED.get(kind, set()) <= keys:
            raise ValueError(f"unknown or missing fields for {name} ({kind})")
        for key, value in spec.items():
            if key in {"name", "type"}:
                continue
            if key.endswith("_ref"):
                if key[:-4] in spec or key[:-4] not in _NUMERIC:
                    raise ValueError(f"conflicting/unsupported reference: {key}")
                _expression_tree(value, known)
            elif key in _NUMERIC | {"ranges", "row_sums"}:
                check_refs(value)
            if key in _BOOLEAN and type(value) is not bool:
                raise ValueError(f"{key} must be a boolean")
        # Check static integer bounds and propagate fixed scalar values.
        # Dependent/structured specs are checked when instantiated, without
        # materializing potentially huge arrays during schema validation.
        try:
            options = _options(spec, constant)
        except ValueError:
            options = None
        if options is not None:
            if kind == "int":
                low, high = options.get("min", 0), options.get("max", 100)
                g._bounds(low, high)
                if low == high:
                    constant[name] = _generate(Random(0), kind, options)
            elif kind == "const":
                constant[name] = options["value"]
        known.add(name)
        specs[name] = spec
    constraints = config.get("constraints", [])
    if not isinstance(constraints, list):
        raise ValueError("constraints must be an array of expressions")
    for constraint in constraints:
        _expression_tree(constraint, known)
    layout = config.get("output")
    if not isinstance(layout, list):
        raise ValueError("output must be an array")
    for item in layout:
        if not isinstance(item, dict) or len(set(item) & {"line", "rows", "lines", "edges", "literal"}) != 1:
            raise ValueError("invalid output operation")
        operation = next(k for k in ("line", "rows", "lines", "edges", "literal") if k in item)
        if set(item) - {operation, *(["one_based"] if operation == "edges" else [])}:
            raise ValueError("unknown output option")
        if operation == "literal":
            if not isinstance(item[operation], str) or any(c in item[operation] for c in "\r\n"):
                raise ValueError("literal must be one line")
        else:
            names = item[operation] if operation == "line" else [item[operation]]
            if not isinstance(names, list) or any(not isinstance(n, str) or n not in known for n in names):
                raise ValueError("output refers to an unknown variable")
        if "one_based" in item and type(item["one_based"]) is not bool:
            raise ValueError("one_based must be a boolean")
    cases = config.get("cases", {})
    if not isinstance(cases, dict) or set(cases) - {"count", "format", "aggregate"}:
        raise ValueError("invalid cases settings")
    count = cases.get("count", 1)
    if isinstance(count, dict):
        if set(count) != {"min", "max"}:
            raise ValueError("count range needs min and max")
        g._bounds(count["min"], count["max"])
        count_min = count["min"]
    else:
        g._size(count)
        count_min = count
    g._size(count_min)
    if cases.get("format", "t_prefixed") not in {"t_prefixed", "plain"}:
        raise ValueError("cases.format must be t_prefixed or plain")
    aggregate = cases.get("aggregate", {})
    if not isinstance(aggregate, dict):
        raise ValueError("aggregate must be an object")
    for name, budget in aggregate.items():
        spec = specs.get(name, {})
        if spec.get("type") != "int" or set(spec) - {"name", "type", "min", "max"}:
            raise ValueError("aggregate targets must be independent plain integer variables")
        if not isinstance(budget, dict) or set(budget) - {"sum_min", "sum_max", "total", "cost_max"}:
            raise ValueError("unknown aggregate constraint")
        if any(type(v) is not int or v < 0 for v in budget.values()):
            raise ValueError("aggregate budgets must be nonnegative integers")
        low, high = spec.get("min", 0), spec.get("max", 100)
        g._size(low)
        g._bounds(low, high)
        # Check even the minimum count; actual sampled count is checked on allocation.
        if count_min * low > budget.get("sum_max", count_min * high):
            raise ValueError("infeasible aggregate sum_max")
        if "cost_max" in budget and ("total" in budget or "sum_min" in budget):
            raise ValueError("cost_max supports sum_max only")
    return config


def _plan(config, seed, count):
    cases = config.get("cases", {})
    rng = Random(seed)
    if count is None:
        count = cases.get("count", 1)
        if isinstance(count, dict):
            count = rng.integer(count["min"], count["max"])
    g._size(count)
    # Case seeds never share the generation stream with another case or budgets.
    seeds = [rng.next_u64() for _ in range(count)]
    budget_rng = Random(seed ^ 0xD1B54A32D192ED03)
    specs = {v["name"]: v for v in config["variables"]}
    allocated = {}
    for name, budget in sorted(cases.get("aggregate", {}).items()):
        spec = specs[name]
        allocated[name] = g.random_sizes(budget_rng, count, spec.get("min", 0), spec.get("max", 100), **budget)
    return seeds, allocated


def _case(config, seed, fixed):
    rng = Random(seed)
    for _ in range(config.get("attempts", 100)):
        context = {}
        for spec in config["variables"]:
            name = spec["name"]
            try:
                context[name] = fixed[name] if name in fixed else _generate(rng, spec["type"], _options(spec, context))
            except (TypeError, KeyError, ValueError, ArithmeticError) as exc:
                raise ValueError(f"variable {name}: {exc}") from exc
        if all(expression(c, context) for c in config.get("constraints", [])):
            return context
    raise ValueError("constraints unsatisfied within attempt budget; use constructive generators for rare constraints")


def generate_cases(config, *, seed=None, count=None, case_index=None):
    """Return dictionaries; case_index is zero-based and reproduces a batch case."""
    if count is not None:
        g._size(count)
        if not isinstance(config, dict) or not isinstance(config.get("cases", {}), dict):
            raise ValueError("configuration and cases must be objects")
        config = {**config, "cases": {**config.get("cases", {}), "count": count}}
    validate_config(config)
    seed = config.get("seed", 0) if seed is None else seed
    if type(seed) is not int:
        raise ValueError("seed must be an integer")
    seeds, allocated = _plan(config, seed, count)
    if case_index is not None and (type(case_index) is not int or not 0 <= case_index < len(seeds)):
        raise ValueError("case index outside batch")
    indices = range(len(seeds)) if case_index is None else [case_index]
    return [_case(config, seeds[i], {name: values[i] for name, values in allocated.items()}) for i in indices]


def generate_text(config, *, seed=None, count=None, case_index=None):
    cases = generate_cases(config, seed=seed, count=count, case_index=case_index)
    return format_cases(cases, config["output"], t_prefixed=case_index is None and
                        config.get("cases", {}).get("format", "t_prefixed") == "t_prefixed")


def main(argv=None):
    parser = argparse.ArgumentParser(prog="cplib gen", description=__doc__)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--seed", type=int)
    parser.add_argument("--cases", type=int, help="override case count")
    parser.add_argument("--case-index", type=int, help="regenerate one zero-based batch case without T")
    outputs = parser.add_mutually_exclusive_group()
    outputs.add_argument("--out", "-o", type=Path)
    outputs.add_argument("--out-dir", type=Path, help="write case-000000.txt etc., one case per file")
    parser.add_argument("--validate", action="store_true", help="validate and instantiate without emitting input")
    parser.add_argument("--force", action="store_true", help="replace existing generated files")
    args = parser.parse_args(argv)
    try:
        config = json.loads(args.config.read_text(encoding="utf-8"))
        cases = generate_cases(config, seed=args.seed, count=args.cases, case_index=args.case_index)
        texts = [format_case(c, config["output"]) for c in cases]
        if args.validate:
            print(f"valid: {len(cases)} case(s)", file=sys.stderr)
            return 0
        from .contest import write_output
        if args.out_dir is not None:
            indices = range(len(cases)) if args.case_index is None else [args.case_index]
            paths = [args.out_dir / f"case-{i:06d}.txt" for i in indices]
            # Preflight every target before writing any file.
            for path in paths:
                if path.resolve() == args.config.resolve() or path.exists() and not args.force:
                    raise ValueError(f"output already exists or overwrites config: {path}")
            for path, text in zip(paths, texts):
                write_output(path, text, force=args.force, protected=[args.config])
        else:
            prefix = str(len(cases)) + "\n" if args.case_index is None and config.get("cases", {}).get("format", "t_prefixed") == "t_prefixed" else ""
            write_output(args.out, prefix + "".join(texts), force=args.force, protected=[args.config])
        return 0
    except (OSError, ValueError, TypeError, KeyError, OverflowError) as exc:
        parser.exit(2, f"cplib gen: {exc}\n")


if __name__ == "__main__":
    raise SystemExit(main())
