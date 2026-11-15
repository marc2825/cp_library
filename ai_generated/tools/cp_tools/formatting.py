"""Standard-input layouts, independent of generation and RNG consumption."""
from .random_data import Graph

__all__ = ["format_case", "format_cases"]


def _token(value):
    if isinstance(value, bool):
        return str(int(value))
    if isinstance(value, (dict, list, tuple, Graph)) or value is None:
        raise ValueError("expected a scalar token")
    value = str(value)
    if not value or any(c.isspace() for c in value):
        raise ValueError("tokens must be nonempty and contain no whitespace")
    return value


def _line(values):
    tokens = []
    for value in values:
        tokens.extend(_token(x) for x in value) if isinstance(value, (list, tuple)) else tokens.append(_token(value))
    return " ".join(tokens)


def format_case(values, layout):
    """line flattens arrays/tuples once; rows emits records; edges shifts vertices.

    {"literal": "0 0"} permits sentinels; {"lines": "A"} emits one scalar per line.
    Empty rows produce no output; an empty array in line produces a blank line.
    """
    lines = []
    for item in layout:
        if not isinstance(item, dict) or len(set(item) & {"line", "lines", "rows", "edges", "literal"}) != 1:
            raise ValueError("each layout item needs exactly one output operation")
        if "line" in item:
            if set(item) != {"line"} or not isinstance(item["line"], list):
                raise ValueError("line must be a list of variable names")
            lines.append(_line([values[name] for name in item["line"]]))
        elif "literal" in item:
            if set(item) != {"literal"} or not isinstance(item["literal"], str) or "\n" in item["literal"] or "\r" in item["literal"]:
                raise ValueError("literal must be one line")
            lines.append(item["literal"])
        else:
            operation = next(k for k in ("lines", "rows", "edges") if k in item)
            allowed = {operation, "one_based"} if operation == "edges" else {operation}
            if set(item) - allowed or "one_based" in item and type(item["one_based"]) is not bool:
                raise ValueError("invalid layout option")
            rows = values[item[operation]]
            if operation == "edges":
                rows = rows.edges if isinstance(rows, Graph) else rows
                shift = int(item.get("one_based", False))
                for row in rows:
                    if len(row) < 2 or any(type(v) is not int for v in row[:2]):
                        raise ValueError("edges need two integer endpoints")
                    lines.append(_line([row[0] + shift, row[1] + shift, *row[2:]]))
            elif operation == "lines":
                lines.extend(_token(row) for row in rows)
            else:
                lines.extend(_line(row) if isinstance(row, (list, tuple)) else _token(row) for row in rows)
    return "".join(line + "\n" for line in lines)


def format_cases(cases, layout, *, t_prefixed=True):
    return (str(len(cases)) + "\n" if t_prefixed else "") + "".join(format_case(c, layout) for c in cases)
