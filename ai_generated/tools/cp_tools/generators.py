"""Constructive test data generators. All integer bounds are inclusive.

Uses the existing SplitMix64 engine. Structured distributions are deliberately
not uniform over the set of all feasible objects; see docs/development/generation.md.
"""
from decimal import Decimal
import math
import string

from .random_data import Random

__all__ = ["Random", "sample_indices", "random_int", "random_float", "random_fixed",
           "random_bool", "random_char", "random_int_tuple", "random_ordered_tuple",
           "random_interval", "random_records", "random_array", "random_array_with_sum",
           "random_sizes", "random_string", "random_anagram_pair", "random_brackets"]


def _size(n):
    if type(n) is not int or n < 0:
        raise ValueError("size must be a nonnegative integer")


def _bounds(low, high):
    if type(low) is not int or type(high) is not int or low > high:
        raise ValueError("need integer low <= high")


def sample_indices(rng, population, count):
    """Ordered sample from range(population), O(count) time/memory, no enumeration."""
    _size(population)
    _size(count)
    if count > population:
        raise ValueError("sample exceeds population")
    # Lazy partial Fisher-Yates; a dict stores only touched positions.
    swaps, result = {}, []
    for i in range(count):
        j = rng.integer(i, population - 1)
        result.append(swaps.get(j, j))
        swaps[j] = swaps.get(i, i)
    return result


def random_int(rng, low, high, *, mode="uniform", multiple=1, zero_weight=3):
    _bounds(low, high)
    if type(multiple) is not int or multiple <= 0:
        raise ValueError("multiple must be positive")
    if type(zero_weight) is not int or zero_weight < 0:
        raise ValueError("zero_weight must be nonnegative")
    lo, hi = -(-low // multiple), high // multiple
    if lo > hi:
        raise ValueError("range contains no multiple")
    if isinstance(mode, (list, tuple)):
        if not mode:
            raise ValueError("empty mode mixture")
        mode = rng.choice(mode)
    if mode not in {"uniform", "random", "minimum", "maximum", "boundary", "small", "large", "zero_heavy", "power2", "power10"}:
        raise ValueError(f"unknown integer mode: {mode}")
    if mode in {"uniform", "random"}:
        value = rng.integer(lo, hi)
    elif mode == "minimum":
        value = lo
    elif mode == "maximum":
        value = hi
    elif mode == "boundary":
        value = rng.choice((lo, min(lo + 1, hi), max(lo, hi - 1), hi))
    elif mode == "small":
        value = rng.integer(lo, min(hi, lo + 10))
    elif mode == "large":
        value = rng.integer(max(lo, hi - 10), hi)
    elif mode == "zero_heavy":
        value = 0 if lo <= 0 <= hi and rng.integer(0, zero_weight) else rng.integer(lo, hi)
    elif mode in {"power2", "power10"}:
        base, power, candidates = (2 if mode == "power2" else 10), 1, {lo, hi}
        while power <= max(abs(low), abs(high)) + 1:
            for sign in (-1, 1):
                for delta in (-1, 0, 1):
                    x = sign * power + delta
                    if low <= x <= high and x % multiple == 0:
                        candidates.add(x // multiple)
            power *= base
        value = rng.choice(sorted(candidates))
    else:
        raise ValueError(f"unknown integer mode: {mode}")
    return value * multiple


def random_float(rng, low=0.0, high=1.0):
    """53 random mantissa bits; finite inclusive bounds (rounding may reach high)."""
    if not math.isfinite(low) or not math.isfinite(high) or low > high:
        raise ValueError("need finite low <= high")
    ratio = (rng.next_u64() >> 11) / (1 << 53)
    return min(high, max(low, low * (1 - ratio) + high * ratio))


def random_fixed(rng, low, high, digits=2):
    """Decimal with exactly digits places, sampled on the decimal lattice."""
    _size(digits)
    low, high = Decimal(str(low)), Decimal(str(high))
    if not low.is_finite() or not high.is_finite() or low > high:
        raise ValueError("need finite low <= high")
    # Integer ratios avoid Decimal context rounding even for huge integers.
    scale = 10 ** digits
    ln, ld = low.as_integer_ratio()
    hn, hd = high.as_integer_ratio()
    lo, hi = -(-(ln * scale) // ld), hn * scale // hd
    _bounds(lo, hi)
    value = rng.integer(lo, hi)
    sign = "-" if value < 0 else ""
    whole, fraction = divmod(abs(value), scale)
    return Decimal(f"{sign}{whole}" + (f".{fraction:0{digits}d}" if digits else ""))


def random_bool(rng, true_weight=1, false_weight=1):
    return rng.weighted_choice((True, False), (true_weight, false_weight))


def random_char(rng, alphabet=string.ascii_lowercase):
    if not isinstance(alphabet, str) or not alphabet:
        raise ValueError("alphabet must be a nonempty string")
    return rng.choice(alphabet)


def random_int_tuple(rng, ranges, *, distinct=False, predicate=None, attempts=1000):
    """Independent ranges with optional bounded rejection for custom predicates.

    Exhaustion is an error, not a proof of infeasibility. Prefer constructive
    ordered tuples, intervals and sums for rare constraints.
    """
    ranges = list(ranges)
    for low, high in ranges:
        _bounds(low, high)
    if type(attempts) is not int or attempts < 1:
        raise ValueError("attempts must be positive")
    if distinct and ranges and len(set(ranges)) == 1:
        low, high = ranges[0]
        value = tuple(low + i for i in sample_indices(rng, high - low + 1, len(ranges)))
        if predicate is None or predicate(value):
            return value
    for _ in range(attempts):
        value = tuple(rng.integer(lo, hi) for lo, hi in ranges)
        if (not distinct or len(set(value)) == len(value)) and (predicate is None or predicate(value)):
            return value
    raise ValueError("tuple constraints unsatisfied within attempt budget")


def random_ordered_tuple(rng, count, low, high, *, strict=True):
    _size(count)
    _bounds(low, high)
    if strict:
        return tuple(sorted(low + i for i in sample_indices(rng, high - low + 1, count)))
    return tuple(sorted(rng.integer(low, high) for _ in range(count)))


def random_interval(rng, low, high, *, half_open=False, length=None):
    """Closed [l,r] or nonempty [l,r); both endpoints stay in [low,high]."""
    _bounds(low, high)
    if length is not None:
        if type(length) is not int or length < 1:
            raise ValueError("length must be positive")
        span = length if half_open else length - 1
        if span > high - low:
            raise ValueError("interval does not fit")
        start = rng.integer(low, high - span)
        return start, start + span
    return random_ordered_tuple(rng, 2, low, high, strict=half_open)


def random_records(rng, count, fields):
    """fields: callables taking rng; supports tuples with mixed types."""
    _size(count)
    fields = tuple(fields)
    return [tuple(field(rng) for field in fields) for _ in range(count)]


def random_array(rng, n, low, high, *, unique=False, mode="uniform", distinct_count=None):
    _size(n)
    _bounds(low, high)
    if isinstance(mode, (list, tuple)):
        if not mode:
            raise ValueError("empty mode mixture")
        mode = rng.choice(mode)
    if mode not in {"uniform", "random", "unique", "sorted", "reversed", "almost_sorted", "all_equal", "alternating", "few_distinct",
                    "minimum", "maximum", "boundary", "small", "large", "zero_heavy", "power2", "power10"}:
        raise ValueError(f"unknown array mode: {mode}")
    if mode == "unique":
        unique = True
    if distinct_count is not None:
        if mode not in {"uniform", "random", "unique", "sorted", "reversed", "almost_sorted", "all_equal", "alternating", "few_distinct"}:
            raise ValueError("distinct_count cannot combine with scalar distribution modes")
        if type(distinct_count) is not int or not 0 <= distinct_count <= min(n, high - low + 1):
            raise ValueError("infeasible distinct count")
        if (n > 0 and distinct_count == 0) or (unique and distinct_count != n):
            raise ValueError("distinct count conflicts with size/unique")
    if mode in {"all_equal", "alternating", "few_distinct"} or distinct_count is not None:
        k = distinct_count if distinct_count is not None else min(n, high - low + 1,
                1 if mode == "all_equal" else 2 if mode == "alternating" else 3)
        if mode == "all_equal" and k > 1 or mode == "alternating" and k > 2 or unique and k != n:
            raise ValueError("mode conflicts with distinct/unique constraint")
        alphabet = [low + i for i in sample_indices(rng, high - low + 1, k)]
        if not n:
            return []
        values = [alphabet[i % k] for i in range(n)] if mode in {"alternating", "all_equal"} else (
            alphabet + [rng.choice(alphabet) for _ in range(n - k)])
        if mode not in {"alternating", "all_equal"}:
            rng.shuffle(values)
    elif unique:
        if mode not in {"uniform", "random", "unique", "sorted", "reversed", "almost_sorted"}:
            raise ValueError("unique supports uniform or ordering modes")
        values = [low + i for i in sample_indices(rng, high - low + 1, n)]
    else:
        scalar_mode = "uniform" if mode in {"sorted", "reversed", "almost_sorted"} else mode
        values = [random_int(rng, low, high, mode=scalar_mode) for _ in range(n)]
    if mode in {"sorted", "reversed", "almost_sorted"}:
        values.sort(reverse=mode == "reversed")
    if mode == "almost_sorted" and n > 1:
        for _ in range(max(1, n // 100)):
            i, j = rng.integer(0, n - 1), rng.integer(0, n - 1)
            values[i], values[j] = values[j], values[i]
    return values


def random_array_with_sum(rng, n, low, high, total=None, *, sum_min=None, sum_max=None):
    """O(n) constructive bounded composition; not a uniform composition sampler."""
    _size(n)
    _bounds(low, high)
    if any(value is not None and type(value) is not int for value in (total, sum_min, sum_max)):
        raise ValueError("sum bounds must be integers")
    lower = max(n * low, sum_min if sum_min is not None else n * low)
    upper = min(n * high, sum_max if sum_max is not None else n * high)
    if lower > upper or total is not None and not lower <= total <= upper:
        raise ValueError("infeasible sum constraints")
    if any(type(x) is not int for x in (lower, upper)) or total is not None and type(total) is not int:
        raise ValueError("sum bounds must be integers")
    remaining = rng.integer(lower, upper) if total is None else total
    values = []
    for i in range(n):
        count = n - i - 1
        x = rng.integer(max(low, remaining - count * high), min(high, remaining - count * low))
        values.append(x)
        remaining -= x
    rng.shuffle(values)
    return values


def random_sizes(rng, count, low, high, *, sum_max=None, sum_min=None, total=None, cost_max=None):
    """Allocate nonnegative case sizes, optionally sum(size**2) <= cost_max."""
    _size(low)
    _size(count)
    _bounds(low, high)
    for value in (sum_max, sum_min, total):
        if value is not None and type(value) is not int:
            raise ValueError("aggregate budgets must be integers")
    if cost_max is None:
        return random_array_with_sum(rng, count, low, high, total, sum_min=sum_min, sum_max=sum_max)
    _size(cost_max)
    if total is not None or sum_min is not None:
        raise ValueError("square cost supports sum_max only")
    if count * low * low > cost_max or sum_max is not None and count * low > sum_max:
        raise ValueError("infeasible aggregate budget")
    remaining_sum = count * high if sum_max is None else sum_max
    values = []
    for i in range(count):
        rest = count - i - 1
        cap = min(high, remaining_sum - rest * low, math.isqrt(cost_max - rest * low * low))
        x = rng.integer(low, cap)
        values.append(x)
        cost_max -= x * x
        remaining_sum -= x
    rng.shuffle(values)
    return values


def random_string(rng, n, alphabet=string.ascii_lowercase, *, mode="uniform", unique=False,
                  counts=None, distinct_count=None):
    _size(n)
    if not isinstance(alphabet, str) or not alphabet:
        raise ValueError("alphabet must be nonempty")
    alphabet = "".join(dict.fromkeys(alphabet))
    if not isinstance(mode, str):
        raise ValueError("string mode must be a single mode name")
    if counts is not None:
        if mode != "uniform" or unique or distinct_count is not None:
            raise ValueError("counts cannot be combined with mode/unique/distinct_count")
        if not isinstance(counts, dict) or any(c not in alphabet or len(c) != 1 or type(k) is not int or k < 0
                                             for c, k in counts.items()) or sum(counts.values()) != n:
            raise ValueError("invalid character counts")
        chars = [c for c in sorted(counts) for _ in range(counts[c])]
        rng.shuffle(chars)
        return "".join(chars)
    if mode in {"palindrome", "non_palindrome", "no_adjacent", "periodic"}:
        if unique or distinct_count is not None:
            raise ValueError("structured string modes do not combine with unique/distinct_count")
        if mode == "palindrome":
            half = random_string(rng, (n + 1) // 2, alphabet)
            return half + half[:n // 2][::-1]
        if mode == "non_palindrome":
            if n < 2 or len(alphabet) < 2:
                raise ValueError("non-palindrome needs length >= 2 and two characters")
            middle = random_string(rng, n - 2, alphabet)
            a, b = rng.sample(alphabet, 2)
            return a + middle + b
        if mode == "periodic":
            period = random_string(rng, min(n, 3), alphabet)
            return (period * ((n + len(period) - 1) // len(period)))[:n] if n else ""
        if n > 1 and len(alphabet) < 2:
            raise ValueError("no-adjacent needs at least two characters")
        chars, last = [], -1
        for _ in range(n):
            i = rng.integer(0, len(alphabet) - 1 if last < 0 else len(alphabet) - 2)
            i += last >= 0 and i >= last
            chars.append(alphabet[i])
            last = i
        return "".join(chars)
    return "".join(alphabet[i] for i in random_array(rng, n, 0, len(alphabet) - 1,
                    mode=mode, unique=unique, distinct_count=distinct_count))


def random_anagram_pair(rng, n, alphabet=string.ascii_lowercase):
    s = random_string(rng, n, alphabet)
    chars = list(s)
    rng.shuffle(chars)
    return s, "".join(chars)


def random_brackets(rng, pairs, *, mode="random"):
    """Balanced parentheses, random constructive walk or nested/alternating."""
    _size(pairs)
    if mode == "nested":
        return "(" * pairs + ")" * pairs
    if mode == "alternating":
        return "()" * pairs
    if mode != "random":
        raise ValueError("unknown bracket mode")
    opened, balance, chars = 0, 0, []
    for _ in range(2 * pairs):
        if opened < pairs and (balance == 0 or rng.below(2)):
            chars.append("(")
            opened += 1
            balance += 1
        else:
            chars.append(")")
            balance -= 1
    return "".join(chars)
