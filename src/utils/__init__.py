from typing import List, Tuple


def lerp_over_table(table: List[Tuple[float, ...]], x: float) -> Tuple[float, ...]:
    if x <= table[0][0]:
        return table[0][1:]
    if x >= table[-1][0]:
        return table[-1][1:]

    for i in range(1, len(table)):
        x0, *y0 = table[i - 1]
        x1, *y1 = table[i]
        if x0 <= x <= x1:
            # Calculate the interpolation factor.
            t = (x - x0) / (x1 - x0)
            # Interpolate each corresponding y component.
            return tuple(y0_i + t * (y1_i - y0_i) for y0_i, y1_i in zip(y0, y1))
    return tuple(0 for _ in table[0][1:])


def format_table(table: List[Tuple[float, ...]]) -> List[Tuple[float, ...]]:
    return sorted(
        [tuple(float(value) for value in row) for row in table], key=lambda x: x[0]
    )
