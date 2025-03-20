from typing import List, Tuple
import time

from wpilib import SmartDashboard


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


def clamp(lower: float, upper: float, x: float) -> float:
    return max(lower, min(upper, x))


def time_f(name: str):
    def wrapper(f):
        def new_f(*args):
            start = time.time()
            out = f(*args)
            took = time.time() - start
            SmartDashboard.putNumber(f"{name} took", took)
            return out

        return new_f

    return wrapper


morse_code = {
    "A": [1, 3],
    "B": [3, 1, 1, 1],
    "C": [3, 1, 3, 1],
    "D": [3, 1, 1],
    "E": [1],
    "F": [1, 1, 3, 1],
    "G": [3, 3, 1],
    "H": [1, 1, 1, 1],
    "I": [1, 1],
    "J": [1, 3, 3, 3],
    "K": [3, 1, 3],
    "L": [1, 3, 1, 1],
    "M": [3, 3],
    "N": [3, 1],
    "O": [3, 3, 3],
    "P": [1, 3, 3, 1],
    "Q": [3, 3, 1, 3],
    "R": [1, 3, 1],
    "S": [1, 1, 1],
    "T": [3],
    "U": [1, 1, 3],
    "V": [1, 1, 1, 3],
    "W": [1, 3, 3],
    "X": [3, 1, 1, 3],
    "Y": [3, 1, 3, 3],
    "Z": [3, 3, 1, 1],
    "0": [3, 3, 3, 3, 3],
    "1": [1, 3, 3, 3, 3],
    "2": [1, 1, 3, 3, 3],
    "3": [1, 1, 1, 3, 3],
    "4": [1, 1, 1, 1, 3],
    "5": [1, 1, 1, 1, 1],
    "6": [3, 1, 1, 1, 1],
    "7": [3, 3, 1, 1, 1],
    "8": [3, 3, 3, 1, 1],
    "9": [3, 3, 3, 3, 1],
    "!": [3, 1, 3, 1, 3, 3],
    " ": [0],
}


def letter_to_morse(letter: str):
    return morse_code[letter.upper() if letter.upper() in morse_code else " "]
