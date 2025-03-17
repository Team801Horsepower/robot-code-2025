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
    "A": [0.05, 0.15],
    "B": [0.15, 0.05, 0.05, 0.05],
    "C": [0.15, 0.05, 0.15, 0.05],
    "D": [0.15, 0.05, 0.05],
    "E": [0.05],
    "F": [0.05, 0.05, 0.15, 0.05],
    "G": [0.15, 0.15, 0.05],
    "H": [0.05, 0.05, 0.05, 0.05],
    "I": [0.05, 0.05],
    "J": [0.05, 0.15, 0.15, 0.15],
    "K": [0.15, 0.05, 0.15],
    "L": [0.05, 0.15, 0.05, 0.05],
    "M": [0.15, 0.15],
    "N": [0.15, 0.05],
    "O": [0.15, 0.15, 0.15],
    "P": [0.05, 0.15, 0.15, 0.05],
    "Q": [0.15, 0.15, 0.05, 0.15],
    "R": [0.05, 0.15, 0.05],
    "S": [0.05, 0.05, 0.05],
    "T": [0.15],
    "U": [0.05, 0.05, 0.15],
    "V": [0.05, 0.05, 0.05, 0.15],
    "W": [0.05, 0.15, 0.15],
    "X": [0.15, 0.05, 0.05, 0.15],
    "Y": [0.15, 0.05, 0.15, 0.15],
    "Z": [0.15, 0.15, 0.05, 0.05],
    "0": [0.15, 0.15, 0.15, 0.15, 0.15],
    "1": [0.05, 0.15, 0.15, 0.15, 0.15],
    "2": [0.05, 0.05, 0.15, 0.15, 0.15],
    "3": [0.05, 0.05, 0.05, 0.15, 0.15],
    "4": [0.05, 0.05, 0.05, 0.05, 0.15],
    "5": [0.05, 0.05, 0.05, 0.05, 0.05],
    "6": [0.15, 0.05, 0.05, 0.05, 0.05],
    "7": [0.15, 0.15, 0.05, 0.05, 0.05],
    "8": [0.15, 0.15, 0.15, 0.05, 0.05],
    "9": [0.15, 0.15, 0.15, 0.15, 0.05],
    "!": [0.15, 0.05, 0.15, 0.05, 0.15, 0.15],
    " ": [],
}


def letter_to_morse(letter: str):
    return morse_code[letter.upper() if letter.upper() in morse_code else " "]
