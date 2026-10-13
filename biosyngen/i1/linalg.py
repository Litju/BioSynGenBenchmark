"""Small dense linear algebra for the I1 planar model (standard library only)."""

from __future__ import annotations


def solve(matrix: list[list[float]], rhs: list[float]) -> list[float]:
    """Solve A x = b by Gaussian elimination with partial pivoting.

    The input lists are not modified. Raises ArithmeticError for a singular or
    numerically singular system (pivot below 1e-14 of the largest row entry).
    """
    n = len(rhs)
    a = [row[:] + [rhs[i]] for i, row in enumerate(matrix)]
    scale = [max(abs(v) for v in row[:n]) or 1.0 for row in a]
    for col in range(n):
        pivot = max(range(col, n), key=lambda r: abs(a[r][col]) / scale[r])
        if abs(a[pivot][col]) <= 1e-14 * scale[pivot]:
            raise ArithmeticError("singular linear system")
        if pivot != col:
            a[col], a[pivot] = a[pivot], a[col]
            scale[col], scale[pivot] = scale[pivot], scale[col]
        pivot_row = a[col]
        inv = 1.0 / pivot_row[col]
        for r in range(col + 1, n):
            row = a[r]
            factor = row[col] * inv
            if factor != 0.0:
                for c in range(col, n + 1):
                    row[c] -= factor * pivot_row[c]
    x = [0.0] * n
    for r in range(n - 1, -1, -1):
        row = a[r]
        total = row[n]
        for c in range(r + 1, n):
            total -= row[c] * x[c]
        x[r] = total / row[r]
    return x
