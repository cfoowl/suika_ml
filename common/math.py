from math import sqrt, cos, sin
import random

PI = 3.141592741
EPSILON = 0.0001
RAND_MAX = 32767


class Vec2:
    """
    Vecteur 2D
    """

    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __neg__(self):
        return Vec2(-self.x, -self.y)

    def __add__(self, other):
        if isinstance(other, Vec2):
            return Vec2(self.x + other.x, self.y + other.y)
        elif isinstance(other, (int, float or int)):
            return Vec2(self.x + other, self.y + other)

    def __sub__(self, other):
        if isinstance(other, Vec2):
            return Vec2(self.x - other.x, self.y - other.y)
        elif isinstance(other, (int, float or int)):
            return Vec2(self.x - other, self.y - other)

    def __mul__(self, other):
        if isinstance(other, Vec2):
            return Vec2(self.x * other.x, self.y * other.y)
        elif isinstance(other, (int, float)):
            return Vec2(self.x * other, self.y * other)
        else:
            raise TypeError("Unsupported operand type for *: '{}'".format(type(other)))

    def __rmul__(self, other):
        return self.__mul__(other)

    def __truediv__(self, other):
        if isinstance(other, (float, int)):
            return Vec2(self.x / other, self.y / other)

    def __rtruediv__(self, other):
        return Vec2(other / self.x, other / self.y)

    def Set(self, x_, y_):
        self.x = x_
        self.y = y_

    def LenSqr(self) -> float:
        return self.x**2 + self.y**2

    def Len(self) -> float:
        return sqrt(self.x**2 + self.y**2)

    def Rotate(self, radians) -> None:
        c = cos(radians)
        s = sin(radians)

        xp = self.x * c - self.y * s
        yp = self.x * s + self.y * c

        self.x = xp
        self.y = yp

    def normalize(self) -> None:
        Len = self.Len()
        if Len > EPSILON:
            invLen = 1 / Len
            self.x *= invLen
            self.y *= invLen


class Mat2:
    """
    Matrice 2x2
    """

    def __init__(self, a=None, b=None, c=None, d=None):
        if a != None and b != None and c == None and d == None:
            """
            a : real
            b : radians
            """
            c = cos(b)
            s = sin(b)
            self.m00 = c
            self.m01 = -s
            self.m10 = s
            self.m11 = c
        elif a != None and b != None and c != None and d != None:
            self.m00 = a
            self.m01 = b
            self.m10 = c
            self.m11 = d
        else:
            self.m00 = 0
            self.m01 = 0
            self.m10 = 0
            self.m11 = 0

    def __mul__(self, other):
        if isinstance(other, Vec2):
            return Vec2(
                self.m00 * other.x + self.m01 * other.y,
                self.m10 * other.x + self.m11 * other.y,
            )
        elif isinstance(other, Mat2):
            return Mat2(
                self.m00 * other.m00 + self.m01 * other.m10,
                self.m00 * other.m01 + self.m01 * other.m11,
                self.m10 * other.m00 + self.m11 * other.m10,
                self.m10 * other.m01 + self.m11 * other.m11,
            )
        else:
            raise TypeError("Unsupported operand type for *: '{}'".format(type(other)))

    def Set(self, radians):
        c = cos(radians)
        s = sin(radians)
        self.m00 = c
        self.m01 = -s
        self.m10 = s
        self.m11 = c

    def Abs(self):
        return Mat2(abs(self.m00), abs(self.m01), abs(self.m10), abs(self.m11))

    def AxisX(self) -> Vec2:
        return Vec2(self.m00, self.m10)

    def AxisY(self) -> Vec2:
        return Vec2(self.m01, self.m11)

    def Transpose(self):
        return Mat2(self.m00, self.m10, self.m01, self.m11)


def Min(a: Vec2, b: Vec2) -> Vec2:
    return Vec2(min(a.x, b.x), min(a.y, b.y))


def Max(a: Vec2, b: Vec2) -> Vec2:
    return Vec2(max(a.x, b.x), max(a.y, b.y))


def Dot(a: Vec2, b: Vec2) -> float:
    """
    Calculate the dot product of two vectors
    """
    return a.x * b.x + a.y * b.y


def DistSqr(a: Vec2, b: Vec2) -> float:
    """
    Calculate the square of the distance between two vectors
    """
    c = a - b
    return Dot(c, c)


def Cross(a, b):
    """
    Calculate the cross product of two vectors
    """
    if isinstance(a, Vec2) and isinstance(b, Vec2):
        return a.x * b.y - a.y * b.x
    if isinstance(a, Vec2) and isinstance(b, float or int):
        return Vec2(b * a.y, -b * a.x)
    if isinstance(a, float or int) and isinstance(b, Vec2):
        return Vec2(-a * b.y, a * b.x)


def Equal(a, b) -> bool:
    return abs(a - b) < EPSILON


def Sqr(a):
    return a * a


def Clamp(Min, Max, a):
    if a < Min:
        return Min
    if a > Max:
        return Max
    return a


def Round(a):
    return int(a + 0.5)


def Random(l, h):
    a = random.random()
    a /= RAND_MAX
    a = (h - l) * a + l
    return a


def BiasGreaterThan(a, b):
    kBiasRelative = 0.95
    kBiasAbsolute = 0.01
    return a >= b * kBiasRelative + a * kBiasAbsolute
