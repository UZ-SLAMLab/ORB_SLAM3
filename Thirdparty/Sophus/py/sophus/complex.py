import sophus
import sympy
import sys
import unittest


class Complex:
    """ Complex class """

    def __init__(self, real, imag):
        self.real = real
        self.imag = imag

    def __mul__(self, right):
        """ complex multiplication """
        return Complex(self.real * right.real - self.imag * right.imag,
                       self.imag * right.real + self.real * right.imag)

    def __add__(self, right):
        return Complex(elf.real + right.real, self.imag + right.imag)

    def __neg__(self):
        return Complex(-self.real, -self.image)

    def __truediv__(self, scalar):
        """ scalar division """
        return Complex(self.real / scalar, self.imag / scalar)

    def __repr__(self):
        return "( " + repr(self.real) + " + " + repr(self.imag) + "i )"

    def __getitem__(self, key):
        """ We use the following convention [real, imag] """
        if key == 0:
            return self.real
        else:
            return self.imag

    def squared_norm(self):
        """ squared norm when considering the complex number as tuple """
        return self.real**2 + self.imag**2

    def conj(self):
        """ complex conjugate """
        return Complex(self.real, -self.imag)

    def inv(self):
        """ complex inverse """
        return self.conj() / self.squared_norm()

    @staticmethod
    def identity():
        return Complex(1, 0)

    @staticmethod
    def zero():
        return Complex(0, 0)

    def __eq__(self, other):
        if isinstance(self, other.__class__):
            return self.real == other.real and self.imag == other.imag
        return False

    def subs(self, x, y):
        return Complex(self.real.subs(x, y), self.imag.subs(x, y))

    def simplify(self):
        return Complex(sympy.simplify(self.real),
                       sympy.simplify(self.imag))

    @staticmethod
    def Da_a_mul_b(a, b):
        """ derivatice of complex muliplication wrt left multiplier a """
        return sympy.Matrix([[b.real, -b.imag],
                             [b.imag, b.real]])

    @staticmethod
    def Db_a_mul_b(a, b):
        """ derivatice of complex muliplication wrt right multiplicand b """
        return sympy.Matrix([[a.real, -a.imag],
                             [a.imag, a.real]])


class TestComplex(unittest.TestCase):
    def setUp(self):
        x, y = sympy.symbols('x y', real=True)
        u, v = sympy.symbols('u v', real=True)
        self.a = Complex(x, y)
        self.b = Complex(u, v)

    def test_muliplications(self):
        product = self.a * self.a.inv()
        self.assertEqual(product.simplify(),
                         Complex.identity())
        product = self.a.inv() * self.a
        self.assertEqual(product.simplify(),
                         Complex.identity())

    def test_derivatives(self):
        d = sympy.Matrix(2, 2, lambda r, c: sympy.diff(
            (self.a * self.b)[r], self.a[c]))
        self.assertEqual(d,
                         Complex.Da_a_mul_b(self.a, self.b))
        d = sympy.Matrix(2, 2, lambda r, c: sympy.diff(
            (self.a * self.b)[r], self.b[c]))
        self.assertEqual(d,
                         Complex.Db_a_mul_b(self.a, self.b))


if __name__ == '__main__':
    unittest.main()
    print('hello')
