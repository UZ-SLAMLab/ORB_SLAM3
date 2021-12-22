""" run with: python3 -m sophus.quaternion """

import sophus
import sympy
import sys
import unittest


class Quaternion:
    """ Quaternion class """

    def __init__(self, real, vec):
        """ Quaternion consists of a real scalar, and an imaginary 3-vector """
        assert isinstance(vec, sympy.Matrix)
        assert vec.shape == (3, 1), vec.shape
        self.real = real
        self.vec = vec

    def __mul__(self, right):
        """ quaternion multiplication """
        return Quaternion(self[3] * right[3] - self.vec.dot(right.vec),
                          self[3] * right.vec + right[3] * self.vec +
                          self.vec.cross(right.vec))

    def __add__(self, right):
        """ quaternion multiplication """
        return Quaternion(self[3] + right[3], self.vec + right.vec)

    def __neg__(self):
        return Quaternion(-self[3], -self.vec)

    def __truediv__(self, scalar):
        """ scalar division """
        return Quaternion(self.real / scalar, self.vec / scalar)

    def __repr__(self):
        return "( " + repr(self[3]) + " + " + repr(self.vec) + "i )"

    def __getitem__(self, key):
        """ We use the following convention [vec0, vec1, vec2, real] """
        assert (key >= 0 and key < 4)
        if key == 3:
            return self.real
        else:
            return self.vec[key]

    def squared_norm(self):
        """ squared norm when considering the quaternion as 4-tuple """
        return sophus.squared_norm(self.vec) + self.real**2

    def conj(self):
        """ quaternion conjugate """
        return Quaternion(self.real, -self.vec)

    def inv(self):
        """ quaternion inverse """
        return self.conj() / self.squared_norm()

    @staticmethod
    def identity():
        return Quaternion(1, sophus.Vector3(0, 0, 0))

    @staticmethod
    def zero():
        return Quaternion(0, sophus.Vector3(0, 0, 0))

    def subs(self, x, y):
        return Quaternion(self.real.subs(x, y), self.vec.subs(x, y))

    def simplify(self):
        v = sympy.simplify(self.vec)
        return Quaternion(sympy.simplify(self.real),
                          sophus.Vector3(v[0], v[1], v[2]))

    def __eq__(self, other):
        if isinstance(self, other.__class__):
            return self.real == other.real and self.vec == other.vec
        return False

    @staticmethod
    def Da_a_mul_b(a, b):
        """ derivatice of quaternion muliplication wrt left multiplier a """
        v0 = b.vec[0]
        v1 = b.vec[1]
        v2 = b.vec[2]
        y = b.real
        return sympy.Matrix([[y, v2, -v1, v0],
                             [-v2, y, v0, v1],
                             [v1, -v0, y, v2],
                             [-v0, -v1, -v2, y]])

    @staticmethod
    def Db_a_mul_b(a, b):
        """ derivatice of quaternion muliplication wrt right multiplicand b """
        u0 = a.vec[0]
        u1 = a.vec[1]
        u2 = a.vec[2]
        x = a.real
        return sympy.Matrix([[x, -u2, u1, u0],
                             [u2, x, -u0, u1],
                             [-u1, u0, x, u2],
                             [-u0, -u1, -u2, x]])


class TestQuaternion(unittest.TestCase):
    def setUp(self):
        x, u0, u1, u2 = sympy.symbols('x u0 u1 u2', real=True)
        y, v0, v1, v2 = sympy.symbols('y v0 v1 v2', real=True)
        u = sophus.Vector3(u0, u1, u2)
        v = sophus.Vector3(v0, v1, v2)
        self.a = Quaternion(x, u)
        self.b = Quaternion(y, v)

    def test_muliplications(self):
        product = self.a * self.a.inv()
        self.assertEqual(product.simplify(),
                         Quaternion.identity())
        product = self.a.inv() * self.a
        self.assertEqual(product.simplify(),
                         Quaternion.identity())

    def test_derivatives(self):
        d = sympy.Matrix(4, 4, lambda r, c: sympy.diff(
            (self.a * self.b)[r], self.a[c]))
        self.assertEqual(d,
                         Quaternion.Da_a_mul_b(self.a, self.b))
        d = sympy.Matrix(4, 4, lambda r, c: sympy.diff(
            (self.a * self.b)[r], self.b[c]))
        self.assertEqual(d,
                         Quaternion.Db_a_mul_b(self.a, self.b))


if __name__ == '__main__':
    unittest.main()
    print('hello')
