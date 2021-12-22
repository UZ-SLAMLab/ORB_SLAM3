import sympy
import sys
import unittest
import sophus


class DualQuaternion:
    """ Dual quaternion class """

    def __init__(self, real_q, inf_q):
        """ Dual quaternion consists of a real quaternion, and an infinitesimal
            quaternion """
        self.real_q = real_q
        self.inf_q = inf_q

    def __mul__(self, right):
        """ dual quaternion multiplication """
        return DualQuaternion(self.real_q * right.real_q,
                              self.real_q * right.inf_q +
                              self.inf_q * right.real_q)

    def __truediv__(self, scalar):
        """ scalar division """
        return DualQuaternion(self.real_q / scalar, self.inf_q / scalar)

    def __repr__(self):
        return "( " + repr(self.real_q) + \
               " + " + repr(self.inf_q) + ")"

    def __getitem__(self, key):
        assert (key >= 0 and key < 8)
        if key < 4:
            return self.real_q[i]
        else:
            return self.inf_q[i - 4]

    def squared_norm(self):
        """ squared norm when considering the dual quaternion as 8-tuple """
        return self.real_q.squared_norm() + self.inf_q.squared_norm()

    def conj(self):
        """ dual quaternion conjugate """
        return DualQuaternion(self.real_q.conj(), self.inf_q.conj())

    def inv(self):
        """ dual quaternion inverse """
        return DualQuaternion(self.real_q.inv(),
                              -self.real_q.inv() * self.inf_q *
                              self.real_q.inv())

    def simplify(self):
        return DualQuaternion(self.real_q.simplify(),
                              self.inf_q.simplify())

    @staticmethod
    def identity():
        return DualQuaternion(sophus.Quaternion.identity(),
                              sophus.Quaternion.zero())

    def __eq__(self, other):
        if isinstance(self, other.__class__):
            return self.real_q == other.real_q and self.inf_q == other.inf_q
        return False


class TestDualQuaternion(unittest.TestCase):
    def setUp(self):
        w, s0, s1, s2 = sympy.symbols('w s0 s1 s2', real=True)
        x, t0, t1, t2 = sympy.symbols('x t0 t1 t2', real=True)
        y, u0, u1, u2 = sympy.symbols('y u0 u1 u2', real=True)
        z, v0, v1, v2 = sympy.symbols('z v0 v1 v2', real=True)

        s = sophus.Vector3(s0, s1, s2)
        t = sophus.Vector3(t0, t1, t2)
        u = sophus.Vector3(u0, u1, u2)
        v = sophus.Vector3(v0, v1, v2)
        self.a = DualQuaternion(sophus.Quaternion(w, s),
                                sophus.Quaternion(x, t))
        self.b = DualQuaternion(sophus.Quaternion(y, u),
                                sophus.Quaternion(z, v))

    def test_muliplications(self):
        product = self.a * self.a.inv()
        self.assertEqual(product.simplify(),
                         DualQuaternion.identity())
        product = self.a.inv() * self.a
        self.assertEqual(product.simplify(),
                         DualQuaternion.identity())


if __name__ == '__main__':
    unittest.main()
