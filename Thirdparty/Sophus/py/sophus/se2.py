import sympy
import sys
import unittest
import sophus
import functools


class Se2:
    """ 2 dimensional group of rigid body transformations """

    def __init__(self, so2, t):
        """ internally represented by a unit complex number z and a translation
            2-vector """
        self.so2 = so2
        self.t = t

    @staticmethod
    def exp(v):
        """ exponential map """
        theta = v[2]
        so2 = sophus.So2.exp(theta)

        a = so2.z.imag / theta
        b = (1 - so2.z.real) / theta

        t = sophus.Vector2(a * v[0] - b * v[1],
                           b * v[0] + a * v[1])
        return Se2(so2, t)

    def log(self):
        theta = self.so2.log()
        halftheta = 0.5 * theta
        a = -(halftheta * self.so2.z.imag) / (self.so2.z.real - 1)

        V_inv = sympy.Matrix([[a, halftheta],
                              [-halftheta, a]])
        upsilon = V_inv * self.t
        return sophus.Vector3(upsilon[0], upsilon[1], theta)

    def __repr__(self):
        return "Se2: [" + repr(self.so2) + " " + repr(self.t)

    @staticmethod
    def hat(v):
        upsilon = sophus.Vector2(v[0], v[1])
        theta = v[2]
        return sophus.So2.hat(theta).\
            row_join(upsilon).\
            col_join(sympy.Matrix.zeros(1, 3))

    def matrix(self):
        """ returns matrix representation """
        R = self.so2.matrix()
        return (R.row_join(self.t)).col_join(sympy.Matrix(1, 3, [0, 0, 1]))

    def __mul__(self, right):
        """ left-multiplication
            either rotation concatenation or point-transform """
        if isinstance(right, sympy.Matrix):
            assert right.shape == (2, 1), right.shape
            return self.so2 * right + self.t
        elif isinstance(right, Se2):
            return Se2(self.so2 * right.so2,
                       self.t + self.so2 * right.t)
        assert False, "unsupported type: {0}".format(type(right))

    def __getitem__(self, key):
        """ We use the following convention [q0, q1, q2, q3, t0, t1, t2] """
        assert (key >= 0 and key < 4)
        if key < 2:
            return self.so2[key]
        else:
            return self.t[key - 2]

    @staticmethod
    def calc_Dx_exp_x(x):
        return sympy.Matrix(4, 3, lambda r, c:
                            sympy.diff(Se2.exp(x)[r], x[c]))

    @staticmethod
    def Dx_exp_x_at_0():
        return sympy.Matrix([[0, 0, 0],
                             [0, 0, 1],
                             [1, 0, 0],
                             [0, 1, 0]])

    def calc_Dx_this_mul_exp_x_at_0(self, x):
        v = Se2.exp(x)
        return sympy.Matrix(4, 3, lambda r, c:
                            sympy.diff((self * Se2.exp(x))[r], x[c])). \
            subs(x[0], 0).subs(x[1], 0).limit(x[2], 0)

    @staticmethod
    def calc_Dx_exp_x_at_0(x):
        return Se2.calc_Dx_exp_x(x).subs(x[0], 0).subs(x[1], 0).limit(x[2], 0)

    @staticmethod
    def Dxi_x_matrix(x, i):
        if i < 2:
            return sophus.So2.Dxi_x_matrix(x, i).\
                row_join(sympy.Matrix.zeros(2, 1)).\
                col_join(sympy.Matrix.zeros(1, 3))
        M = sympy.Matrix.zeros(3, 3)
        M[i - 2, 2] = 1
        return M

    @staticmethod
    def calc_Dxi_x_matrix(x, i):
        return sympy.Matrix(3, 3, lambda r, c:
                            sympy.diff(x.matrix()[r, c], x[i]))

    @staticmethod
    def Dxi_exp_x_matrix(x, i):
        T = Se2.exp(x)
        Dx_exp_x = Se2.calc_Dx_exp_x(x)
        l = [Dx_exp_x[j, i] * Se2.Dxi_x_matrix(T, j) for j in range(0, 4)]
        return functools.reduce((lambda a, b: a + b), l)

    @staticmethod
    def calc_Dxi_exp_x_matrix(x, i):
        return sympy.Matrix(3, 3, lambda r, c:
                            sympy.diff(Se2.exp(x).matrix()[r, c], x[i]))

    @staticmethod
    def Dxi_exp_x_matrix_at_0(i):
        v = sophus.ZeroVector3()
        v[i] = 1
        return Se2.hat(v)

    @staticmethod
    def calc_Dxi_exp_x_matrix_at_0(x, i):
        return sympy.Matrix(3, 3, lambda r, c:
                            sympy.diff(Se2.exp(x).matrix()[r, c], x[i])
                            ).subs(x[0], 0).subs(x[1], 0).limit(x[2], 0)


class TestSe2(unittest.TestCase):
    def setUp(self):
        upsilon0, upsilon1, theta = sympy.symbols(
            'upsilon[0], upsilon[1], theta',
            real=True)
        x, y = sympy.symbols('c[0] c[1]', real=True)
        p0, p1 = sympy.symbols('p0 p1', real=True)
        t0, t1 = sympy.symbols('t[0] t[1]', real=True)
        self.upsilon_theta = sophus.Vector3(
            upsilon0, upsilon1, theta)
        self.t = sophus.Vector2(t0, t1)
        self.a = Se2(sophus.So2(sophus.Complex(x, y)), self.t)
        self.p = sophus.Vector2(p0, p1)

    def test_exp_log(self):
        for v in [sophus.Vector3(0., 1, 0.5),
                  sophus.Vector3(0.1, 0.1, 0.1),
                  sophus.Vector3(0.01, 0.2, 0.03)]:
            w = Se2.exp(v).log()
            for i in range(0, 3):
                self.assertAlmostEqual(v[i], w[i])

    def test_matrix(self):
        T_foo_bar = Se2.exp(self.upsilon_theta)
        Tmat_foo_bar = T_foo_bar.matrix()
        point_bar = self.p
        p1_foo = T_foo_bar * point_bar
        p2_foo = sophus.proj(Tmat_foo_bar * sophus.unproj(point_bar))
        self.assertEqual(sympy.simplify(p1_foo - p2_foo),
                         sophus.ZeroVector2())

    def test_derivatives(self):
        self.assertEqual(sympy.simplify(
            Se2.calc_Dx_exp_x_at_0(self.upsilon_theta) -
            Se2.Dx_exp_x_at_0()),
            sympy.Matrix.zeros(4, 3))
        for i in range(0, 4):
            self.assertEqual(sympy.simplify(Se2.calc_Dxi_x_matrix(self.a, i) -
                                            Se2.Dxi_x_matrix(self.a, i)),
                             sympy.Matrix.zeros(3, 3))
        for i in range(0, 3):
            self.assertEqual(sympy.simplify(
                Se2.Dxi_exp_x_matrix(self.upsilon_theta, i) -
                Se2.calc_Dxi_exp_x_matrix(self.upsilon_theta, i)),
                sympy.Matrix.zeros(3, 3))
            self.assertEqual(sympy.simplify(
                Se2.Dxi_exp_x_matrix_at_0(i) -
                Se2.calc_Dxi_exp_x_matrix_at_0(self.upsilon_theta, i)),
                sympy.Matrix.zeros(3, 3))

    def test_codegen(self):
        stream = sophus.cse_codegen(Se2.calc_Dx_exp_x(self.upsilon_theta))
        filename = "cpp_gencode/Se2_Dx_exp_x.cpp"

        # set to true to generate codegen files
        if False:
            file = open(filename, "w")
            for line in stream:
                file.write(line)
            file.close()
        else:
            file = open(filename, "r")
            file_lines = file.readlines()
            for i, line in enumerate(stream):
                self.assertEqual(line, file_lines[i])
            file.close()
        stream.close

        stream = sophus.cse_codegen(self.a.calc_Dx_this_mul_exp_x_at_0(
            self.upsilon_theta))
        filename = "cpp_gencode/Se2_Dx_this_mul_exp_x_at_0.cpp"
        # set to true to generate codegen files
        if False:
            file = open(filename, "w")
            for line in stream:
                file.write(line)
            file.close()
        else:
            file = open(filename, "r")
            file_lines = file.readlines()
            for i, line in enumerate(stream):
                self.assertEqual(line, file_lines[i])
            file.close()
        stream.close


if __name__ == '__main__':
    unittest.main()
