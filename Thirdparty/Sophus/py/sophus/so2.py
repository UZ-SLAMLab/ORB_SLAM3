import sympy
import sys
import unittest
import sophus
import functools


class So2:
    """ 2 dimensional group of orthogonal matrices with determinant 1 """

    def __init__(self, z):
        """ internally represented by a unit complex number z """
        self.z = z

    @staticmethod
    def exp(theta):
        """ exponential map """
        return So2(
            sophus.Complex(
                sympy.cos(theta),
                sympy.sin(theta)))

    def log(self):
        """ logarithmic map"""
        return sympy.atan2(self.z.imag, self.z.real)

    def __repr__(self):
        return "So2:" + repr(self.z)

    @staticmethod
    def hat(theta):
        return sympy.Matrix([[0, -theta],
                             [theta, 0]])

    def matrix(self):
        """ returns matrix representation """
        return sympy.Matrix([
            [self.z.real, -self.z.imag],
            [self.z.imag,  self.z.real]])

    def __mul__(self, right):
        """ left-multiplication
            either rotation concatenation or point-transform """
        if isinstance(right, sympy.Matrix):
            assert right.shape == (2, 1), right.shape
            return self.matrix() * right
        elif isinstance(right, So2):
            return So2(self.z * right.z)
        assert False, "unsupported type: {0}".format(type(right))

    def __getitem__(self, key):
        return self.z[key]

    @staticmethod
    def calc_Dx_exp_x(x):
        return sympy.Matrix(2, 1, lambda r, c:
                            sympy.diff(So2.exp(x)[r], x))

    @staticmethod
    def Dx_exp_x_at_0():
        return sympy.Matrix([0, 1])

    @staticmethod
    def calc_Dx_exp_x_at_0(x):
        return So2.calc_Dx_exp_x(x).limit(x, 0)

    def calc_Dx_this_mul_exp_x_at_0(self, x):
        return sympy.Matrix(2, 1, lambda r, c:
                            sympy.diff((self * So2.exp(x))[r], x))\
            .limit(x, 0)

    @staticmethod
    def Dxi_x_matrix(x, i):
        if i == 0:
            return sympy.Matrix([[1, 0],
                                 [0, 1]])
        if i == 1:
            return sympy.Matrix([[0, -1],
                                 [1, 0]])

    @staticmethod
    def calc_Dxi_x_matrix(x, i):
        return sympy.Matrix(2, 2, lambda r, c:
                            sympy.diff(x.matrix()[r, c], x[i]))

    @staticmethod
    def Dx_exp_x_matrix(x):
        R = So2.exp(x)
        Dx_exp_x = So2.calc_Dx_exp_x(x)
        l = [Dx_exp_x[j] * So2.Dxi_x_matrix(R, j) for j in [0, 1]]
        return functools.reduce((lambda a, b: a + b), l)

    @staticmethod
    def calc_Dx_exp_x_matrix(x):
        return sympy.Matrix(2, 2, lambda r, c:
                            sympy.diff(So2.exp(x).matrix()[r, c], x))

    @staticmethod
    def Dx_exp_x_matrix_at_0():
        return So2.hat(1)

    @staticmethod
    def calc_Dx_exp_x_matrix_at_0(x):
        return sympy.Matrix(2, 2, lambda r, c:
                            sympy.diff(So2.exp(x).matrix()[r, c], x)
                            ).limit(x, 0)


class TestSo2(unittest.TestCase):
    def setUp(self):
        self.theta = sympy.symbols(
            'theta', real=True)
        x, y = sympy.symbols('c[0] c[1]', real=True)
        p0, p1 = sympy.symbols('p0 p1', real=True)
        self.a = So2(sophus.Complex(x, y))
        self.p = sophus.Vector2(p0, p1)

    def test_exp_log(self):
        for theta in [0.,  0.5, 0.1]:
            w = So2.exp(theta).log()
            self.assertAlmostEqual(theta, w)

    def test_matrix(self):
        R_foo_bar = So2.exp(self.theta)
        Rmat_foo_bar = R_foo_bar.matrix()
        point_bar = self.p
        p1_foo = R_foo_bar * point_bar
        p2_foo = Rmat_foo_bar * point_bar
        self.assertEqual(sympy.simplify(p1_foo - p2_foo),
                         sophus.ZeroVector2())

    def test_derivatives(self):
        self.assertEqual(sympy.simplify(So2.calc_Dx_exp_x_at_0(self.theta) -
                                        So2.Dx_exp_x_at_0()),
                         sympy.Matrix.zeros(2, 1))
        for i in [0, 1]:
            self.assertEqual(sympy.simplify(So2.calc_Dxi_x_matrix(self.a, i) -
                                            So2.Dxi_x_matrix(self.a, i)),
                             sympy.Matrix.zeros(2, 2))

        self.assertEqual(sympy.simplify(
            So2.Dx_exp_x_matrix(self.theta) -
            So2.calc_Dx_exp_x_matrix(self.theta)),
            sympy.Matrix.zeros(2, 2))
        self.assertEqual(sympy.simplify(
            So2.Dx_exp_x_matrix_at_0() -
            So2.calc_Dx_exp_x_matrix_at_0(self.theta)),
            sympy.Matrix.zeros(2, 2))

    def test_codegen(self):
        stream = sophus.cse_codegen(So2.calc_Dx_exp_x(self.theta))
        filename = "cpp_gencode/So2_Dx_exp_x.cpp"
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

        stream = sophus.cse_codegen(
            self.a.calc_Dx_this_mul_exp_x_at_0(self.theta))
        filename = "cpp_gencode/So2_Dx_this_mul_exp_x_at_0.cpp"
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
