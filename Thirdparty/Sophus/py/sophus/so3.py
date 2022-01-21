import sympy
import sys
import unittest
import sophus
import functools


class So3:
    """ 3 dimensional group of orthogonal matrices with determinant 1 """

    def __init__(self, q):
        """ internally represented by a unit quaternion q """
        self.q = q

    @staticmethod
    def exp(v):
        """ exponential map """
        theta_sq = sophus.squared_norm(v)
        theta = sympy.sqrt(theta_sq)
        return So3(
            sophus.Quaternion(
                sympy.cos(0.5 * theta),
                sympy.sin(0.5 * theta) / theta * v))

    def log(self):
        """ logarithmic map"""
        n = sympy.sqrt(sophus.squared_norm(self.q.vec))
        return 2 * sympy.atan(n / self.q.real) / n * self.q.vec

    def __repr__(self):
        return "So3:" + repr(self.q)

    def inverse(self):
        return So3(self.q.conj())

    @staticmethod
    def hat(o):
        return sympy.Matrix([[0, -o[2], o[1]],
                             [o[2], 0, -o[0]],
                             [-o[1], o[0], 0]])
    
    """vee-operator
   
    It takes the 3x3-matrix representation ``Omega`` and maps it to the
    corresponding vector representation of Lie algebra.
   
    This is the inverse of the hat-operator, see above.
   
    Precondition: ``Omega`` must have the following structure:
   
                   |  0 -c  b |
                   |  c  0 -a |
                   | -b  a  0 | 
    """
    @staticmethod
    def vee(Omega):
        v = sophus.Vector3(Omega.row(2).col(1), Omega.row(0).col(2), Omega.row(1).col(0))
        return v

    def matrix(self):
        """ returns matrix representation """
        return sympy.Matrix([[
            1 - 2 * self.q.vec[1]**2 - 2 * self.q.vec[2]**2,
            2 * self.q.vec[0] * self.q.vec[1] -
            2 * self.q.vec[2] * self.q[3],
            2 * self.q.vec[0] * self.q.vec[2] +
            2 * self.q.vec[1] * self.q[3]
        ], [
            2 * self.q.vec[0] * self.q.vec[1] +
            2 * self.q.vec[2] * self.q[3],
            1 - 2 * self.q.vec[0]**2 - 2 * self.q.vec[2]**2,
            2 * self.q.vec[1] * self.q.vec[2] -
            2 * self.q.vec[0] * self.q[3]
        ], [
            2 * self.q.vec[0] * self.q.vec[2] -
            2 * self.q.vec[1] * self.q[3],
            2 * self.q.vec[1] * self.q.vec[2] +
            2 * self.q.vec[0] * self.q[3],
            1 - 2 * self.q.vec[0]**2 - 2 * self.q.vec[1]**2
        ]])

    def __mul__(self, right):
        """ left-multiplication
            either rotation concatenation or point-transform """
        if isinstance(right, sympy.Matrix):
            assert right.shape == (3, 1), right.shape
            return (self.q * sophus.Quaternion(0, right) * self.q.conj()).vec
        elif isinstance(right, So3):
            return So3(self.q * right.q)
        assert False, "unsupported type: {0}".format(type(right))

    def __getitem__(self, key):
        return self.q[key]

    @staticmethod
    def calc_Dx_exp_x(x):
        return sympy.Matrix(4, 3, lambda r, c:
                            sympy.diff(So3.exp(x)[r], x[c]))

    @staticmethod
    def Dx_exp_x_at_0():
        return sympy.Matrix([[0.5, 0.0, 0.0],
                             [0.0, 0.5, 0.0],
                             [0.0, 0.0, 0.5],
                             [0.0, 0.0, 0.0]])

    @staticmethod
    def calc_Dx_exp_x_at_0(x):
        return So3.calc_Dx_exp_x(x).subs(x[0], 0).subs(x[1], 0).limit(x[2], 0)

    def calc_Dx_this_mul_exp_x_at_0(self, x):
        return sympy.Matrix(4, 3, lambda r, c:
                            sympy.diff((self * So3.exp(x))[r], x[c]))\
            .subs(x[0], 0).subs(x[1], 0).limit(x[2], 0)

    def calc_Dx_exp_x_mul_this_at_0(self, x):
        return sympy.Matrix(3, 4, lambda r, c:
                            sympy.diff((self * So3.exp(x))[c], x[r, 0]))\
            .subs(x[0], 0).subs(x[1], 0).limit(x[2], 0)

    @staticmethod
    def Dxi_x_matrix(x, i):
        if i == 0:
            return sympy.Matrix([[0, 2 * x[1], 2 * x[2]],
                                 [2 * x[1], -4 * x[0], -2 * x[3]],
                                 [2 * x[2], 2 * x[3], -4 * x[0]]])
        if i == 1:
            return sympy.Matrix([[-4 * x[1], 2 * x[0], 2 * x[3]],
                                 [2 * x[0], 0, 2 * x[2]],
                                 [-2 * x[3], 2 * x[2], -4 * x[1]]])
        if i == 2:
            return sympy.Matrix([[-4 * x[2], -2 * x[3], 2 * x[0]],
                                 [2 * x[3], -4 * x[2], 2 * x[1]],
                                 [2 * x[0], 2 * x[1], 0]])
        if i == 3:
            return sympy.Matrix([[0, -2 * x[2], 2 * x[1]],
                                 [2 * x[2], 0, -2 * x[0]],
                                 [-2 * x[1], 2 * x[0], 0]])

    @staticmethod
    def calc_Dxi_x_matrix(x, i):
        return sympy.Matrix(3, 3, lambda r, c:
                            sympy.diff(x.matrix()[r, c], x[i]))

    @staticmethod
    def Dxi_exp_x_matrix(x, i):
        R = So3.exp(x)
        Dx_exp_x = So3.calc_Dx_exp_x(x)
        l = [Dx_exp_x[j, i] * So3.Dxi_x_matrix(R, j) for j in [0, 1, 2, 3]]
        return functools.reduce((lambda a, b: a + b), l)

    @staticmethod
    def calc_Dxi_exp_x_matrix(x, i):
        return sympy.Matrix(3, 3, lambda r, c:
                            sympy.diff(So3.exp(x).matrix()[r, c], x[i]))

    @staticmethod
    def Dxi_exp_x_matrix_at_0(i):
        v = sophus.ZeroVector3()
        v[i] = 1
        return So3.hat(v)

    @staticmethod
    def calc_Dxi_exp_x_matrix_at_0(x, i):
        return sympy.Matrix(3, 3, lambda r, c:
                            sympy.diff(So3.exp(x).matrix()[r, c], x[i])
                            ).subs(x[0], 0).subs(x[1], 0).limit(x[2], 0)


class TestSo3(unittest.TestCase):
    def setUp(self):
        omega0, omega1, omega2 = sympy.symbols(
            'omega[0], omega[1], omega[2]', real=True)
        x, v0, v1, v2 = sympy.symbols('q.w() q.x() q.y() q.z()', real=True)
        p0, p1, p2 = sympy.symbols('p0 p1 p2', real=True)
        v = sophus.Vector3(v0, v1, v2)
        self.omega = sophus.Vector3(omega0, omega1, omega2)
        self.a = So3(sophus.Quaternion(x, v))
        self.p = sophus.Vector3(p0, p1, p2)

    def test_exp_log(self):
        for o in [sophus.Vector3(0., 1, 0.5),
                  sophus.Vector3(0.1, 0.1, 0.1),
                  sophus.Vector3(0.01, 0.2, 0.03)]:
            w = So3.exp(o).log()
            for i in range(0, 3):
                self.assertAlmostEqual(o[i], w[i])

    def test_matrix(self):
        R_foo_bar = So3.exp(self.omega)
        Rmat_foo_bar = R_foo_bar.matrix()
        point_bar = self.p
        p1_foo = R_foo_bar * point_bar
        p2_foo = Rmat_foo_bar * point_bar
        self.assertEqual(sympy.simplify(p1_foo - p2_foo),
                         sophus.ZeroVector3())

    def test_derivatives(self):
        self.assertEqual(sympy.simplify(So3.calc_Dx_exp_x_at_0(self.omega) -
                                        So3.Dx_exp_x_at_0()),
                         sympy.Matrix.zeros(4, 3))

        for i in [0, 1, 2, 3]:
            self.assertEqual(sympy.simplify(So3.calc_Dxi_x_matrix(self.a, i) -
                                            So3.Dxi_x_matrix(self.a, i)),
                             sympy.Matrix.zeros(3, 3))
        for i in [0, 1, 2]:
            self.assertEqual(sympy.simplify(
                So3.Dxi_exp_x_matrix(self.omega, i) -
                So3.calc_Dxi_exp_x_matrix(self.omega, i)),
                sympy.Matrix.zeros(3, 3))
            self.assertEqual(sympy.simplify(
                So3.Dxi_exp_x_matrix_at_0(i) -
                So3.calc_Dxi_exp_x_matrix_at_0(self.omega, i)),
                sympy.Matrix.zeros(3, 3))

    def test_codegen(self):
        stream = sophus.cse_codegen(So3.calc_Dx_exp_x(self.omega))
        filename = "cpp_gencode/So3_Dx_exp_x.cpp"
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
            self.a.calc_Dx_this_mul_exp_x_at_0(self.omega))
        filename = "cpp_gencode/So3_Dx_this_mul_exp_x_at_0.cpp"
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
