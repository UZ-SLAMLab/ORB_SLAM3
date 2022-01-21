import sympy
import sys
import unittest
import sophus
import functools


class Se3:
    """ 3 dimensional group of rigid body transformations """

    def __init__(self, so3, t):
        """ internally represented by a unit quaternion q and a translation
            3-vector """
        assert isinstance(so3, sophus.So3)
        assert isinstance(t, sympy.Matrix)
        assert t.shape == (3, 1), t.shape

        self.so3 = so3
        self.t = t

    @staticmethod
    def exp(v):
        """ exponential map """
        upsilon = v[0:3, :]
        omega = sophus.Vector3(v[3], v[4], v[5])
        so3 = sophus.So3.exp(omega)
        Omega = sophus.So3.hat(omega)
        Omega_sq = Omega * Omega
        theta = sympy.sqrt(sophus.squared_norm(omega))
        V = (sympy.Matrix.eye(3) +
             (1 - sympy.cos(theta)) / (theta**2) * Omega +
             (theta - sympy.sin(theta)) / (theta**3) * Omega_sq)
        return Se3(so3, V * upsilon)

    def log(self):

        omega = self.so3.log()
        theta = sympy.sqrt(sophus.squared_norm(omega))
        Omega = sophus.So3.hat(omega)

        half_theta = 0.5 * theta

        V_inv = sympy.Matrix.eye(3) - 0.5 * Omega + (1 - theta * sympy.cos(
            half_theta) / (2 * sympy.sin(half_theta))) / (theta * theta) *\
            (Omega * Omega)
        upsilon = V_inv * self.t
        return upsilon.col_join(omega)

    def __repr__(self):
        return "Se3: [" + repr(self.so3) + " " + repr(self.t)

    def inverse(self):
        invR = self.so3.inverse()
        return Se3(invR, invR * (-1 * self.t))

    @staticmethod
    def hat(v):
        """ R^6 => R^4x4  """
        """ returns 4x4-matrix representation ``Omega`` """
        upsilon = sophus.Vector3(v[0], v[1], v[2])
        omega = sophus.Vector3(v[3], v[4], v[5])
        return sophus.So3.hat(omega).\
            row_join(upsilon).\
            col_join(sympy.Matrix.zeros(1, 4))
    
    @staticmethod
    def vee(Omega):
        """ R^4x4 => R^6 """
        """ returns 6-vector representation of Lie algebra """
        """ This is the inverse of the hat-operator """
        
        head = sophus.Vector3(Omega[0,3], Omega[1,3], Omega[2,3])
        tail = sophus.So3.vee(Omega[0:3,0:3])
        upsilon_omega = \
            sophus.Vector6(head[0], head[1], head[2], tail[0], tail[1], tail[2])
        return upsilon_omega
            

    def matrix(self):
        """ returns matrix representation """
        R = self.so3.matrix()
        return (R.row_join(self.t)).col_join(sympy.Matrix(1, 4, [0, 0, 0, 1]))

    def __mul__(self, right):
        """ left-multiplication
            either rotation concatenation or point-transform """
        if isinstance(right, sympy.Matrix):
            assert right.shape == (3, 1), right.shape
            return self.so3 * right + self.t
        elif isinstance(right, Se3):
            r = self.so3 * right.so3
            t = self.t + self.so3 * right.t
            return Se3(r, t)
        assert False, "unsupported type: {0}".format(type(right))

    def __getitem__(self, key):
        """ We use the following convention [q0, q1, q2, q3, t0, t1, t2] """
        assert (key >= 0 and key < 7)
        if key < 4:
            return self.so3[key]
        else:
            return self.t[key - 4]

    @staticmethod
    def calc_Dx_exp_x(x):
        return sympy.Matrix(7, 6, lambda r, c:
                            sympy.diff(Se3.exp(x)[r], x[c]))

    @staticmethod
    def Dx_exp_x_at_0():
        return sympy.Matrix([[0.0, 0.0, 0.0, 0.5, 0.0, 0.0],
                             [0.0, 0.0, 0.0, 0.0, 0.5, 0.0],
                             [0.0, 0.0, 0.0, 0.0, 0.0, 0.5],
                             [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                             [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                             [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
                             [0.0, 0.0, 1.0, 0.0, 0.0, 0.0]])

    def calc_Dx_this_mul_exp_x_at_0(self, x):
        v = Se3.exp(x)
        return sympy.Matrix(7, 6, lambda r, c:
                            sympy.diff((self * Se3.exp(x))[r], x[c])). \
            subs(x[0], 0).subs(x[1], 0).subs(x[2], 0).\
            subs(x[3], 0).subs(x[4], 0).limit(x[5], 0)

    @staticmethod
    def calc_Dx_exp_x_at_0(x):
        return Se3.calc_Dx_exp_x(x).subs(x[0], 0).subs(x[1], 0).subs(x[2], 0).\
            subs(x[3], 0).subs(x[4], 0).limit(x[5], 0)

    @staticmethod
    def Dxi_x_matrix(x, i):
        if i < 4:
            return sophus.So3.Dxi_x_matrix(x, i).\
                row_join(sympy.Matrix.zeros(3, 1)).\
                col_join(sympy.Matrix.zeros(1, 4))
        M = sympy.Matrix.zeros(4, 4)
        M[i - 4, 3] = 1
        return M

    @staticmethod
    def calc_Dxi_x_matrix(x, i):
        return sympy.Matrix(4, 4, lambda r, c:
                            sympy.diff(x.matrix()[r, c], x[i]))

    @staticmethod
    def Dxi_exp_x_matrix(x, i):
        T = Se3.exp(x)
        Dx_exp_x = Se3.calc_Dx_exp_x(x)
        l = [Dx_exp_x[j, i] * Se3.Dxi_x_matrix(T, j) for j in range(0, 7)]
        return functools.reduce((lambda a, b: a + b), l)

    @staticmethod
    def calc_Dxi_exp_x_matrix(x, i):
        return sympy.Matrix(4, 4, lambda r, c:
                            sympy.diff(Se3.exp(x).matrix()[r, c], x[i]))

    @staticmethod
    def Dxi_exp_x_matrix_at_0(i):
        v = sophus.ZeroVector6()
        v[i] = 1
        return Se3.hat(v)

    @staticmethod
    def calc_Dxi_exp_x_matrix_at_0(x, i):
        return sympy.Matrix(4, 4, lambda r, c:
                            sympy.diff(Se3.exp(x).matrix()[r, c], x[i])
                            ).subs(x[0], 0).subs(x[1], 0).subs(x[2], 0).\
            subs(x[3], 0).subs(x[4], 0).limit(x[5], 0)


class TestSe3(unittest.TestCase):
    def setUp(self):
        upsilon0, upsilon1, upsilon2, omega0, omega1, omega2 = sympy.symbols(
            'upsilon[0], upsilon[1], upsilon[2], omega[0], omega[1], omega[2]',
            real=True)
        x, v0, v1, v2 = sympy.symbols('q.w() q.x() q.y() q.z()', real=True)
        p0, p1, p2 = sympy.symbols('p0 p1 p2', real=True)
        t0, t1, t2 = sympy.symbols('t[0] t[1] t[2]', real=True)
        v = sophus.Vector3(v0, v1, v2)
        self.upsilon_omega = sophus.Vector6(
            upsilon0, upsilon1, upsilon2, omega0, omega1, omega2)
        self.t = sophus.Vector3(t0, t1, t2)
        self.a = Se3(sophus.So3(sophus.Quaternion(x, v)), self.t)
        self.p = sophus.Vector3(p0, p1, p2)

    def test_exp_log(self):
        for v in [sophus.Vector6(0., 1, 0.5, 2., 1, 0.5),
                  sophus.Vector6(0.1, 0.1, 0.1, 0., 1, 0.5),
                  sophus.Vector6(0.01, 0.2, 0.03, 0.01, 0.2, 0.03)]:
            w = Se3.exp(v).log()
            for i in range(0, 3):
                self.assertAlmostEqual(v[i], w[i])

    def test_matrix(self):
        T_foo_bar = Se3.exp(self.upsilon_omega)
        Tmat_foo_bar = T_foo_bar.matrix()
        point_bar = self.p
        p1_foo = T_foo_bar * point_bar
        p2_foo = sophus.proj(Tmat_foo_bar * sophus.unproj(point_bar))
        self.assertEqual(sympy.simplify(p1_foo - p2_foo),
                         sophus.ZeroVector3())

    def test_derivatives(self):
        self.assertEqual(sympy.simplify(
            Se3.calc_Dx_exp_x_at_0(self.upsilon_omega) -
            Se3.Dx_exp_x_at_0()),
            sympy.Matrix.zeros(7, 6))

        for i in range(0, 7):
            self.assertEqual(sympy.simplify(Se3.calc_Dxi_x_matrix(self.a, i) -
                                            Se3.Dxi_x_matrix(self.a, i)),
                             sympy.Matrix.zeros(4, 4))
        for i in range(0, 6):
            self.assertEqual(sympy.simplify(
                Se3.Dxi_exp_x_matrix(self.upsilon_omega, i) -
                Se3.calc_Dxi_exp_x_matrix(self.upsilon_omega, i)),
                sympy.Matrix.zeros(4, 4))
            self.assertEqual(sympy.simplify(
                Se3.Dxi_exp_x_matrix_at_0(i) -
                Se3.calc_Dxi_exp_x_matrix_at_0(self.upsilon_omega, i)),
                sympy.Matrix.zeros(4, 4))

    def test_codegen(self):
        stream = sophus.cse_codegen(self.a.calc_Dx_exp_x(self.upsilon_omega))
        filename = "cpp_gencode/Se3_Dx_exp_x.cpp"
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
            self.upsilon_omega))
        filename = "cpp_gencode/Se3_Dx_this_mul_exp_x_at_0.cpp"
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
