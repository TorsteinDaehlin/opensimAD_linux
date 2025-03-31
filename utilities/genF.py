import casadi as ca
import importlib

def generateF(dim):
    import foo
    importlib.reload(foo)
    cg = ca.CodeGenerator('foo_jac')
    arg = ca.SX.sym('arg', dim)
    y, _, _ = foo.foo(arg)
    F = ca.Function('F', [arg], [y])
    cg.add(F)
    cg.add(F.jacobian())
    cg.generate()

generateF(nInput)