import sys
import os
import casadi as ca
import importlib

def generateF(dim, fooPath):
    sys.path.append(fooPath)
    os.chdir(fooPath)
    import foo
    importlib.reload(foo)
    cg = ca.CodeGenerator('foo_jac')
    arg = ca.SX.sym('arg', dim)
    y, _, _ = foo.foo(arg)
    F = ca.Function('F', [arg], [y])
    cg.add(F)
    cg.add(F.jacobian())
    cg.generate()

nInput = int(sys.argv[1])
fooPath = sys.argv[2]
generateF(nInput, fooPath)