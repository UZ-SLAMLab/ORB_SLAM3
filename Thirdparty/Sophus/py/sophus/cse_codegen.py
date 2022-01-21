import sympy
import io


def cse_codegen(symbols):
    cse_results = sympy.cse(symbols, sympy.numbered_symbols("c"))
    output = io.StringIO()
    for helper in cse_results[0]:
        output.write("Scalar const ")
        output.write(sympy.printing.ccode(helper[1], helper[0]))
        output.write("\n")
    assert len(cse_results[1]) == 1

    output.write(sympy.printing.ccode(cse_results[1][0], "result"))
    output.write("\n")
    output.seek(0)
    return output
