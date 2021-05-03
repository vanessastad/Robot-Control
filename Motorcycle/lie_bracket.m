function dL = lie_bracket(f, g, q)

dL = (jacobian(g, q)) * f - (jacobian(f, q)) * g;