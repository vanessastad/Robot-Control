function dL = dir_der(f, h, q)
L = h * f;
dL = simplify(jacobian(L,q));
