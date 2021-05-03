function y = calcolo_jacobiano(q)

%syms q

x = q^2;

y = jacobian(x,q)
