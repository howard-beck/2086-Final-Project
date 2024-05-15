function [ts, xs] = prop_difeq(deriv, t0tf, x0)
    h = 0.1;

    t0 = t0tf(1);
    tf = t0tf(2);

    N = (tf - t0)/h;

    ts = linspace(t0, tf, N+1);
    [D, ~] = size(x0);

    xs = zeros(N+1, D);
    xs(1, :) = x0;

    x = x0;

    for i = 2:N
        t = ts(i - 1);

        k1 = deriv(t, x);
        k2 = deriv(t+h/2, x + h*k1/2);
        k3 = deriv(t+h/2, x + h*k2/2);
        k4 = deriv(t+h, x + h*k3);

        x = x + h/6 * (k1 + 2*k2 + 2*k3 + k4);

        xs(i, :) = x;
    end
end