function y = interp_lin(xt, yt, x)
%INTERP_LIN  Interpolação linear em tabela com saturação nas pontas.
%   Substitui interp1 nos laços de tempo real: mais rápida e direta de
%   portar para C (lookup table de microcontrolador).
%#codegen
n = numel(xt);
if x <= xt(1), y = yt(1); return; end
if x >= xt(n), y = yt(n); return; end
i = 1;
while x > xt(i+1)
    i = i + 1;
end
y = yt(i) + (yt(i+1) - yt(i))*(x - xt(i))/(xt(i+1) - xt(i));
end
