function [ciclo, grade] = cenario_perturbacao(Ts)
%CENARIO_PERTURBACAO  Ensaio sintético para verificar os requisitos do Cap. 4:
%   - partida 0 -> 60 km/h em 15 s
%   - patamar de 60 km/h
%   - t = 40..80 s : rampa de +4 % simulada pelo dinamômetro (perturbação degrau)
%   - t = 100 s    : degrau de referência 60 -> 70 km/h (sobressinal, t. acomodação)
t1 = (0:150).';
v1 = zeros(size(t1));
v1(t1 >= 5 & t1 <= 20) = (t1(t1 >= 5 & t1 <= 20) - 5)/15*60;
v1(t1 > 20)  = 60;
v1(t1 >= 100) = 70;
v1 = v1/3.6;

ciclo.t1 = t1;  ciclo.v1 = v1;  ciclo.v1_mph = v1/0.44704;  ciclo.Ts = Ts;
ciclo.t  = (0:Ts:t1(end)).';
ciclo.v  = interp1(t1, v1, ciclo.t, 'linear');
a1       = [diff(v1); 0];
ciclo.a  = interp1(t1, a1, ciclo.t, 'previous');  ciclo.a(isnan(ciclo.a)) = 0;
ciclo.N  = numel(ciclo.t);

grade = zeros(ciclo.N, 1);
grade(ciclo.t >= 40 & ciclo.t < 80) = atan(0.04);
end
