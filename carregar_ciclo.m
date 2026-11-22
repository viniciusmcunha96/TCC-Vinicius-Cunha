function ciclo = carregar_ciclo(arquivo, Ts)
%CARREGAR_CICLO  Lê o ciclo de condução (CSV ; separado) e reamostra em Ts.
%
%   Colunas esperadas: timestamp(s) ; Speed(mph) ; Speed(kph)
%   A referência original é dada a 1 Hz. O traço exibido ao motorista no
%   dinamômetro é a interpolação LINEAR desses pontos, e é contra ela que a
%   tolerância da EPA (40 CFR 1066.425) é verificada — por isso o
%   controlador recebe o traço interpolado, e não degraus a cada 1 s.

fid = fopen(arquivo, 'r');
if fid < 0, error('Não foi possível abrir %s', arquivo); end
fgetl(fid);                                       % cabeçalho
C = textscan(fid, '%f %f %f', 'Delimiter', ';');
fclose(fid);

ciclo.t1   = C{1} - C{1}(1);                      % s   (1 Hz)
ciclo.v1   = C{3} / 3.6;                          % m/s (1 Hz)
ciclo.v1_mph = C{2};
ciclo.Ts   = Ts;
ciclo.t    = (0:Ts:ciclo.t1(end)).';
ciclo.v    = interp1(ciclo.t1, ciclo.v1, ciclo.t, 'linear');
% aceleração do traço (constante por trechos entre pontos de 1 s)
a1         = [diff(ciclo.v1) ./ diff(ciclo.t1); 0];
ciclo.a    = interp1(ciclo.t1, a1, ciclo.t, 'previous');
ciclo.a(isnan(ciclo.a)) = 0;
ciclo.N    = numel(ciclo.t);
end
