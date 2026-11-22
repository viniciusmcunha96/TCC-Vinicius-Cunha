function s = planta_init(pr, v0)
%PLANTA_INIT  Estado inicial da planta (robô + veículo no dinamômetro).
%   Struct montado de uma vez (exigência da geração de código do Simulink).
%#codegen
if nargin < 2, v0 = 0; end
s = struct( ...
    'v',        v0, ...                    % m/s  velocidade do veículo / rolo
    'x',        0, ...                     % m    distância
    'pa',       0, ...                     % %    posição do pedal do acelerador
    'pb',       pr.brk.hold, ...           % %    posição do pedal do freio (parte freado)
    'we',       pr.eng.rpm_idle*pi/30, ... % rad/s rotação do motor
    'Te',       0, ...                     % N·m  torque do motor (antes das perdas)
    'Tt',       0, ...                     % N·m  torque na turbina
    'Ftr',      0, ...                     % N    força trativa na roda
    'Fb',       0, ...                     % N    força de frenagem
    'Fdyn',     0, ...                     % N    força aplicada pelo dinamômetro
    'marcha',   1, ...
    't_ult',    10, ...                    % s    tempo desde a última troca
    'ult_up',   true, ...                  % última troca foi ascendente?
    't_up',     0, ...                     % s    tempo com condição de subida ativa
    't_dn',     0);                        % s    tempo com condição de descida ativa
end
