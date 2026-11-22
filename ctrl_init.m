function c = ctrl_init(p, tipo)
%CTRL_INIT  Inicializa o controlador de velocidade do robô condutor.
%
%   tipo = 1 'PI'    : PI por alocação de polos (Åström e Hägglund, 2006)
%   tipo = 2 'FF_PI' : PI + alimentação direta pelo modelo inverso nominal
%
%   Estado de tamanho fixo -> compatível com bloco MATLAB Function / Coder.

c = struct( ...
    'tipo',  tipo, ...
    'I',     0, ...        % integral do erro (m)
    'e_ant', 0, ...        % erro anterior (Tustin)
    'modo',  0, ...        % 1 acelerador | 0 nenhum pedal | -1 freio | 2 parado
    'F_cmd', 0, ...        % N  força pedida
    'pa',    0, ...        % %  comando acelerador
    'pb',    p.brk.hold, ...
    'trocas_pedal', 0, ... % transições acelerador <-> freio
    'ultimo_pedal', 0, ... % 1 acel / -1 freio
    'zeta',  1.0, ...      % requisitos do Cap. 4 da monografia: zeta = 1
    'wn',    5/3);         % t5% ~ 5/wn = 3 s  -> wn = 5/3 rad/s
end
