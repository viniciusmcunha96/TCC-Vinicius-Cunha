function construir_modelo_simulink(nome)
%CONSTRUIR_MODELO_SIMULINK  Gera o modelo Simulink do robô condutor (MIL).
%
%   construir_modelo_simulink            -> cria 'robo_condutor_mil.slx'
%
%   O modelo reaproveita AS MESMAS funções dos scripts (ctrl_passo,
%   planta_passo, ...) dentro de blocos MATLAB Function, de modo que
%   script e diagrama de blocos dão o mesmo resultado. Estrutura:
%
%   [Referência] -> [Controlador] -> [Planta: robô + veículo + dinamômetro]
%        ^               ^                         |
%        |               +--[velocidade do rolo]---+
%        |               +--[rpm, marcha (OBD/CAN)]+
%
%   O bloco "Controlador" é a fronteira do software embarcado: é ele que
%   será gerado em C (Embedded Coder) nas etapas SIL/HIL do protótipo.
%
%   Executar depois com: rodar_modelo_simulink
%   Requer: Simulink. Pasta destes arquivos no path do MATLAB.

if nargin < 1, nome = 'robo_condutor_mil'; end
[p, ~] = parametros_sistema();

if bdIsLoaded(nome), close_system(nome, 0); end
if exist([nome '.slx'], 'file'), delete([nome '.slx']); end
new_system(nome);
set_param(nome, 'Solver', 'FixedStepDiscrete', 'FixedStep', 'Ts', ...
                'StopTime', 'ciclo_tf');

%% ---------------- Blocos -------------------------------------------------
add_block('simulink/Sources/From Workspace', [nome '/Referencia'], ...
    'VariableName', 'ref_data', 'SampleTime', 'Ts', 'Interpolate', 'off', ...
    'OutputAfterFinalValue', 'Holding final value', 'Position', [30 80 110 120]);
add_block('simulink/Signal Routing/Demux', [nome '/DemuxRef'], ...
    'Outputs', '3', 'Position', [150 60 155 160]);
add_block('simulink/Sources/From Workspace', [nome '/Inclinacao'], ...
    'VariableName', 'grade_data', 'SampleTime', 'Ts', 'Interpolate', 'off', ...
    'OutputAfterFinalValue', 'Holding final value', 'Position', [300 230 380 260]);

add_block('simulink/User-Defined Functions/MATLAB Function', [nome '/Controlador'], ...
    'Position', [250 50 400 200]);
add_block('simulink/User-Defined Functions/MATLAB Function', [nome '/Planta'], ...
    'Position', [480 60 630 200]);

% Medição de velocidade do rolo (ideal; a amostragem introduz um período de atraso)
add_block('simulink/Discrete/Unit Delay', [nome '/Atraso_v'], ...
    'InitialCondition', '0', 'SampleTime', 'Ts', 'Position', [700 70 730 100]);
add_block('simulink/Discrete/Unit Delay', [nome '/Atraso_rpm'], ...
    'InitialCondition', 'p.eng.rpm_idle', 'SampleTime', 'Ts', 'Position', [700 290 730 320]);
add_block('simulink/Discrete/Unit Delay', [nome '/Atraso_marcha'], ...
    'InitialCondition', '1', 'SampleTime', 'Ts', 'Position', [700 340 730 370]);

% Registro
add_block('simulink/Signal Routing/Mux', [nome '/MuxLog'], ...
    'Inputs', '7', 'Position', [760 420 765 560]);
add_block('simulink/Sinks/To Workspace', [nome '/Log'], ...
    'VariableName', 'log_sim', 'SaveFormat', 'Array', 'Position', [800 475 880 505]);

%% ---------------- Código dos blocos MATLAB Function ----------------------
cod_ctrl = sprintf([ ...
 'function [pa_cmd, pb_cmd, F_cmd] = controlador(v_ref, a_ref, v_fut, v, rpm, marcha, p, tipo)\n' ...
 '%%#codegen\n' ...
 '%% Software embarcado do robô: um período Ts por chamada.\n' ...
 'persistent c\n' ...
 'if isempty(c), c = ctrl_init(p, tipo); end\n' ...
 '[c, pa_cmd, pb_cmd] = ctrl_passo(c, v_ref, a_ref, v_fut, v, rpm, marcha, p);\n' ...
 'F_cmd = c.F_cmd;\n']);
cod_planta = sprintf([ ...
 'function [v, rpm, marcha, pa, pb] = planta(pa_cmd, pb_cmd, grade, pr, n_sub)\n' ...
 '%%#codegen\n' ...
 '%% Robô + motor + conversor + AT6 + freio + veículo no dinamômetro.\n' ...
 'persistent s\n' ...
 'if isempty(s), s = planta_init(pr, 0); end\n' ...
 'prl = pr;  prl.dyn.grade = grade;\n' ...
 'for i = 1:n_sub\n' ...
 '    s = planta_passo(s, pa_cmd, pb_cmd, prl);\n' ...
 'end\n' ...
 'v = s.v;  rpm = s.we*30/pi;  marcha = s.marcha;  pa = s.pa;  pb = s.pb;\n']);

rt = sfroot;
ch = rt.find('-isa', 'Stateflow.EMChart', 'Path', [nome '/Controlador']);
ch.Script = cod_ctrl;
configurar_parametros(ch, {'p', 'tipo'});
configurar_escalares(ch);
ch = rt.find('-isa', 'Stateflow.EMChart', 'Path', [nome '/Planta']);
ch.Script = cod_planta;
configurar_parametros(ch, {'pr', 'n_sub'});
configurar_escalares(ch);

%% ---------------- Ligações -----------------------------------------------
L = @(a, b) add_line(nome, a, b, 'autorouting', 'on');
L('Referencia/1', 'DemuxRef/1');
for i = 1:3, L(sprintf('DemuxRef/%d', i), sprintf('Controlador/%d', i)); end
L('Controlador/1', 'Planta/1');
L('Controlador/2', 'Planta/2');
L('Inclinacao/1',  'Planta/3');
% realimentação de velocidade (rolo do dinamômetro)
L('Planta/1', 'Atraso_v/1');       L('Atraso_v/1', 'Controlador/4');
% rpm e marcha (OBD-II / CAN)
L('Planta/2', 'Atraso_rpm/1');     L('Atraso_rpm/1', 'Controlador/5');
L('Planta/3', 'Atraso_marcha/1');  L('Atraso_marcha/1', 'Controlador/6');
% registro: [vref v pa pb F rpm marcha]
L('DemuxRef/1', 'MuxLog/1');  L('Planta/1', 'MuxLog/2');  L('Planta/4', 'MuxLog/3');
L('Planta/5', 'MuxLog/4');    L('Controlador/3', 'MuxLog/5');
L('Planta/2', 'MuxLog/6');    L('Planta/3', 'MuxLog/7');
L('MuxLog/1', 'Log/1');

save_system(nome);
fprintf('Modelo %s.slx criado. Rode: rodar_modelo_simulink\n', nome);
end

function configurar_parametros(ch, nomes)
% Argumentos vindos do workspace como parâmetros NÃO ajustáveis:
% dimensões e tabelas ficam constantes na compilação (exigência do Coder).
for i = 1:numel(nomes)
    d = ch.find('-isa', 'Stateflow.Data', 'Name', nomes{i});
    d.Scope   = 'Parameter';
    d.Tunable = false;
end
end

function configurar_escalares(ch)
% Fixa todas as entradas e saídas do bloco como escalares double.
% Sem isso, o Simulink tenta inferir as dimensões pela malha fechada
% (Planta -> Unit Delay -> Controlador -> Planta) e não consegue resolver:
% "Block 'Planta' does not fully set the dimensions of output 'v'".
dados = ch.find('-isa', 'Stateflow.Data');
for i = 1:numel(dados)
    d = dados(i);
    if any(strcmp(d.Scope, {'Input', 'Output'}))
        d.Props.Array.Size   = '1';
        d.DataType           = 'double';
        d.Props.Complexity   = 'Off';
    end
end
end
