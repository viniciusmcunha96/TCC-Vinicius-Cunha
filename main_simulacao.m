%% =======================================================================
%  TCC — Controle de perfil de velocidade com atuação robótica nos pedais
%  Simulação MIL (Model-in-the-Loop) do ensaio FTP-75 em dinamômetro de rolos
%
%  Vinícius Miglioli Cunha — UFMG / Engenharia de Controle e Automação
%   - entrada da planta = curso dos pedais
%   - modelo de robô, motor, conversor de torque, câmbio AT6, freio e
%     carga do dinamômetro, todos com parâmetros de fonte documentada
%     (ver parametros_sistema.m)
%   - planta com incertezas documentadas vs. modelo nominal do controlador
%   - 2 controladores comparados: PI (alocação de polos) e FF + PI
%   - métricas: erro, tolerância EPA (40 CFR 1066.425), SAE J2951,
%     esforço dos atuadores; ensaio de rejeição de perturbação
%
%  Arquivos:  parametros_sistema  carregar_ciclo  cenario_perturbacao
%             planta_init  planta_passo  ctrl_init  ctrl_passo
%             simular_ciclo  metricas_ciclo  plotar_resultados
%             conversor_qs  interp_lin
%             construir_modelo_simulink (versão em blocos)
%
%% =======================================================================
clc; clear; close all;

modo_rapido = false;   % true -> simula só a fase 1 (505 s) para iterar rápido

[p, pr] = parametros_sistema();
ciclo   = carregar_ciclo('FTP-75.csv', p.Ts);
if modo_rapido                                     % recorta na fase 1 (505 s)
    idx = ciclo.t <= 505;  k1 = ciclo.t1 <= 505;
    ciclo.t = ciclo.t(idx);  ciclo.v = ciclo.v(idx);  ciclo.a = ciclo.a(idx);
    ciclo.N = sum(idx);  ciclo.t1 = ciclo.t1(k1);  ciclo.v1 = ciclo.v1(k1);
    ciclo.v1_mph = ciclo.v1_mph(k1);
end

nomes = {'PI', 'FF + PI'};
nc = numel(nomes);
L = cell(1,nc);  M = cell(1,nc);
for tipo = 1:nc
    fprintf('Simulando %-13s ... ', nomes{tipo});  tic;
    L{tipo} = simular_ciclo(p, pr, ciclo, tipo);
    M{tipo} = metricas_ciclo(L{tipo}, p, ciclo);
    fprintf('%.1f s\n', toc);
end

%% ---- Tabela de resultados ------------------------------------------------
fprintf(['\n%-28s' repmat(' %12s', 1, nc) '\n'], 'Métrica', nomes{:});
linha = @(rot, campo, fmt) fprintf(['%-28s' repmat([' %12' fmt], 1, nc) '\n'], rot, ...
         M{1}.(campo), M{2}.(campo));
linha('RMSE (km/h)',            'RMSE',  '.3f');
linha('MAE (km/h)',             'MAE',   '.3f');
linha('Erro máximo (km/h)',     'EMAX',  '.3f');
linha('Tempo fora da faixa (s)','tempo_fora_s', '.1f');
linha('Excursões > 2 s (EPA)',  'violacoes_2s', 'd');
linha('RMSSE (mph)',            'RMSSE', '.3f');
linha('IWR (%)',                'IWR',   '.2f');
linha('ER (%)',                 'ER',    '.2f');
linha('DR (%)',                 'DR',    '.2f');
linha('EER (%)',                'EER',   '.2f');
linha('ASCR (%)',               'ASCR',  '.2f');
linha('Curso pedais (%/km)',    'curso_pedais_pct_km', '.0f');
linha('Trocas acel<->freio',    'trocas_pedal', 'd');

% linhas prontas para LaTeX (tabela da monografia)
fprintf('\n%% --- LaTeX ---\n');
campos = {'RMSE','MAE','EMAX','RMSSE','IWR','violacoes_2s','trocas_pedal'};
for tipo = 1:nc
    fprintf('%s', nomes{tipo});
    for i = 1:numel(campos), fprintf(' & %.3g', M{tipo}.(campos{i})); end
    fprintf(' \\\\\n');
end

%% ---- Ensaio de rejeição de perturbação e degrau ------------------------
[cp, grade] = cenario_perturbacao(p.Ts);
Lp = cell(1,nc);
for tipo = 1:nc
    Lp{tipo} = simular_ciclo(p, pr, cp, tipo, grade);
end

%% ---- Salva os resultados ANTES dos gráficos ----------------------------
% Se a geração das figuras falhar ou for interrompida, não é preciso
% simular de novo: rode apenas  gerar_figuras
save('resultados_simulacao.mat', 'L', 'M', 'Lp', 'nomes', 'p', 'pr', 'ciclo');
fprintf('\nResultados salvos em resultados_simulacao.mat\n');

%% ---- Gráficos -----------------------------------------------------------
fprintf('Gerando figuras...\n');
plotar_resultados(L, M, Lp, nomes, ciclo);

