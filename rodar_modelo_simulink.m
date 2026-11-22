%% Executa o modelo Simulink do robô condutor no FTP-75 e calcula as métricas
%  (mesmos resultados de main_simulacao.m, a menos do gerador de ruído).
clc; clear;
tipo  = 2;                         % 1 PI | 2 FF + PI
nome  = 'robo_condutor_mil';

[p, pr] = parametros_sistema();
ciclo   = carregar_ciclo('FTP-75.csv', p.Ts);
Ts      = p.Ts;
n_sub   = round(p.Ts/p.dt);
N       = ciclo.N;
kf      = min((1:N).' + round(1.0/Ts),  N);    % 1 s à frente
ref_data   = [ciclo.t, ciclo.v, ciclo.a, ciclo.v(kf)];
grade_data = [ciclo.t, zeros(N, 1)];
ciclo_tf   = ciclo.t(end);

construir_modelo_simulink(nome);      % reconstrói sempre (evita usar um .slx antigo)
out = sim(nome, 'ReturnWorkspaceOutputs', 'on');
X   = out.get('log_sim');                      % [vref v pa pb F rpm marcha]

% reamostra a 10 Hz no mesmo formato de simular_ciclo
k10 = 1:round(p.Ts_log/Ts):size(X, 1);
L.t  = ciclo.t(k10) + Ts;
L.vref = interp1(ciclo.t, ciclo.v, min(L.t, ciclo.t(end)));
L.v  = X(k10, 2);  L.pa = X(k10, 3);  L.pb = X(k10, 4);  L.F = X(k10, 5);
L.rpm = X(k10, 6); L.marcha = X(k10, 7);
L.trocas_pedal = sum(abs(diff(sign(X(:,3) - X(:,4)))) > 1);   % aproximação
M = metricas_ciclo(L, p, ciclo);
fprintf('Simulink (tipo %d): RMSE %.3f km/h | MAE %.3f | máx %.3f | excursões>2 s: %d | IWR %.2f %%\n', ...
        tipo, M.RMSE, M.MAE, M.EMAX, M.violacoes_2s, M.IWR);

figure('Color', 'w');
plot(L.t, L.vref*3.6, 'k--', L.t, L.v*3.6, 'b'); grid on;
xlabel('Tempo (s)'); ylabel('km/h'); legend('Referência', 'Simulink');
title(sprintf('Modelo Simulink — RMSE %.3f km/h', M.RMSE));
