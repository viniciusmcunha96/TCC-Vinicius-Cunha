function M = metricas_ciclo(L, p, ciclo)
%METRICAS_CICLO  Métricas de seguimento, conformidade EPA e SAE J2951.
%
%  1) Erro: RMSE, MAE, máximo (km/h) — mesmas da Tabela 2 da monografia
%  2) Tolerância EPA (40 CFR 1066.425): limite superior = 2 mph acima do
%     ponto mais alto do traço dentro de ±1 s; inferior = 2 mph abaixo do
%     mais baixo. Excursões são aceitas se durarem até 2,0 s cada.
%  3) SAE J2951 (qualidade de condução em dinamômetro), a 10 Hz:
%     ER, DR, EER, ASCR, IWR (%) e RMSSE (mph).
%     Driven (D) = velocidade simulada; Target (T) = traço.
%  4) Esforço dos atuadores: curso total dos pedais e trocas acel<->freio.

dt  = p.Ts_log;
vT  = L.vref(:);  vD = L.v(:);
e   = (vT - vD)*3.6;                                   % km/h
M.RMSE = sqrt(mean(e.^2));
M.MAE  = mean(abs(e));
M.EMAX = max(abs(e));

% ---- tolerância EPA ------------------------------------------------------
mph  = 1/0.44704;
tt   = L.t(:);
tf   = (0:0.1:ciclo.t1(end)).';
vf   = interp1(ciclo.t1, ciclo.v1, tf)*mph;
w    = 10;                                             % ±1 s a 10 Hz
vmax = movmax(vf, [w w]);  vmin = movmin(vf, [w w]);
up   = interp1(tf, vmax, min(tt, tf(end))) + 2;
lo   = interp1(tf, vmin, min(tt, tf(end))) - 2;
fora = (vD*mph > up) | (vD*mph < lo);
M.tempo_fora_s = sum(fora)*dt;
[ini, fim] = trechos(fora);
dur = (fim - ini + 1)*dt;
M.eventos_fora   = numel(dur);
M.violacoes_2s   = sum(dur > 2.0);                     % reprovariam o ensaio
M.max_evento_s   = max([0; dur(:)]);
M.lim_sup = up/mph;  M.lim_inf = lo/mph;               % m/s (para plotagem)

% ---- SAE J2951 -----------------------------------------------------------
ETW = p.veh.m;  A = p.dyn.A;  B = p.dyn.B;  C = p.dyn.C;
[CE_T, IW_T, D_T, ASC_T] = j2951(vT, dt, ETW, A, B, C);
[CE_D, IW_D, D_D, ASC_D] = j2951(vD, dt, ETW, A, B, C);
M.ER    = (CE_D - CE_T)/CE_T*100;
M.DR    = (D_D - D_T)/D_T*100;
M.EER   = (1 - (D_D/CE_D)/(D_T/CE_T))*100;
M.ASCR  = (ASC_D - ASC_T)/ASC_T*100;
M.IWR   = (IW_D - IW_T)/IW_T*100;
M.RMSSE = sqrt(mean(((vD - vT)*mph).^2));

% ---- esforço dos atuadores -----------------------------------------------
km = D_D/1000;
M.curso_pedais_pct_km = (sum(abs(diff(L.pa))) + sum(abs(diff(L.pb))))/km;
M.trocas_pedal = L.trocas_pedal;
end

function [CE, IW, D, ASC] = j2951(v, dt, ETW, A, B, C)
a   = [diff(v); 0]/dt;                 % aceleração (diferença à frente)
d   = v*dt;                            % distância por amostra
FI  = ETW*a;                           % força inercial
F   = FI + A*(v > 0.01) + B*v + C*v.^2;
CE  = sum(max(F, 0).*d);               % energia do ciclo (só tração)
IW  = sum(max(FI, 0).*d);              % trabalho inercial
D   = sum(d);
ASC = sum(abs(diff(v)));               % variação absoluta de velocidade
end

function [ini, fim] = trechos(b)
b   = b(:).';
db  = diff([0 b 0]);
ini = find(db == 1);
fim = find(db == -1) - 1;
end
