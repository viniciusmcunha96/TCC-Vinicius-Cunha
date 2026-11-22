function [p, pr] = parametros_sistema()
%PARAMETROS_SISTEMA  Parâmetros nominais (p) e da planta simulada (pr).
%
%   p  -> valores NOMINAIS, usados pelo controlador
%   pr -> valores da PLANTA simulada (nominal + incertezas documentadas)
%

%% ---------------- Veículo e dinamômetro ---------------------------------
lb = 0.45359237;                     % kg/lb
p.veh.m_curb = 1304;                 % kg  massa em ordem de marcha [FT]
% massa de ensaio: loaded vehicle weight = curb + 300 lb [CFR 86.1803-01]
%   1304 kg + 136 kg = 1440 kg = 3175 lb -> classe 3063–3187 lb -> ETW = 3125 lb
%   [CFR 1066.805, Tabela 1]
p.veh.m      = 3125*lb;              % kg  1417,5 kg = inércia simulada pelo dinamômetro
p.veh.g      = 9.81;                 % m/s² [MON]
% raio do pneu 215/45 R18 [FT]: r = (18*25,4/2 + 0,45*215)/1000 [CALC]
p.veh.r      = (18*25.4/2 + 0.45*215)/1000;   % m  0,3254 m (raio geométrico)
p.veh.rho = 1.05;  p.veh.Cd = 0.32;  p.veh.A = 2.19;  p.veh.Cr = 0.015;   % [MON]

% Carga do dinamômetro F = A + B v + C v² + m g sen(θ)  [CFR 1066.210(d)(3)]
p.dyn.A = p.veh.m*p.veh.g*p.veh.Cr;               % N        [CALC, MON]
p.dyn.B = 0;                                      % N/(m/s)  modelo físico da monografia não tem termo linear
p.dyn.C = 0.5*p.veh.rho*p.veh.Cd*p.veh.A;         % N/(m/s)² [CALC, MON]
p.dyn.grade = 0;                                  % rad  inclinação simulada (perturbação)

%% ---------------- Motor 1.3 T270 ----------------------------------------
cv = 735.49875;                                   % W/cv
p.eng.rpm_Tmax = 1750;   p.eng.Tmax = 270;        % rpm, N·m [FT]
p.eng.rpm_Pmax = 5750;                            % rpm [FT]
p.eng.Pmax     = 185*cv;                          % W  185 cv (etanol) [FT] — valor nominal
% Curva de plena carga pelo método de [SAI11]: pontos característicos
%   T_Pmax = Pmax/w_Pmax ; T1000 = T_Pmax/1,273 ; T1500 = T_Pmax/1,095
%   (razões médias para motores de ignição por centelha, Tabela II de [SAI11])
%   e interpolação linear entre os pontos; fora do intervalo, valor da ponta.
p.eng = curva_plena_carga(p.eng);
% Mapa do pedal [SAI11, eq. 5]: Te = T_FL(w)*sqrt(x), x = curso normalizado 0..1
% Perdas [SAI11]: T_loss = 20 % do torque no ponto de potência máxima (constante)
p.eng.Tloss    = 0.20*p.eng.Pmax/(p.eng.rpm_Pmax*pi/30);   % N·m [SAI11, CALC]
p.eng.rpm_idle = 650;                             % rpm  marcha lenta [EPA16]
p.eng.rpm_max  = p.eng.rpm_Pmax;                  % rpm  limite superior usado no modelo [FT]

%% ---------------- Transmissão Aisin AT6 ---------------------------------
p.tr.ig  = [4.459 2.508 1.556 1.142 0.852 0.672]; % [FT]
p.tr.ifd = 3.683;                                 % [FT]
% Lógica de trocas: versão simplificada do ALPHAshift [NEW15]
p.tr.w_min      = 115;     % rad/s  min_speed (ponto de partida, Malibu) [NEW15]
p.tr.w_up_min   = 125;     % rad/s  upshift_min_speed = min_speed + 10 rad/s [NEW15]
p.tr.reserva    = 0.10;    % -      upshift_min_torque_reserve_ratio [NEW15]
p.tr.t_commit_up= 1.5;     % s      upshift_commit_secs (6 marchas) [NEW15]
p.tr.t_commit_dn= 1.0;     % s      downshift_commit_secs (6 marchas) [NEW15]
p.tr.t_delay    = 0.1;     % s      upshift/downshift_delay_secs [NEW15]
p.tr.fr_dn_max  = 0.85;    % -      downshift_max_speed = 85 % de max_speed [NEW15]

%% ---------------- Conversor de torque (curvas genéricas [MW]) ------------
p.tc.SR    = [0 0.5 0.6 0.7 0.8 0.87 0.92 0.94 0.96 0.97 1.0];
p.tc.TR    = [2.232 1.5462 1.4058 1.2746 1.1528 1.0732 1.0192 0.9983 0.9983 0.9983 0.9983];
Kmw        = [12.2938 12.8588 13.1452 13.6285 14.6163 16.2675 19.3503 22.1046 29.9986 50];
% 1/K² (N·m/(rad/s)²); em SR = 1 não há escorregamento -> torque nulo
p.tc.invK2 = [1./Kmw.^2, 0];

%% ---------------- Freio ----------------------------------------------------
p.brk.mu   = 0.80;                                % pico, asfalto seco (0,80–0,90) [WON93]
p.brk.Fmax = p.brk.mu*p.veh.m*p.veh.g;            % N  [CALC]
p.brk.hold = 100;                                 % %  freio pleno com o veículo parado

%% ---------------- Atuadores robóticos -------------------------------------
% Dinâmica do servo desprezada, como em [SAI11]; mantém-se só o limite de
% velocidade do robô de acelerador: 300 mm/s em curso de 130 mm [ABD22]
p.act.taxa_max = 100*300/130;                     % %/s  230,8 %/s [ABD22, CALC]

%% ---------------- Amostragem ----------------------------------------------
p.Ts     = 0.01;      % s  controlador e integração (100 Hz) [MON]
p.dt     = 0.01;      % s  passo da planta (modelo sem dinâmicas rápidas)
p.Ts_log = 0.1;       % s  registro a 10 Hz [CFR 1066.210(d)(2)]

%% ---------------- Planta = nominal + incertezas documentadas --------------
pr = p;
% combustível de ensaio (gasolina): 180 cv em vez de 185 cv (etanol) [FT]
pr.eng.Pmax  = 180*cv;
pr.eng       = curva_plena_carga(pr.eng);
pr.eng.Tloss = 0.20*pr.eng.Pmax/(pr.eng.rpm_Pmax*pi/30);
% erro admissível de aplicação da carga pelo dinamômetro: +1 % [CFR 1066.210(d)(4)]
pr.dyn.A = 1.01*p.dyn.A;
pr.dyn.C = 1.01*p.dyn.C;
end

%% ---------------------------------------------------------------------
function eng = curva_plena_carga(eng)
% Pontos característicos da curva de plena carga [SAI11] + ficha técnica
wP    = eng.rpm_Pmax*pi/30;
T_P   = eng.Pmax/wP;                  % torque na potência máxima
T1000 = T_P/1.273;                    % razão T_Pmax/T1000 (ignição por centelha) [SAI11]
T1500 = T_P/1.095;                    % razão T_Pmax/T1500 [SAI11]
eng.rpm_tab  = [1000  1500  eng.rpm_Tmax  eng.rpm_Pmax];
eng.Tmax_tab = [T1000 T1500 eng.Tmax      T_P];
end
