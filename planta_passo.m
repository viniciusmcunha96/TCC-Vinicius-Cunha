function s = planta_passo(s, pa_cmd, pb_cmd, pr)
%PLANTA_PASSO  Integra a planta por um passo pr.dt.
%
%   Cadeia modelada (entrada = comandos de posição dos pedais, %):
%     robô (limite de velocidade [AB Dynamics, 2022]; dinâmica do servo
%       desprezada [Sailer et al., 2011])
%     -> motor: Te = T_FL(we)*sqrt(x) - T_loss          [Sailer et al., 2011]
%     -> conversor de torque quase-estático              [MathWorks]
%     -> câmbio AT6 + diferencial, lógica de trocas      [Newman et al., 2015]
%     -> força trativa  |  freio: F_b = mu m g * x_b     [Wong, 1993]
%     -> dinâmica longitudinal no dinamômetro:
%        m dv/dt = F_tr - F_b - (A + C v² + m g sen θ)   [40 CFR 1066.210]
%#codegen
dt = pr.dt;

%% 1) Robô: limite de velocidade de curso
dmax = pr.act.taxa_max*dt;
s.pa = min(max(s.pa + min(max(min(max(pa_cmd,0),100) - s.pa, -dmax), dmax), 0), 100);
s.pb = min(max(s.pb + min(max(min(max(pb_cmd,0),100) - s.pb, -dmax), dmax), 0), 100);
x  = s.pa/100;

%% 2) Lógica de trocas (ALPHAshift simplificado, sem mapa de consumo)
ig   = pr.tr.ig;
g    = s.marcha;
r    = pr.veh.r;
wt   = s.v/r*ig(g)*pr.tr.ifd;                % rotação da turbina
wmax = pr.eng.rpm_max*pi/30;
s.t_ult = s.t_ult + dt;
commit = pr.tr.t_commit_dn;
if s.ult_up, commit = pr.tr.t_commit_up; end

cond_up = false;  cond_dn = false;
if g < 6
    wt_novo = wt*ig(g+1)/ig(g);
    % torque de motor necessário na marcha acima para a mesma força na roda
    Te_novo = s.Te*ig(g)/ig(g+1);
    T_disp  = interp_lin(pr.eng.rpm_tab, pr.eng.Tmax_tab, wt_novo*30/pi);
    cond_up = (wt_novo >= pr.tr.w_up_min && Te_novo <= (1 - pr.tr.reserva)*T_disp) ...
              || wt > wmax;
end
if g > 1
    wt_baixo = wt*ig(g-1)/ig(g);
    % abaixo da rotação mínima, ou demanda acima do disponível (pedal a fundo)
    cond_dn = wt < pr.tr.w_min || ...
              (s.pa >= 100 && wt_baixo <= pr.tr.fr_dn_max*wmax);
end
if cond_up, s.t_up = s.t_up + dt; else, s.t_up = 0; end
if cond_dn, s.t_dn = s.t_dn + dt; else, s.t_dn = 0; end
if s.t_ult >= commit
    if s.t_dn >= pr.tr.t_delay
        s.marcha = g - 1;  s.t_ult = 0;  s.ult_up = false;  s.t_dn = 0;  s.t_up = 0;
    elseif s.t_up >= pr.tr.t_delay
        s.marcha = g + 1;  s.t_ult = 0;  s.ult_up = true;   s.t_up = 0;  s.t_dn = 0;
    end
end
ratio = ig(s.marcha)*pr.tr.ifd;
wt    = s.v/r*ratio;

%% 3) Motor + conversor (quase-estático)
[we, Tt, ~, Te] = conversor_qs(x, wt, pr);
s.we = we;  s.Te = Te;  s.Tt = Tt;
s.Ftr = Tt*ratio/r;

%% 4) Freio (proporcional ao curso, força máxima pelo atrito pneu/rolo)
s.Fb = pr.brk.Fmax*s.pb/100;

%% 5) Dinamômetro
s.Fdyn = pr.dyn.A*(s.v > 0) + pr.dyn.B*s.v + pr.dyn.C*s.v^2 ...
         + pr.veh.m*pr.veh.g*sin(pr.dyn.grade);

%% 6) Dinâmica longitudinal (massa = inércia equivalente simulada, ETW)
m  = pr.veh.m;
Fg = pr.veh.m*pr.veh.g*sin(pr.dyn.grade);
if s.v <= 0
    % parado: só sai do repouso se a força trativa vencer freio + atrito
    Fliq = s.Ftr - s.Fb - pr.dyn.A - Fg;
    s.v  = max(0, dt*Fliq/m);
else
    s.v = max(0, s.v + dt*(s.Ftr - s.Fb - s.Fdyn)/m);
end
s.x = s.x + s.v*dt;
end
