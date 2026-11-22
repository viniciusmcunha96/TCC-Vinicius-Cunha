function [c, pa_cmd, pb_cmd] = ctrl_passo(c, v_ref, a_ref, v_ref_fut, v, rpm, marcha, p)
%CTRL_PASSO  Um período de amostragem (p.Ts) do controlador.
%
%  Entradas (disponíveis no protótipo):
%    v_ref, a_ref  referência do traço e sua inclinação (m/s, m/s²)
%    v_ref_fut     referência 1 s à frente (o traço é definido a 1 Hz)
%    v             velocidade medida no rolo (m/s)
%    rpm           rotação do motor (OBD-II PID 0x0C)
%    marcha        marcha engatada (rede CAN do câmbio)
%  Saídas: comandos de posição dos pedais (%)
%
%  [lei de controle -> força F]  ->  [alocação de pedais pelo modelo inverso]
%#codegen
Ts    = p.Ts;
m     = p.veh.m;
r     = p.veh.r;
ratio = p.tr.ig(marcha)*p.tr.ifd;

%% 0) Veículo parado e traço parado no próximo segundo -> freio pleno
if v_ref <= 0 && v_ref_fut <= 0 && v <= 0
    c.modo = 2;  c.I = 0;  c.e_ant = 0;  c.F_cmd = 0;
    c.pa = 0;    c.pb = p.brk.hold;
    pa_cmd = c.pa;  pb_cmd = c.pb;
    return
end

%% 1) Lei de controle
e  = v_ref - v;
b  = p.dyn.B + 2*p.dyn.C*v;              % b = dF_din/dv no ponto de operação
K  = 2*c.zeta*c.wn*m - b;                % eq. (2.20) com Kp = 1/b, tau = m/b
Ti = K/(m*c.wn^2);                       % eq. (2.21)
if c.tipo == 2
    F_ff = m*a_ref + p.dyn.A*(v_ref > 0) + p.dyn.B*v_ref + p.dyn.C*v_ref^2;
else
    F_ff = 0;
end
I_novo = c.I + Ts/2*(e + c.e_ant);       % integrador de Tustin
F_des  = F_ff + K*(e + I_novo/Ti);

%% 2) Limites e força sem pedal (modelo nominal)
wt  = v/r*ratio;
we  = rpm*pi/30;
[~, Tt0] = conversor_qs(0, wt, p);       % pedal solto: creep ou freio-motor
F_coast  = Tt0*ratio/r;
if wt <= we
    TR = interp_lin(p.tc.SR, p.tc.TR, wt/max(we, 1e-6));
else
    TR = 1;
end
T_FL  = interp_lin(p.eng.rpm_tab, p.eng.Tmax_tab, rpm);
F_max = max(T_FL - p.eng.Tloss, 0)*TR*ratio/r;
F_min = F_coast - p.brk.Fmax;
F     = min(max(F_des, F_min), max(F_max, F_coast));

% anti-windup por integração condicional
if ~((F_des > F_max && e > 0) || (F_des < F_min && e < 0))
    c.I = I_novo;
end
c.e_ant = e;
c.F_cmd = F;

%% 3) Alocação de pedais (inversa nominal, um pedal por vez)
pa = 0;  pb = 0;
if F > F_coast
    c.modo = 1;
    Tp_req = F*r/ratio/TR;                 % torque na bomba = torque na turbina / TR
    x = ((Tp_req + p.eng.Tloss)/max(T_FL, 1))^2;   % inversa de Te = T_FL*sqrt(x)
    pa = 100*min(max(x, 0), 1);
    if c.ultimo_pedal == -1, c.trocas_pedal = c.trocas_pedal + 1; end
    c.ultimo_pedal = 1;
elseif F < F_coast
    c.modo = -1;
    pb = 100*min((F_coast - F)/p.brk.Fmax, 1);
    if c.ultimo_pedal == 1, c.trocas_pedal = c.trocas_pedal + 1; end
    c.ultimo_pedal = -1;
else
    c.modo = 0;
end
c.pa = pa;  c.pb = pb;
pa_cmd = pa;  pb_cmd = pb;
end
