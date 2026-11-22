function [we, Tt, Tp, Te] = conversor_qs(x, wt, p)
%CONVERSOR_QS  Motor + conversor de torque em regime quase-estático.
%
%   Dado o curso do acelerador x (0..1) e a rotação da turbina wt (rad/s),
%   encontra a rotação do motor we em que o torque líquido do motor iguala
%   o torque absorvido pela bomba do conversor:
%
%       T_FL(we)*sqrt(x) - T_loss = Tp(we, wt)              (equilíbrio)
%
%   Motor:     Te = T_FL(we)*sqrt(x)               [Sailer et al., 2011, eq. 5]
%              T_loss = 20 % do torque na potência máxima        [Sailer, 2011]
%   Conversor: Tp = (we/K(SR))^2,  Tt = TR(SR)*Tp,  SR = wt/we   [MathWorks]
%              em desaceleração (wt > we) o fluxo se inverte:
%              Tp = -(wt/K(we/wt))^2, Tt = Tp  (modo coast, razão invertida)
%   Marcha lenta: se o motor não sustenta a rotação de marcha lenta, a ECU
%   mantém we = w_idle e fornece o torque que a bomba absorve.
%
%   A inércia do motor é desprezada (hipótese quase-estática), como na
%   abordagem de Sailer et al. (2011), que modela o motor por um mapa estático.
%#codegen

w_idle = p.eng.rpm_idle*pi/30;
w_max  = p.eng.rpm_max*pi/30;
x      = min(max(x, 0), 1);

f_lo = residuo(w_idle, x, wt, p);
if f_lo <= 0
    % marcha lenta governada pela ECU
    we = w_idle;
    [Tp, Tt] = torque_conversor(we, wt, p);
    Te = Tp + p.eng.Tloss;
    return
end
f_hi = residuo(w_max, x, wt, p);
if f_hi >= 0
    we = w_max;                                  % limite superior de rotação
else
    a = w_idle;  b = w_max;                      % bisseção (função monótona)
    for i = 1:40
        c = 0.5*(a + b);
        if residuo(c, x, wt, p) > 0
            a = c;
        else
            b = c;
        end
    end
    we = 0.5*(a + b);
end
[Tp, Tt] = torque_conversor(we, wt, p);
Te = interp_lin(p.eng.rpm_tab, p.eng.Tmax_tab, we*30/pi)*sqrt(x);
end

%% ---------------------------------------------------------------------
function f = residuo(we, x, wt, p)
Te_liq = interp_lin(p.eng.rpm_tab, p.eng.Tmax_tab, we*30/pi)*sqrt(x) - p.eng.Tloss;
Tp = torque_conversor(we, wt, p);
f = Te_liq - Tp;
end

function [Tp, Tt] = torque_conversor(we, wt, p)
if wt <= we
    SR = wt/max(we, 1e-6);
    Tp = interp_lin(p.tc.SR, p.tc.invK2, SR)*we^2;
    Tt = interp_lin(p.tc.SR, p.tc.TR, SR)*Tp;
else
    SRc = we/wt;                                 % razão invertida (coast)
    Tp = -interp_lin(p.tc.SR, p.tc.invK2, SRc)*wt^2;
    Tt = Tp;
end
end
