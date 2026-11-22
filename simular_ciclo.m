function L = simular_ciclo(p, pr, ciclo, tipo, grade)
%SIMULAR_CICLO  Malha fechada controlador (Ts) + planta (dt) sobre um ciclo.
%   grade (opcional): vetor de inclinação simulada pelo dinamômetro (rad),
%   amostrado em ciclo.t — usado no ensaio de rejeição de perturbação.
if nargin < 5 || isempty(grade), grade = zeros(ciclo.N, 1); end

n_sub = round(p.Ts/p.dt);
n_log = round(p.Ts_log/p.Ts);
n_fut = round(1.0/p.Ts);                 % 1 s à frente (parada/partida)
N     = ciclo.N;
M     = floor((N-1)/n_log) + 1;

campos = {'t','vref','v','pa','pb','F','Ftr','Fb','rpm','marcha','grade'};
for i = 1:numel(campos), L.(campos{i}) = zeros(M,1); end

s = planta_init(pr, ciclo.v(1));
c = ctrl_init(p, tipo);
pa_cmd = 0; pb_cmd = p.brk.hold; j = 0;

for k = 1:N
    % ---- medições (velocidade do rolo, rpm e marcha), amostradas em Ts ----
    v_med = s.v;
    rpm   = s.we*30/pi;

    % ---- controlador ----
    kf = min(k + n_fut, N);
    [c, pa_cmd, pb_cmd] = ctrl_passo(c, ciclo.v(k), ciclo.a(k), ciclo.v(kf), ...
                                     v_med, rpm, s.marcha, p);
    % ---- planta (n_sub subpassos) ----
    pr.dyn.grade = grade(k);
    for i = 1:n_sub
        s = planta_passo(s, pa_cmd, pb_cmd, pr);
    end
    % ---- registro a 10 Hz ----
    if mod(k-1, n_log) == 0
        j = j + 1;
        L.t(j) = ciclo.t(k);  L.vref(j) = ciclo.v(min(k+1,N));  L.v(j) = s.v;
        L.pa(j) = s.pa;  L.pb(j) = s.pb;  L.F(j) = c.F_cmd;
        L.Ftr(j) = s.Ftr; L.Fb(j) = s.Fb; L.rpm(j) = s.we*30/pi;
        L.marcha(j) = s.marcha;  L.grade(j) = grade(k);
    end
end
% referência no instante do registro (o estado foi integrado até t+Ts)
L.vref = interp1(ciclo.t, ciclo.v, min(L.t + p.Ts, ciclo.t(end)));
L.t    = L.t + p.Ts;
L.trocas_pedal = c.trocas_pedal;
L.tipo = tipo;
end
