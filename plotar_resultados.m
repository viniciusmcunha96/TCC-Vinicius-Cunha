function plotar_resultados(L, M, Lp, nomes, ciclo)
%PLOTAR_RESULTADOS  Figuras para a monografia (salvas em PNG).
cores = [0.47 0.47 0.47; 0.00 0.45 0.74];   % PI, FF+PI
nc = numel(nomes);
kmh = 3.6;

%% Fig. 1 — Seguimento no ciclo completo com faixa de tolerância EPA
f1 = figure('Name','Seguimento FTP-75','Color','w','Position',[60 60 1200 800]);
janelas = {[0 ciclo.t1(end)], [0 200], [1360 1520]};
titulos = {'Ciclo FTP-75 completo', 'Detalhe: partida a frio (0–200 s)', ...
           'Detalhe: fim da fase 2 e início da fase 3'};
for w = 1:3
    subplot(3,1,w); hold on; grid on; box on;
    t = L{nc}.t;
    j = t >= janelas{w}(1) & t <= janelas{w}(2);       % só os pontos da janela
    if w == 1, j = j & mod((1:numel(t)).', 5) == 1; end % ciclo completo: 2 Hz basta
    tj = t(j);
    fill([tj; flipud(tj)], [M{nc}.lim_sup(j); flipud(M{nc}.lim_inf(j))]*kmh, [0.9 0.9 0.9], ...
         'EdgeColor','none', 'DisplayName','Tolerância EPA (±2 mph, ±1 s)');
    plot(tj, L{nc}.vref(j)*kmh, 'k--', 'LineWidth', 1.0, 'DisplayName', 'Referência');
    for k = 1:nc
        plot(tj, L{k}.v(j)*kmh, 'Color', cores(k,:), 'LineWidth', 1.0, 'DisplayName', nomes{k});
    end
    xlim(janelas{w}); ylabel('Velocidade (km/h)'); title(titulos{w});
    if w == 1, legend('Location','northeast'); end
end
xlabel('Tempo (s)');
salvar_png(f1, 'fig_seguimento_ftp75.png');

%% Fig. 2 — Erro de velocidade
f2 = figure('Name','Erro','Color','w','Position',[80 80 1200 500]);
hold on; grid on; box on;
for k = 1:nc
    plot(L{k}.t, (L{k}.vref - L{k}.v)*kmh, 'Color', cores(k,:), 'DisplayName', ...
         sprintf('%s (RMSE %.2f km/h)', nomes{k}, M{k}.RMSE));
end
lim = 2*1.609344;
plot(xlim, [lim lim], 'k:', xlim, -[lim lim], 'k:', 'HandleVisibility','off');
xlabel('Tempo (s)'); ylabel('Erro (km/h)'); legend('Location','northeast');
title('Erro de velocidade (linhas pontilhadas: ±2 mph)');
salvar_png(f2, 'fig_erro_ftp75.png');

%% Fig. 3 — Esforço dos atuadores (pedais) — PI vs FF+PI
f3 = figure('Name','Pedais','Color','w','Position',[100 100 1200 700]);
for k = 1:nc
    subplot(nc,1,k); hold on; grid on; box on;
    plot(L{k}.t, L{k}.pa, 'Color', [0.10 0.60 0.30], 'DisplayName', 'Acelerador');
    plot(L{k}.t, -L{k}.pb, 'Color', [0.80 0.15 0.15], 'DisplayName', 'Freio (negativo)');
    xlim([0 200]); ylim([-100 100]); ylabel('Curso (%)');
    title(sprintf('%s — %d trocas acelerador/freio no ciclo', nomes{k}, M{k}.trocas_pedal));
    legend('Location','southeast');
end
xlabel('Tempo (s)');
salvar_png(f3, 'fig_pedais.png');

%% Fig. 4 — Trem de força (FF + PI)
f4 = figure('Name','Trem de forca','Color','w','Position',[120 120 1200 600]);
subplot(3,1,1); plot(L{nc}.t, L{nc}.v*kmh, 'b'); grid on; ylabel('km/h'); xlim([0 505]);
title('FF + PI: velocidade, marcha e rotação (fase 1)');
subplot(3,1,2); stairs(L{nc}.t, L{nc}.marcha, 'k'); grid on; ylabel('Marcha'); ylim([0.5 6.5]); xlim([0 505]);
subplot(3,1,3); plot(L{nc}.t, L{nc}.rpm, 'r'); grid on; ylabel('rpm'); xlabel('Tempo (s)'); xlim([0 505]);
salvar_png(f4, 'fig_trem_forca.png');

%% Fig. 5 — Rejeição de perturbação e degrau de referência
f5 = figure('Name','Perturbacao','Color','w','Position',[140 140 1200 650]);
subplot(2,1,1); hold on; grid on; box on;
plot(Lp{nc}.t, Lp{nc}.vref*kmh, 'k--', 'DisplayName', 'Referência');
for k = 1:nc
    plot(Lp{k}.t, Lp{k}.v*kmh, 'Color', cores(k,:), 'DisplayName', nomes{k});
end
ylabel('Velocidade (km/h)'); legend('Location','southeast');
title('Rampa de +4 % entre 40 e 80 s (dinamômetro) e degrau 60 \rightarrow 70 km/h em 100 s');
subplot(2,1,2); hold on; grid on; box on;
for k = 1:nc
    plot(Lp{k}.t, Lp{k}.F/1000, 'Color', cores(k,:), 'DisplayName', nomes{k});
end
ylabel('Força pedida (kN)'); xlabel('Tempo (s)');
salvar_png(f5, 'fig_perturbacao.png');
end

%% ---------------------------------------------------------------------
function salvar_png(f, arquivo)
% Salva a figura em PNG a 200 dpi. exportgraphics (R2020a+) é bem mais
% rápido que saveas para figuras com muitos pontos; print é o fallback.
drawnow;
if exist('exportgraphics', 'file')
    exportgraphics(f, arquivo, 'Resolution', 200);
else
    print(f, arquivo, '-dpng', '-r200');
end
fprintf('  figura salva: %s\n', arquivo);
end
