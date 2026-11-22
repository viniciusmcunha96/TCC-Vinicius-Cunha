%% Gera as figuras a partir dos resultados já salvos, sem simular de novo.
%  Requer o arquivo resultados_simulacao.mat criado por main_simulacao.m.
clear; close all;
if ~exist('resultados_simulacao.mat', 'file')
    error('Rode main_simulacao.m primeiro (resultados_simulacao.mat não encontrado).');
end
load('resultados_simulacao.mat', 'L', 'M', 'Lp', 'nomes', 'ciclo');
plotar_resultados(L, M, Lp, nomes, ciclo);
