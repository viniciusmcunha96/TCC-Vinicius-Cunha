# Controle de perfil de velocidade em malha fechada de veículos em dinamômetros de rolos
 
Código de simulação da monografia de conclusão de curso de **Vinícius Miglioli Cunha**
(Engenharia de Controle e Automação, Escola de Engenharia da UFMG, 2026),
orientada pelo Prof. Tiago Coelho Magalhães.
 
O trabalho projeta e avalia, em simulação **MIL (*Model-in-the-Loop*)**, um controlador de velocidade
para um **robô condutor** que atua nos pedais de acelerador e freio de um veículo com transmissão
automática, executando o ciclo **FTP-75** em dinamômetro de rolos. O caso de estudo é um
Fiat Fastback 1.3 turbo com câmbio automático de seis marchas.
 
São comparados dois controladores, com os mesmos ganhos, a mesma alocação de pedais e o mesmo *anti-windup*:
 
| Controlador | Descrição |
|---|---|
| **PI** | PI sintonizado por alocação de polos (ζ = 1, ωn = 5/3 rad/s), com integrador de Tustin e integração condicional |
| **FF + PI** | O mesmo PI somado a uma alimentação direta (*feedforward*) calculada pelo modelo inverso nominal do veículo no dinamômetro |
 
---
 
## Requisitos
 
- **MATLAB R2025a** (versão usada no trabalho: 25.1.0.2973910, *Update* 1). Versões a partir da R2020a devem funcionar (`movmax`/`movmin` e `exportgraphics`).
- Nenhuma *toolbox* é necessária para os *scripts* (`main_simulacao.m`).
- **Simulink** é necessário apenas para a versão em diagrama de blocos (`construir_modelo_simulink.m`, `rodar_modelo_simulink.m` e `robo_condutor_mil.slx`).
## Como usar
 
1. Clone ou baixe o repositório e abra a pasta no MATLAB (ela precisa ser a pasta atual ou estar no *path*).
2. Rode o *script* principal:
```matlab
main_simulacao
```
 
   Ele carrega os parâmetros e o ciclo, simula o FTP-75 completo (1874 s) para os dois controladores,
   imprime a tabela de métricas no *Command Window* (com linhas prontas para LaTeX), executa o ensaio de
   rejeição de perturbação, salva tudo em `resultados_simulacao.mat` e gera as figuras em PNG.
 
3. Para gerar novamente as figuras **sem simular de novo** (a partir do `.mat` salvo):
```matlab
gerar_figuras
```
 
4. Para iterar mais rápido, defina `modo_rapido = true` no início de `main_simulacao.m`: a simulação fica restrita à fase 1 do ciclo (505 s).
### Versão em Simulink (opcional)
 
```matlab
construir_modelo_simulink     % cria robo_condutor_mil.slx a partir das mesmas funções dos scripts
rodar_modelo_simulink         % executa o modelo no FTP-75 e calcula as mesmas métricas
```
 
O modelo usa as mesmas funções de controlador e planta dentro de blocos *MATLAB Function*,
de modo que *script* e diagrama de blocos produzem os mesmos resultados (a menos do gerador de ruído do modelo Simulink).
 
## Estrutura dos arquivos
 
| Arquivo | Função |
|---|---|
| `main_simulacao.m` | *Script* principal: parâmetros, ciclo, simulação dos dois controladores, métricas, ensaio de perturbação, `.mat` e figuras |
| `parametros_sistema.m` | Parâmetros **nominais** (`p`, usados pelo controlador) e da **planta simulada** (`pr`, nominal + incertezas). A origem de cada valor está indicada no comentário da linha |
| `carregar_ciclo.m` | Lê `FTP-75.csv` (1 Hz, mi/h), converte para m/s, interpola linearmente a cada 10 ms e calcula a aceleração de referência |
| `simular_ciclo.m` | Malha fechada controlador + planta ao longo de um ciclo; registra os sinais a 10 Hz |
| `ctrl_init.m`, `ctrl_passo.m` | Inicialização e um período (10 ms) do controlador: lógica de parada, PI, alimentação direta, limites de força, *anti-windup* e alocação de pedais |
| `planta_init.m`, `planta_passo.m` | Estado inicial e um passo da planta: limite de velocidade do robô, mapa do pedal, motor, conversor de torque, câmbio AT6 com lógica de trocas, freio e carga do dinamômetro |
| `conversor_qs.m` | Equilíbrio motor–conversor de torque em regime quase-estático (usado pela planta e, com parâmetros nominais, pelo controlador) |
| `interp_lin.m` | Interpolação linear em tabela com saturação nas pontas (substitui `interp1` nos laços e facilita a portabilidade para C) |
| `metricas_ciclo.m` | Métricas de seguimento (RMSE, MAE, erro máximo), tolerância da EPA (40 CFR 1066.425), índices da SAE J2951 e esforço dos atuadores |
| `cenario_perturbacao.m` | Ensaio de 150 s: partida até 60 km/h, rampa de +4 % entre 40 e 80 s e mudança de referência de 60 para 70 km/h em t = 100 s |
| `plotar_resultados.m` | Gera as figuras do Capítulo 5 da monografia |
| `gerar_figuras.m` | Refaz as figuras a partir de `resultados_simulacao.mat` |
| `construir_modelo_simulink.m`, `rodar_modelo_simulink.m`, `robo_condutor_mil.slx` | Versão em diagrama de blocos (Simulink) |
| `FTP-75.csv` | Traço do FTP-75 (EPA, *Dynamometer Drive Schedules*), colunas `timestamp(s);Speed(mph);Speed(kph)` |
 
## Modelo simulado
 
```
Traço FTP-75 ──► [PI] ──┐
      │                 (+)──► [Alocação de pedais] ──► Robô ──► Trem de força / Freio ──► Veículo no dinamômetro
      └──► [Feedforward]┘          (modelo inverso)                                              │
                 ▲                                                                                │
                 └──────────────────────── velocidade do rolo, rotação do motor, marcha ◄─────────┘
```
 
- **Controlador** (período de 10 ms): calcula a força desejada na roda, F = F_ff + F_PI, e a converte em
  comando de acelerador **ou** de freio pelo modelo inverso nominal da cadeia pedal → força
  (um pedal por vez, sem histerese).
- **Planta**: robô com limite de velocidade de 230,8 %/s; mapa do pedal T_e = T_FL(ω)·√x com perdas de 20 %
  do torque na potência máxima; conversor de torque quase-estático (curvas padrão do bloco *Torque Converter*
  do Simscape Driveline); câmbio de seis marchas com lógica de trocas inspirada no ALPHAshift (EPA);
  freio linear; carga do dinamômetro F = A + Bv + Cv² + mg·sen θ com massa de ensaio de 1417,5 kg.
- **Incertezas** entre o modelo do controlador e a planta: curva de torque de 180 cv (gasolina) na planta
  contra 185 cv (etanol) no controlador, e coeficientes A e C da carga 1 % maiores na planta.
- **Hipóteses simplificadoras**: sem inércia e sem atraso de torque do motor (nem do turbocompressor),
  sem *lock-up* do conversor, trocas instantâneas, freio sem dinâmica hidráulica, servos do robô sem dinâmica
  (apenas limite de velocidade) e medições ideais. Os resultados representam, portanto, o desempenho em
  condições favoráveis.
## Resultados (FTP-75 completo)
 
| Métrica | PI | FF + PI |
|---|---:|---:|
| RMSE (km/h) | 0,238 | 0,020 |
| MAE (km/h) | 0,142 | 0,008 |
| Erro máximo (km/h) | 1,316 | 0,229 |
| Tempo fora da faixa da EPA (s) | 0,0 | 0,0 |
| Excursões > 2 s (EPA) | 0 | 0 |
| RMSSE (mi/h) | 0,148 | 0,012 |
| IWR (%) | 2,10 | 0,17 |
| ER (%) | 0,65 | 0,06 |
| DR (%) | 0,04 | < 0,01 |
| EER (%) | 0,60 | 0,06 |
| ASCR (%) | 1,56 | 0,12 |
| Curso dos pedais (%/km) | 862 | 862 |
| Alternâncias acelerador–freio | 138 | 144 |
 
Os dois controladores mantêm o veículo dentro da faixa de tolerância da EPA durante todo o ciclo.
A alimentação direta reduz o erro de seguimento e os índices da SAE J2951 em cerca de uma ordem de grandeza,
com o mesmo curso acumulado dos pedais.
 
## Etapas seguintes
 
Este código corresponde à etapa MIL do desenvolvimento baseado em modelos. As próximas etapas previstas são
a verificação da geração automática de código do controlador, as simulações SIL e HIL e a construção de um
protótipo com CLP Haiwell e atuadores lineares, seguida da identificação dos parâmetros no veículo real.
 
## Referência
 
CUNHA, V. M. **Controle de perfil de velocidade em malha fechada de veículos em dinamômetros de rolos**.
2026. Monografia (Graduação em Engenharia de Controle e Automação) – Escola de Engenharia,
Universidade Federal de Minas Gerais, Belo Horizonte, 2026.
 
