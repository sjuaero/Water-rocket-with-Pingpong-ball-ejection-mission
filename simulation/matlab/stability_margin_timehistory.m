%% stability_margin_timehistory.m
% 기존 thrust_phase_func.m, mass_cg_inertia.m을 그대로 재사용해서
% 추력 단계 ~ 건조비행 전환 구간의 CG(t) vs 고정 CP 안정여유를 계산.
% Simscape/Simulink 모델은 전혀 건드리지 않는 독립 스크립트입니다.
%
% 전제: rocket3d_params.m, thrust_phase_func.m, mass_cg_inertia.m이
%       모두 같은 폴더(또는 path)에 있어야 합니다.

clear thrust_phase_func   % persistent 변수(가스체적/압력/온도/phase) 초기화 — 필수
run('rocket3d_params.m')  % 기존 파라미터 전부 로드 (V_bottle, P_i, gamma_gas, R_gas 등)

%% 안정성 분석 전용 상수
x_cp   = 255;        % Barrowman CP, 노즈콘 끝 기준 [mm]
d_ref  = 92;          % 몸통 직경 [mm]
Y_nose = 160.119;     % 노즈콘 끝의 CATIA 글로벌 Y좌표 [mm]
Ts     = 0.001;       % 적분 타임스텝 [s]
t_sim  = 1.0;         % 충분히 길게 — 탄도비행 진입까지 보고 싶은 시간

%% 시간 루프 (Simulink Integrator 블록을 수동으로 대체)
N = round(t_sim/Ts);
t_log       = zeros(N,1);
m_water_log = zeros(N,1);
phase_log   = zeros(N,1);
cg_log      = zeros(N,1);
margin_log  = zeros(N,1);

m_water = m_water_0;   % 초기 물 질량 [kg]

for k = 1:N
    t_log(k) = (k-1)*Ts;

    % thrust_phase_func은 persistent 내부상태로 phase 1~3을 알아서 관리함.
    % V_gas_i, P_i, T_i는 최초 호출(k=1)에서만 실제로 쓰이고 이후엔 무시됨.
    [~, mdot, phase, ~, ~] = thrust_phase_func(Ts, ...
        V_bottle, P_i, V_gas_i, T_i, beta_poly, gamma_gas, R_gas, P_atm, ...
        rho_water, C_d_water, C_d_air, A_e);

    % mdot 적분 -> m_water(t)  (Simulink Integrator + Saturation 역할)
    m_water = max(0, min(m_water_0, m_water + mdot*Ts));

    % 현재 m_water로 CG 보간
    [~, com, ~] = mass_cg_inertia(m_water, m_dry_i, m_water_0, ...
        CG_dry_body, CG_full_body, I_dry_vec, I_full_vec);

    % CATIA Y좌표 -> 노즈콘 끝 기준 거리로 변환 (부호 주의)
    x_cg = Y_nose - com(2)*1000;   % com은 m 단위이므로 *1000 -> mm

    m_water_log(k) = m_water;
    phase_log(k)   = phase;
    cg_log(k)      = x_cg;
    margin_log(k)  = (x_cp - x_cg) / d_ref;
end

%% 시각화
figure;

subplot(3,1,1)
plot(t_log, m_water_log*1000, 'LineWidth', 1.3);
ylabel('물 질량 [g]'); grid on; title('물 질량 / CG / 안정여유 시간 이력');

subplot(3,1,2)
plot(t_log, cg_log, 'b-', 'LineWidth', 1.5); hold on
yline(x_cp, 'r--', 'CP (고정, 255mm)');
ylabel('위치 [mm, 노즈콘 끝 기준]'); legend('CG(t)','CP'); grid on

subplot(3,1,3)
plot(t_log, margin_log, 'k-', 'LineWidth', 1.5); hold on
yline(0, 'r--');
xlabel('시간 [s]'); ylabel('안정 여유 [cal]'); grid on

% 추력 종료 시점(phase 1->2 전환) 표시
idx_burnout = find(phase_log >= 2, 1, 'first');
if ~isempty(idx_burnout)
    subplot(3,1,3)
    xline(t_log(idx_burnout), 'g--', '물 소진');
end
