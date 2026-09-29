%% rocket3d_params.m  ─  Simscape Multibody 물로켓 시뮬레이션 파라미터
% Model Settings > Callbacks > InitFcn 에  run('rocket3d_params.m')  등록

%% ── 추진/열역학 ──────────────────────────────────────────────────────
d_e         = 0.021;
d_rocket    = 0.092;
A_e         = pi*(d_e/2)^2;
A_rocket    = pi*(d_rocket/2)^2;
C_d_water   = 0.85;
C_d_air     = 0.90;
C_D_drag    = 0.25;
T_celsius   = 25;
T_i         = T_celsius + 273.15;
P_atm       = 101325;
rho_air_ext = 1.225;
rho_water   = 1000;
g_acc       = 9.81;
R_gas       = 287.05;
gamma_gas   = 1.4;
beta_poly   = 1.31;
V_bottle    = 1.5e-3;
vol_water_L = 0.58;
V_water_i   = vol_water_L * 1e-3;
V_gas_i     = V_bottle - V_water_i;
P_i         = 50*6894.75 + P_atm;
m_air_init  = (P_i * V_gas_i)/(R_gas * T_i);
Ts          = 0.001;

%% ── 발사 조건 ──────────────────────────────────────────────────────
theta_deg   = 50;                         % 수평면에서 발사각 [deg]
rail_length = 1.0;                        % 발사대 레일 길이 [m]

%% ── 질량 ────────────────────────────────────────────────────────────
m_dry_i     = 0.269;                      % 실측 건조질량 [kg]
m_water_0   = vol_water_L * rho_water * V_water_i;  % 초기 물 질량 [kg]
% 더 정확하게: m_water_0 = 0.58 kg
m_water_0   = 0.580;
m_full      = m_dry_i + m_water_0;       % 0.849 kg
m_empty     = m_dry_i;                   % 0.269 kg

%% ── CG (CATIA 글로벌 좌표, Y = 동체축) [m] ─────────────────────────
%  CATIA 측정값 그대로 사용 (Simscape STEP import가 같은 좌표계 유지)
CG_dry_body  = [0.01289;  0.01855; -0.00576];   % [Gx;Gy;Gz] 건조
CG_full_body = [0.01345; -0.10517; -0.00608];   % [Gx;Gy;Gz] 만수

% Y 좌표만 따로 (모멘트 계산용)
CG_dry_Y     =  0.01855;    % [m]
CG_full_Y    = -0.10517;    % [m]

%% ── CP (CATIA Gy 좌표) ──────────────────────────────────────────────
%  노즈콘 끝 Gy = 160.119mm, x_CP = 255mm from nosecone tip
Y_CP_body    = 0.160119 - 0.255;   % = -0.09488 m

%% ── 관성텐서 [6×1: Ixx;Iyy;Izz;Ixy;Ixz;Iyz] (CATIA 좌표 = Simscape 좌표) ─
%  CATIA Y = 동체축 (롤축) → Iyy가 가장 작음
I_dry_vec  = [3.0e-3;  5.071e-4; 3.0e-3;  6.928e-7;  2.156e-6; -2.712e-6];
I_full_vec = [1.1e-2;  1.0e-3;   1.1e-2;  3.14e-5;   2.234e-6; -1.994e-5];

%% ── Barrowman 공력 ──────────────────────────────────────────────────
CNalpha     = 6.07;    % [/rad]

%% ── 발사 초기 자세 (Rigid Transform) ────────────────────────────────
%  body Y → world [cos50°; 0; sin50°] (XZ 평면, 50° 앙각)
%  Rigid Transform 회전행렬 (Rotation Matrix 방식으로 입력):
theta_rad = theta_deg * pi/180;
%  R_init = Ry(90°-theta_deg) * Rx(90°) 조합
%  → body Y가 world [sin(theta);0;cos(theta)] 방향을 가리키도록 설정
%  아래 값은 Simulink Rigid Transform "Rotation Matrix" 칸에 직접 입력
R_init = [cos(pi/2 - theta_rad),  sin(pi/2 - theta_rad), 0; ...
          0,                        0,                     -1; ...
         -sin(pi/2 - theta_rad),  cos(pi/2 - theta_rad),  0];
% 검증: R_init * [0;1;0] = [sin(50°);0;cos(50°)] = 발사 방향
launch_dir = R_init * [0;1;0];

fprintf('[rocket3d_params] 파라미터 로드 완료\n');
fprintf('  m_full=%.3f kg, m_empty=%.3f kg\n', m_full, m_empty);
fprintf('  발사 방향(world XZ): [%.3f, %.3f, %.3f]\n', launch_dir);
fprintf('  Y_CP_body=%.4f m, CG_dry_Y=%.4f m, CG_full_Y=%.4f m\n', ...
        Y_CP_body, CG_dry_Y, CG_full_Y);
