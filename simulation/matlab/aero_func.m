function [Fx, Fy, Fz, Mx, My, Mz] = aero_func(u, v, w, m_cur, ...
        rho_air, A_rocket, Cd0, CNalpha, ...
        x_CP, CG_full_x, CG_empty_x, m_full, m_empty)
%#codegen
% 3D 공력 모델  ─  Barrowman 선형화 + 질량에 따른 dx_cp 보간
%
% 입력
%   u,v,w      : 기체축(body) 속도 [m/s]  (x=종축, y=우측, z=아래)
%   m_cur      : 현재 전체 질량 [kg]  ← thrust 블록의 질량 출력 연결
%   rho_air    : 대기 밀도 [kg/m³]
%   A_rocket   : 기준 면적 (몸통 단면) [m²]
%   Cd0        : 축방향 항력 계수
%   CNalpha    : 법선력 기울기 [/rad]
%   x_CP       : CP 위치 (노즈콘 끝 기준) [m]
%   CG_full_x  : 만수 CG 위치 (노즈콘 끝 기준) [m]
%   CG_empty_x : 건조 CG 위치 (노즈콘 끝 기준) [m]
%   m_full/m_empty : 만수/건조 질량 [kg]

% ── 속도 크기 ─────────────────────────────────────────────────────────
V = sqrt(u^2 + v^2 + w^2);

if V < 1e-3
    Fx=0; Fy=0; Fz=0; Mx=0; My=0; Mz=0;
    return
end

% ── 동압 ──────────────────────────────────────────────────────────────
q = 0.5 * rho_air * V^2;

% ── 축방향 항력 (속도 반대 방향) ─────────────────────────────────────
D  = q * A_rocket * Cd0;
Fx = -D * (u/V);
Fy = -D * (v/V);
Fz = -D * (w/V);

% ── 받음각·옆미끄럼각 ────────────────────────────────────────────────
alpha = atan2(w, u);                        % 피치 받음각 [rad]
beta  = asin(max(min(v/V, 1.0), -1.0));    % 요 옆미끄럼각 [rad]

% ── 법선력 (Barrowman 선형화) ─────────────────────────────────────────
N_pitch = q * A_rocket * CNalpha * alpha;
N_yaw   = q * A_rocket * CNalpha * beta;

Fz = Fz + N_pitch;
Fy = Fy + N_yaw;

% ── 현재 질량에 따른 CG 보간 → dx_cp 계산 ───────────────────────────
frac = (m_cur - m_empty) / max(m_full - m_empty, 1e-6);
frac = max(min(frac, 1.0), 0.0);   % 0~1 클램프
CG_cur = CG_empty_x + frac * (CG_full_x - CG_empty_x);
dx_cp  = x_CP - CG_cur;            % 양수 = CP가 CG 뒤 = 안정

% ── 복원 모멘트 ───────────────────────────────────────────────────────
%  피치: 양의 alpha → nose-up → 음의 My (nose-down 복원, dx_cp>0 일 때)
%  요:   양의 beta  → 우측   → 양의 Mz (복원, dx_cp>0 일 때)
My = -N_pitch * dx_cp;
Mz =  N_yaw   * dx_cp;
Mx = 0;   % 롤 모멘트 (핀 캔트 없으면 0)

end
