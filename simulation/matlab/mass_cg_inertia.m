function [m_total, com, I_mat] = mass_cg_inertia(m_water, ...
        m_dry, m_water_0, CG_dry, CG_full, I_dry_vec, I_full_vec)
%#codegen
% 현재 물 질량에 따라 전체 질량·무게중심·관성 보간
% 입력
%   m_water   : 현재 물 질량 [kg] (Integrator 출력)
%   m_dry     : 건조 질량 [kg]
%   m_water_0 : 초기 물 질량 [kg]
%   CG_dry    : 건조 CG (CATIA 글로벌 좌표) [3×1, m]
%   CG_full   : 만수 CG (CATIA 글로벌 좌표) [3×1, m]
%   I_dry_vec : 건조 관성 [6×1, kg·m²] [Ixx;Iyy;Izz;Ixy;Ixz;Iyz]
%   I_full_vec: 만수 관성 [6×1, kg·m²]
% 출력
%   m_total   : 전체 질량 [kg]
%   com       : 현재 CG (body frame) [3×1, m]
%   I_vec     : 현재 관성텐서 [6×1, kg·m²]

frac = max(0.0, min(1.0, m_water / max(m_water_0, 1e-9)));

m_total = m_dry + m_water;
com     = CG_dry + frac * (CG_full - CG_dry);
Iv = I_dry_vec + frac * (I_full_vec - I_dry_vec);
I_mat = [Iv(1), Iv(4), Iv(5); ...
         Iv(4), Iv(2), Iv(6); ...
         Iv(5), Iv(6), Iv(3)];
