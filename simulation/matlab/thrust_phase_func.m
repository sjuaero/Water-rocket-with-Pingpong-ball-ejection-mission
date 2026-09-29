function [F, mdot, phase, P_out, T_out] = thrust_phase_func(Ts, ...
        V_bottle, P_i, V_gas_i, T_i, beta_poly, gamma, R, P_atm, ...
        rho_water, C_d_water, C_d_air, A_e)
%#codegen
% 물로켓 추력 + 질량유량 계산
% (기존 2D 스크립트의 phase 1~3 while문 로직을 한 스텝짜리 함수로 이식)
%
% Simulink "MATLAB Function" 블록에 이 코드를 그대로 붙여넣고,
% 입력은 전부 rocket3d_params.m 의 값을 Constant 블록으로 연결하세요.
% (Ts만 샘플타임이고 나머지는 고정 파라미터입니다.)
%
% 출력
%   F     : 노즐 추력 [N]
%   mdot  : 추진제(물+공기) 질량유량 [kg/s] (분출이므로 음수)
%   phase : 현재 단계 (1=물 분출, 2=공기 분출, 3=탄도비행)
%   P_out, T_out : 내부 압력[Pa] / 온도[K] (로깅·디버깅용)

persistent V_gas P T phase_s P_phase2_i T_phase2_i

if isempty(V_gas)
    V_gas      = V_gas_i;
    P          = P_i;
    T          = T_i;
    phase_s    = uint8(1);
    P_phase2_i = P_i;
    T_phase2_i = T_i;
end

F = 0; mdot = 0;

switch phase_s
    case 1   % 물 분출 (폴리트로픽 압축공기 팽창)
        P = P_i * (V_gas_i / V_gas)^beta_poly;
        if P > P_atm
            v_e  = C_d_water * sqrt(2*(P - P_atm)/rho_water);
            F    = 2 * C_d_water^2 * A_e * (P - P_atm);
            mdot = -rho_water * A_e * v_e;
            V_gas = V_gas + A_e * v_e * Ts;
            if V_gas >= V_bottle
                V_gas      = V_bottle;
                phase_s    = uint8(2);
                P_phase2_i = P;
                T_phase2_i = T_i * (P / P_i)^((beta_poly - 1)/beta_poly);
                T = T_phase2_i;
            end
        else
            phase_s = uint8(2);
        end

    case 2   % 공기 분출 (단열 팽창, choked/subsonic 판정)
        if P > P_atm * 1.01
            T = T_phase2_i * (P / P_phase2_i)^((gamma - 1)/gamma);
            P_crit = 1.893 * P_atm;
            if P >= P_crit
                P_exit   = P * (2/(gamma+1))^(gamma/(gamma-1));
                T_exit   = T * (2/(gamma+1));
                v_exit   = sqrt(gamma * R * T_exit);
                rho_exit = P_exit / (R * T_exit);
                m_mag    = C_d_air * A_e * rho_exit * v_exit;
                F        = m_mag * v_exit + (P_exit - P_atm) * A_e;
                dP_dt    = -(C_d_air*A_e*gamma*P/V_bottle) * ...
                            sqrt(gamma*R*T*(2/(gamma+1))^((gamma+1)/(gamma-1)));
            else
                P_exit   = P_atm;
                v_exit   = sqrt((2*gamma*R*T/(gamma-1)) * (1 - (P_atm/P)^((gamma-1)/gamma)));
                T_exit   = T * (P_atm/P)^((gamma-1)/gamma);
                rho_exit = P_atm / (R * T_exit);
                m_mag    = C_d_air * A_e * rho_exit * v_exit;
                F        = m_mag * v_exit;
                term1 = (P_atm/P)^(2/gamma);
                term2 = (P_atm/P)^((gamma+1)/gamma);
                if term1 > term2
                    dP_dt = -(C_d_air*A_e*gamma*P/V_bottle) * ...
                             sqrt((2*gamma*R*T/(gamma-1)) * (term1 - term2));
                else
                    dP_dt = 0;
                end
            end
            mdot = -m_mag;
            P = P + dP_dt * Ts;
        else
            phase_s = uint8(3);
            P = P_atm;
        end

    case 3   % 탄도비행 (추력 없음)
        P = P_atm; F = 0; mdot = 0;
end

phase = double(phase_s);
P_out = P;
T_out = T;
end
