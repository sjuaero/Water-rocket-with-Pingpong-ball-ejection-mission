clear; clc; close all;

%% 1. 공통 조건
r_star = 0.1;      % 목 반경 (10cm)
r_e = 1.0;         % 출구 반경 (1m) - 강제 조건
L = 2.0;           % 노즐 전체 길이 (2m)
theta_end = 45;    % 원호 종료 각도

%% 2. Part (1): Circular Arc (원호 구간)
% 원의 중심 (0, 2*r_star) 기준 좌표 생성
theta_rad = deg2rad(linspace(-90, -90 + theta_end, 100));
x1 = r_star * cos(theta_rad);
y1 = (2 * r_star) + r_star * sin(theta_rad);

% 연결점 (Connection Point)
x_c = x1(end);
y_c = y1(end);

%% 3. Part (2): Polynomial Fit(이차함수)
% y = Ax^2 + Bx + C
% 조건 1: x_c에서 y_c를 지나야 함 (위치 연속) -> A*x_c^2 + B*x_c + C = y_c
% 조건 2: x = L에서 y = r_e 여야 함 -> A*L^2 + B*L + C = r_e
% 조건 3: x = L에서 dy/dx = 0 이어야 함 -> 2*A*L + B = 0

% 행렬 계산
M = [x_c^2, x_c, 1;
     L^2,   L,   1;
     2*L,   1,   0];
Target = [y_c; r_e; 0];
coeffs = M \ Target;

A = coeffs(1); B = coeffs(2); C = coeffs(3);

x2 = linspace(x_c, L, 200);
y2 = A*x2.^2 + B*x2 + C;

%% 4. Cone Nozzle (비교용)
% alpha = 15도 기준 -> A*X + B = tan(15deg)*x + a
alpha = deg2rad(15);

% 조건 1: straight portion은 연결점을 지나야 함.
a = y_c - tan(alpha) * x_c;

% 조건 2: r_e가 되는 곳까지 L을 재설정해야함.
x_max_straight = (r_e - a)/tan(alpha);

x_cone = [x_c, x_max_straight];
y_cone = [y_c, r_e];

%% 5. 시각화
figure('Color', 'w', 'Position', [200, 200, 900, 500]);
hold on; grid on;

% Bell Nozzle Plot
plot(x1, y1, 'b', 'LineWidth', 2.5, 'DisplayName', 'Bell: Arc');
plot(x2, y2, 'r', 'LineWidth', 2.5, 'DisplayName', 'Bell: Polynomial (Exit R=1m)');
plot(x1, -y1, 'b', 'LineWidth', 2.5, 'HandleVisibility', 'off');
plot(x2, -y2, 'r', 'LineWidth', 2.5, 'HandleVisibility', 'off');

% Straight Cone Plot
plot(x_cone, y_cone, 'k', 'LineWidth', 1.5, 'DisplayName', 'Straight Cone (\alpha=15°)');
plot(x_cone, -y_cone, 'k', 'LineWidth', 1.5, 'HandleVisibility', 'off');

% 설정
xlabel('Length x (m)'); ylabel('Radius y (m)');
title('Nozzle Shape Design');
legend('Location', 'best');
axis equal;
ylim([-1.2 1.2]);

% 결과 출력
fprintf('--- BELL Nozzle 설계 결과 ---\n');
fprintf('미정계수: A = %.4f, B = %.4f, C = %.4f\n', A, B, C);
fprintf('이차함수: f(X) = %.4f X^2 + %.4f X + %.4f\n', A, B, C);

fprintf('--- CONE Nozzle 설계 결과 ---\n');
fprintf('미정계수: A = %.4f, B = %.4f\n', tan(alpha), a);
fprintf('일차함수: f(X) = %.4f X + %.4f\n', tan(alpha), a);
fprintf('x축 형상 총 길이: %.4f\n', x_max_straight);


%% 6. GSD 엑셀 데이터 추출 (CATIA용 - 방법 2: 분리 추출)

% --- 공통 설정 ---
scale = 1000; % m 단위를 mm로 변환

% 1. 공통 원호 구간 (Common Arc for Bell & Straight)
% x1, y1 데이터를 그대로 사용 (Z=0)
X_arc = x1'; Y_arc = y1'; Z_arc = zeros(size(X_arc));
writematrix([X_arc, Y_arc, Z_arc] * scale, 'GSD_Part1_Common_Arc.xlsx');

% 2. Bell Nozzle 전용: 이차함수 구간 (Polynomial Portion)
% 연결점 중복 방지를 위해 x2(2:end) 사용
X_bell_poly = x2(2:end)'; 
Y_bell_poly = y2(2:end)'; 
Z_bell_poly = zeros(size(X_bell_poly));
writematrix([X_bell_poly, Y_bell_poly, Z_bell_poly] * scale, 'GSD_Part2_Bell_Poly.xlsx');

% 3. Straight Cone 전용: 직선 시작점과 끝점 (Two Points for Line)
% 중간점 없이 시작점과 끝점만 있으면 CATIA에서 완벽한 직선 생성이 가능합니다.
X_cone_pts = [x_c; x_max_straight];
Y_cone_pts = [y_c; r_e];
Z_cone_pts = [0; 0];
writematrix([X_cone_pts, Y_cone_pts, Z_cone_pts] * scale, 'GSD_Part2_Straight_Points.xlsx');

fprintf('\n--- CATIA GSD 분리 데이터 저장 완료 ---\n');
fprintf('1. 공통 원호: GSD_Part1_Common_Arc.xlsx\n');
fprintf('2. 벨 이차함수: GSD_Part2_Bell_Poly.xlsx\n');
fprintf('3. 직선 끝점: GSD_Part2_Straight_Points.xlsx\n');

