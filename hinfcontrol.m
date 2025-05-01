clc;
clear;

% Step 1: Define symbolic variables
syms h mhe m Dh The g R Ratm Patm Tatm rhoatm fh real

% Step 2: Assign constants
g = 9.80;
R = 2077;
Ratm = 287;
The_nominal = 235; % Kelvin (nominal temperature)
m = 484;  % kg

% Step 3: Define dependent symbolic expressions
Patm = 101325 * ((288.15 - 0.0065 * h) / 288.15)^5.25;
Tatm = 288.15 - 0.0065 * h;
rhoatm = Patm/(Ratm*Tatm);

% Consider The as a noise/disturbance variable
fh = (rhoatm * g * mhe * R * The / Patm - m * g - Dh) / ...
     (m + 0.5 * rhoatm * mhe * R * The / Patm);

% Step 4: Differentiate fh w.r.t h, mhe, Dh, and The
dfh_dh = diff(fh, h);
dfh_dmhe = diff(fh, mhe);
dfh_dDh = diff(fh, Dh);
dfh_dThe = diff(fh, The);

% Step 5: Substitute numerical values for linearization point
h_val = 20000;
mhe_val = 66.36;
Dh_val = 0;
The_val = The_nominal;

dfh_dh_value = double(subs(dfh_dh, [h, mhe, Dh, The], [h_val, mhe_val, Dh_val, The_val]));
dfh_dmhe_value = double(subs(dfh_dmhe, [h, mhe, Dh, The], [h_val, mhe_val, Dh_val, The_val]));
dfh_dDh_value = double(subs(dfh_dDh, [h, mhe, Dh, The], [h_val, mhe_val, Dh_val, The_val]));
dfh_dThe_value = double(subs(dfh_dThe, [h, mhe, Dh, The], [h_val, mhe_val, Dh_val, The_val]));

% Display linearized derivatives
disp('dfh/dh:'); disp(dfh_dh_value);
disp('dfh/dmhe:'); disp(dfh_dmhe_value);
disp('dfh/dDh:'); disp(dfh_dDh_value);
disp('dfh/dThe:'); disp(dfh_dThe_value);

% Step 6: Form state-space matrices with The as disturbance
A = [0 1; dfh_dh_value 0];
B1 = [0 0; dfh_dDh_value dfh_dThe_value]; % Disturbance inputs [Dh; The]
B2 = [0; dfh_dmhe_value];
C1 = [1 0]; 
C2 = [1 1]; 
D11 = zeros(1,2);
D12 = 0;
D21 = zeros(1,2);
D22 = 0;

% Step 7: Create state-space model for H-infinity synthesis
G = ss(A, [B1 B2], [C1; C2], [D11 D12; D21 D22]);

% Step 8: H-infinity synthesis
nmeas = 2; % number of measurements (h, doth)
ncon = 1; % number of control inputs (mhe)
[K, CL, gamma] = hinfsyn(G, nmeas, ncon);

disp(['Achieved H-infinity norm (gamma): ', num2str(gamma)]);
disp('H-infinity Controller:');
tf(K)