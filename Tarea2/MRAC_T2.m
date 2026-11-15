% Planta nominal: dx/dt = A*x + B*u
A = [
    -0.0175   0.173   -9.77    -5.63;
    -0.192   -1.09    -0.846   64.6;
     0        0        0        1;
     0.0081  -0.0738   0.0062  -1.9
];

B = [-0.428; 4.91; 0; 4.22];

% Modelo de referencia: dxm/dt = Am*xm + Bm*r
Am = [
    -0.03    0.166     125.6      37.29;
    -0.052  -1.02    -1554.7    -427.82;
     0       0           0         1;
     0.128  -0.0142  -1335.49   -425.12
];

Bm = [-138.1; 1584.2; 0; 1361.6];

eig(A)
eig(Am)

Cm = [1 1 1 1];
Dm = 0;
RefMod=ss(Am,Bm,Cm,Dm)

Q = eye(4);
P = lyap(Am',Q);

step(RefMod)
Kideal = pinv(B)*(Am-A);    
krideal = pinv(B)*Bm;