clear;
clc;


%***** Constants and Variables initialization *****%
n = 10000;

Tload = 0;
V_barramento = 200;
B = 3.2e-4;
J = 8.7e-4;
Rs = 4.7;
P = 4;         %polos
P_2 = P/2;     %pares de polos
Ldq = 20e-3;        % Ld = Lq = L-M %%%%CONFIRMAR ISSO%%%%
ke = 0.377;
ke_2 = ke/2;
kt = 0.377;
t = 0.0001;    %passo de calculo

dwm = 0;
wm = 0;
dtheta_m = 0;
theta_m = 0;
theta_e = 0;
dt = 0;

Ea = zeros(n,1);
Eb = zeros(n,1);
Ec = zeros(n,1);
Va = zeros(n,1);
Vb = zeros(n,1);
Vc = zeros(n,1);
Ia = zeros(n,1);
Ib = zeros(n,1);
Ic = zeros(n,1);
dIa = zeros(n,1);
dIb = zeros(n,1);
dIc = zeros(n,1);
Ta = zeros(n,1);
Tb = zeros(n,1);
Tc = zeros(n,1);
Te = zeros(n,1);
time_lapsed = zeros(n,1);

%**************************************************%
for(T = 1:n),

if (T-1 ~= 0),
    
    
    dwm = (Te(T) - Tload -B*wm)*(dt/J);
    dtheta_m = wm * dt;
    wm = wm + dwm;
    theta_m = theta_m + dtheta_m;
    theta_e = (P_2)*theta_m;
    
    if(theta_e >= 360)
        theta_e = theta_e - 360;
    elseif(theta_e < 0)
        theta_e = theta_e + 360;
    end
    
    Ea(T) = ke_2 * wm * F_theta_e(theta_e);
    Eb(T) = ke_2 * wm * F_theta_e(theta_e - 120);
    %Ec(T) = ke_2 * wm(T) * F_theta_e(theta_e + 120);
    
    Va(T) = V_barramento * F_theta_e(theta_e);
    Vb(T) = V_barramento * F_theta_e(theta_e - 120);
    %Vc(T) = V_barramento * F_theta_e(theta_e + 120);
    
    dIa(T) = (Va(T) - Rs * Ia(T) - Ea(T))*(dt/Ldq);
    dIb(T) = (Vb(T) - Rs * Ib(T) - Eb(T))*(dt/Ldq);
    %dIc(T) = (Vc(T) - Rs * Ic(T) - Ec(T))*(dt/Ldq);
    
    Ia(T) = Ia(T-1) + dIa(T);
    Ib(T) = Ib(T-1) + dIb(T);
    %Ic(T) = Ic(T-1) + dIc(T);
    

Ta(T) = kt * Ia(T) * F_theta_e(theta_e);
Tb(T) = kt * Ib(T) * F_theta_e(theta_e - 120);
%Tc(T) = kt * Ic(T) * F_theta_e(theta_e + 120);
%Te(T) = Ta(T) + Tb(T) + Tc(T);

dt = (dt + t)*dt0;
time_lapsed(T) = T*t;
    
else
    
    Ea(T) = 0;
    Eb(T) = 0;
    Ec(T) = 0;
    Va(T) = 0;
    Vb(T) = 0;
    Vc(T) = 0;
    dIa(T) = 0;
    dIb(T) = 0;
    dIc(T) = 0;
    Ia(T) = 0;
    Ib(T) = 0;
    Ic(T) = 0;
    dwm(T) = 0;
    wm(T) = 0;
        

end
end

plot(time_lapsed,Va);