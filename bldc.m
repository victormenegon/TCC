clear;
clc;

%***** Constants and Variables initialization *****%

n = 100000;

Tload = 0.1;
V_barramento = 200;
B = 3.2e-4;
J = 8.7e-4;
Rs = 4.7;
P = 4;         %polos
P_2 = P/2;     %pares de polos
Ldq = 20e-3;   % Ld = Lq = L-M %%%%CONFIRMAR ISSO%%%%
ke = 0.377;
ke_2 = ke/2;
kt = 0.377;
t = 0.00001;    %passo de calculo
c_360 = 2*pi;
rst_dt = 1;


%**************************************************%

for(T = 1:n), 

if (T-1 > 0),
    
    dt(T) = (dt(T-1) + t)*rst_dt;
    
    dwm(T) = (Te(T) - Tload - B*wm(T-1))*(dt(T)/J);
    wm(T) = wm(T-1) + dwm(T);
    dtheta_m(T) = wm(T) * dt(T);
    theta_m(T) = (theta_m(T) + dtheta_m(T))*rst_dt;
    theta_e(T) = P_2 * theta_m(T);
    
     if(theta_e(T) >= c_360),
         theta_e(T) = theta_e(T) - c_360;
     elseif(theta_e(T) < 0)
         theta_e(T) = theta_e(T) + c_360;
     end
    
   
    ea = F_theta_e(theta_e(T));
    eb = F_theta_e(theta_e(T) - (c_360)/3);
    ec = F_theta_e(theta_e(T) + (c_360)/3);
    
    bla(T) = ea;
    
    Ea(T) = ke_2 * wm(T) * ea;
    Eb(T) = ke_2 * wm(T) * eb;
    Ec(T) = ke_2 * wm(T) * ec;
    
    Va(T) = V_barramento * ea;
    Vb(T) = V_barramento * eb;
    Vc(T) = V_barramento * ec;

    Ta(T) = kt * Ia(T) * ea;
    Tb(T) = kt * Ib(T) * eb;
    Tc(T) = kt * Ic(T) * ec;
    Te(T) = Ta(T) + Tb(T) + Tc(T);
   
    
    dIa(T) = (Va(T) - Rs * Ia(T) - Ea(T))*(dt(T)/Ldq);
    dIb(T) = (Vb(T) - Rs * Ib(T) - Eb(T))*(dt(T)/Ldq);
    dIc(T) = (Vc(T) - Rs * Ic(T) - Ec(T))*(dt(T)/Ldq);
    
    Ia(T) = Ia(T-1) + dIa(T);
    Ib(T) = Ib(T-1) + dIb(T);
    Ic(T) = Ic(T-1) + dIc(T);

    if(theta_m(T) > c_360 | theta_m(T) < -2*pi),
        rst_dt = 0;
    else
        rst_dt = 1;
    end
        

time_lapsed(T) = T*t;
    
else
    
    dwm = zeros(n,1);
    wm = zeros(n,1);
    dtheta_m = zeros(n,1);
    theta_m = zeros(n,1);
    theta_e = zeros(n,1);
    dt = zeros(n,1);
   
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
    bla = zeros(n,1);
    

end
end

plot(time_lapsed*c_360,Va,'color','g');
hold;

plot(time_lapsed*c_360,Vb,'color','b');
hold;

plot(time_lapsed*c_360,Vc,'color','r');


