clear;
clc;

%************ Constants and Variables initialization ************%

n = 10000000;       %simulation lenght

wm_ref = 167;       %rad/s
Tload = 0;          %Nm
B = 3.2e-4;         %Nms
J = 8.7e-4;         %Kgm^2
Rs = 4.7;           %ohms
P = 4;              %polos
P_2 = P/2;          %pares de polos
Ldq = 20e-3;        % Ld = Lq = L-M %%%%CONFIRMAR ISSO%%%%
kt = 0.377;         %V*s/rad
t = 0.000001;       %passo de calculo
c_360 = 2*pi;
phase = -pi/6;      %para corrigir o defasamento entre Vx e Ex, 
                    %pq foi implementado com 30 de avanço.
                    
Kp_wm = 0.00356;    %Proportional Gain
Ki_wm = 5;          %Integral Gain

V_pwm = 10;         %PWM Voltage reference
V_bus = 311;        %Bus Voltage
Fs = 5000;          %Switching frequency of inverter
Ts = 1/Fs;
gain = V_bus/V_pwm; %Gain used to bring V_ref from controller to PWM range

%****************************************************************%

for(T = 1:n)

if (T-1 > 0)
    
    if(T >= 1000000/2)
        Tload = 0.40; 
    end
    
    err_wm(T) = wm_ref - wm(T-1);                   %Proportional error
    err_int_wm(T) = err_int_wm(T-1) + err_wm(T)*t;  %Integral error
    V_ref(T) =  Kp_wm * err_wm(T) + Ki_wm * (0.0537) * err_int_wm(T);       %controller
    
    x(T) = x(T-1)+t;       %X axis to calculate sawtooth wave
    y(T) = Fs*x(T-1);      %Y axis to calculate sawtooth wave
    
    if(x(T) <= Ts)         %PWM generation using Y function and V_ref to check if the switch will be conducting
                           %Result can be ssen in V_barramento
        if(y(T) <= V_ref(T-1)/gain)
            pwm_st(T) = 1;
        else
            pwm_st(T) = 0;
        end
    else
        x(T) = 0;
    end
    
    V_barramento(T) = V_bus * pwm_st(T);
   
    theta_a = theta_e(T-1);                 %electrical angle A, B and C
    theta_b = theta_e(T-1) + (4*c_360)/6;
    theta_c = theta_e(T-1) + (c_360)/3;
    
    ea = F_theta_e(normalize_angle(theta_a));
    eb = F_theta_e(normalize_angle(theta_b));
    ec = F_theta_e(normalize_angle(theta_c));
            
    Ea(T) = kt * wm(T-1) * ea;             %BEMF A, B and C
    Eb(T) = kt * wm(T-1) * eb;
    Ec(T) = kt * wm(T-1) * ec;
    
%%%%% Six-Step Modulation %%%%%
    if(theta_e(T-1) <= 0)
        Va(T) =  0;
        Vb(T) = -V_barramento(T)/2;
        Vc(T) = V_barramento(T)/2;
    elseif(theta_e(T-1) > (pi/6+phase) && theta_e(T-1) <= (pi/2+phase))
        Va(T) =  V_barramento(T)/2;
        Vb(T) = -V_barramento(T)/2;
        Vc(T) = 0;
    elseif(theta_e(T-1) > (pi/2+phase) && theta_e(T-1) <= (5*pi/6+phase))
        Va(T) = V_barramento(T)/2;
        Vb(T) = 0;
        Vc(T) = -V_barramento(T)/2;
    elseif(theta_e(T-1) > (5*pi/6+phase) && theta_e(T-1) <= (7*pi/6+phase))
        Va(T) = 0;
        Vb(T) = V_barramento(T)/2;
        Vc(T) = -V_barramento(T)/2;
    elseif(theta_e(T-1) > (7*pi/6+phase) && theta_e(T-1) <= (3*pi/2+phase))
        Va(T) = -V_barramento(T)/2;
        Vb(T) = V_barramento(T)/2;
        Vc(T) = 0;
    elseif(theta_e(T-1) > (3*pi/2+phase) && theta_e(T-1) <= (33*pi/18+phase))
        Va(T) = -V_barramento(T)/2;
        Vb(T) = 0;
        Vc(T) = V_barramento(T)/2;     
    else
        Va(T) =  0;
        Vb(T) = -V_barramento(T)/2;
        Vc(T) = V_barramento(T)/2;
    end
    
    Ta(T) = kt * Ia(T-1) * ea;
    Tb(T) = kt * Ib(T-1) * eb;
    Tc(T) = kt * Ic(T-1) * ec;
    
    Te(T) = Ta(T) + Tb(T) + Tc(T);
    
    dIa(T) = (Va(T) - Rs * Ia(T-1) - Ea(T))*(t/Ldq);
    dIb(T) = (Vb(T) - Rs * Ib(T-1) - Eb(T))*(t/Ldq);
    dIc(T) = (Vc(T) - Rs * Ic(T-1) - Ec(T))*(t/Ldq);
    
    dwm(T) = (Te(T) - Tload - B*wm(T-1))*(t/J);
    wm(T) = wm(T-1) + dwm(T);
    
    dtheta_m(T) = wm(T) * t;
    
    theta_m(T) = (theta_m(T-1) + dtheta_m(T));
    
    theta_m_norm(T) = normalize_angle(theta_m(T));
     
    theta_e(T) = theta_e(T-1) + P_2 * dtheta_m(T);
    
    theta_e(T) = normalize_angle(theta_e(T));
    
    Ia(T) = Ia(T-1) + dIa(T);
    Ib(T) = Ib(T-1) + dIb(T);
    Ic(T) = Ic(T-1) + dIc(T);
    
time_lapsed(T) = T*t;

else %Variables inicialization
    
    dwm = zeros(n,1);
    wm = zeros(n,1);
    dtheta_m = zeros(n,1);
    theta_m = zeros(n,1);
    theta_e = zeros(n,1);
    theta_m_norm = zeros(n,1);
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
    V_barramento = zeros(n,1);
    V_ref = zeros(n,1);
    err_wm = zeros(n,1);
    err_int_wm = zeros(n,1);
    x = zeros(n,1);
    y = zeros(n,1);
    pwm_st = zeros(n,1);
    V_ref(T) = 311;

end
end

plot(time_lapsed,wm,'color','g');



