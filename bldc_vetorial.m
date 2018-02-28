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
t = 0.000001;        %passo de calculo
c_360 = 2*pi;
phase = -pi/6;      %para corrigir o defasamento entre Vx e Ex,
%pq foi implementado com 30 de avanço.
V_bus = 150;
Fs = 5000;
Ts = 1/Fs;
rst_svm = 1;

Id_ref = 0;
dc_a = 0;
dc_b = 0;
dc_c = 0;

Kp_wm = 11.73;
Ki_wm = 11.73;
Kp_iq = 0.001538;
Ki_iq = 10;
Kp_id = 0.001538;
Ki_id = 10;


%****************************************************************%

for(T = 1:n)
    
    if (T-1 > 0)
         if(T >= 10000000/2)
            V_bus = 311; 
         end
         
        time_lapsed(T) = T*t;
              
        theta_m(T-1) = theta_m(T-1);
        
        theta_a = theta_e(T-1);                 %electrical angle A, B and C
        theta_b = theta_e(T-1) + (4*c_360)/6;
        theta_c = theta_e(T-1) + (c_360)/3;
        
        ea = F_theta_e(normalize_angle(theta_a));
        eb = F_theta_e(normalize_angle(theta_b));
        ec = F_theta_e(normalize_angle(theta_c));
        
        Ea(T) = kt * wm(T-1) * ea;              %BEMF A, B and C
        Eb(T) = kt * wm(T-1) * eb;
        Ec(T) = kt * wm(T-1) * ec;
        
        % Speed Loop %
        err_wm(T) = wm_ref - wm(T-1);                   %Proportional error
        err_int_wm(T) = err_int_wm(T-1) + err_wm(T)*t;  %Integral error
        Iq_ref(T) =  Kp_wm * err_wm(T) + Ki_wm * (0.0537) * err_int_wm(T);       %controller
        
        % Iq Loop %
        err_iq(T) = Iq_ref(T) - Iq(T-1);                   %Proportional error
        err_int_iq(T) = err_int_iq(T-1) + err_iq(T)*t;  %Integral error
        Vq_ref(T) =  Kp_iq * err_iq(T) + Ki_iq * (0.0537) * err_int_iq(T);       %controller
        
        % Id Loop %
        err_id(T) = Id_ref - Id(T-1);                   %Proportional error
        err_int_id(T) = err_int_id(T-1) + err_id(T)*t;  %Integral error
        Vd_ref(T) =  Kp_id * err_id(T) + Ki_id * (0.0537) * err_int_id(T);       %controller
        
        Kabc = [cos(theta_a) -sin(theta_a); 0.5*(-cos(theta_b)+1.73*sin(theta_b)) 0.5*(sin(theta_b)+1.73*cos(theta_b));  0.5*(-cos(theta_b)-1.73*sin(theta_b)) 0.5*(sin(theta_b)-1.73*cos(theta_b))];
        
        dq_to_abc = Kabc * [Vd_ref(T); Vq_ref(T)];
        Va_ref(T) = dq_to_abc(1);
        Vb_ref(T) = dq_to_abc(2);
        Vc_ref(T) = dq_to_abc(3);
        
        %%%%%% SVM Modulation %%%%%% 13/02/18 FONTE: https://www.embedded.com/design/real-world-applications/4441150/2/Painless-MCU-implementation-of-space-vector-modulation-for-electric-motor-systems
        Voltage_Phase = [Va(T-1) Vb(T-1) Vc(T-1)];
        min_V = min(Voltage_Phase);
        max_V = max(Voltage_Phase);
        V_neutral(T) = 0.5 * (max_V + min_V);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        Va(T) = Va_ref(T) * (sin(theta_a)) - V_neutral(T);
        Vb(T) = Vb_ref(T) * (sin(theta_b)) - V_neutral(T);
        Vc(T) = Vc_ref(T) * (sin(theta_c)) - V_neutral(T);
        
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
        
        theta_m(T) = normalize_angle(theta_m(T));
        
        theta_e(T) = theta_e(T-1) + P_2 * dtheta_m(T);
        
        theta_e(T) = normalize_angle(theta_e(T));
        
        Ia(T) = Ia(T-1) + dIa(T);
        Ib(T) = Ib(T-1) + dIb(T);
        Ic(T) = Ic(T-1) + dIc(T);
        
        Kdq = (2/3)*([cos(theta_a) cos(theta_b) cos(theta_c); sin(theta_a) sin(theta_b) sin(theta_c); 0.5 0.5 0.5]);
        
        adc_to_dqo = Kdq*[Ia(T); Ib(T); Ic(T)];
        
        Id(T) = adc_to_dqo(1);
        Iq(T) = adc_to_dqo(2);
        
    else %Variables inicialization
        
        dwm = zeros(n,1);
        wm = zeros(n,1);
        dtheta_m = zeros(n,1);
        theta_m = zeros(n,1);
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
        Id = zeros(n,1);
        Iq = zeros(n,1);
        dIa = zeros(n,1);
        dIb = zeros(n,1);
        dIc = zeros(n,1);
        Ta = zeros(n,1);
        Tb = zeros(n,1);
        Tc = zeros(n,1);
        Te = zeros(n,1);
        time_lapsed = zeros(n,1);
        V_neutral = zeros(n,1);
        Iq_ref = zeros(n,1);
        err_int_wm = zeros(n,1);
        err_wm = zeros(n,1);
        err_int_iq = zeros(n,1);
        err_iq = zeros(n,1);
        err_int_id = zeros(n,1);
        err_id = zeros(n,1);
        Vq_ref = zeros(n,1);
        Vd_ref = zeros(n,1);
        Va_ref = zeros(n,1);
        Vb_ref = zeros(n,1);
        Vc_ref = zeros(n,1);
    end
end

plot(time_lapsed,Id,'color','g');
hold;
plot(time_lapsed,Iq,'color','b');