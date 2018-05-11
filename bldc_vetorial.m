close all;
clear all;
clc;

%************ Constants and Variables initialization ************%

n = 1000000;       %simulation lenght

wm_ref = 167;       %rad/s
Tload = 0.05;          %Nm
B = 2.8142e-4;         %Nms
J = 8.7e-4;         %Kgm^2
Rs = 4.65;           %ohms
P = 4;              %polos
P_2 = P/2;          %pares de polos
Ldq = 67.6e-3;        % Ld = Lq = L-M %%%%CONFIRMAR ISSO%%%%
kt = 0.359;         %V*s/rad
t = 0.0001;       %passo de calculo
c_360 = 2*pi;
V_bus = 150;

Id_ref = 0;
dc_a = 0;
dc_b = 0;
dc_c = 0;

Kp_wm = 1.0759;%(2*damp1*SG*J)/(P_2*P_2*kt);
Ki_wm = 955;%(SG*SG*J)/(P_2*P_2*kt);
Kp_Idq = 295;
Ki_Idq = 666933;

%****************************************************************%

for(T = 1:n)
    
    if (T-1 > 0)

    if(T >= 100000/2)
            Tload = 0.05;
    end
       
        err_wm(T) = wm_ref - wm(T-1);                                 %Proportional error
        err_int_wm(T) = err_int_wm(T-1) + err_wm(T)*t;                %Integral error
        Iq_ref(T) = 0.5;%Kp_wm * err_wm(T) + Ki_wm * err_int_wm(T);       %controller
        
        err_Iq(T) = Iq_ref(T) - Iq(T-1);
        err_int_Iq(T) = err_int_Iq(T-1) + err_Iq(T)*t;
        Vq_ref(T) = Kp_Idq * err_Iq(T) + Ki_Idq*err_int_Iq(T);% + Ldq*wm(T-1)*Id(T-1) + wm(T-1)*kt;
        
        err_Id(T) = Id_ref(T) - Id(T-1);
        err_int_Id(T) = err_int_Id(T-1) + err_Id(T)*t;
        Vd_ref(T) = Kp_Idq * err_Id(T) + Ki_Idq*err_int_Id(T);% - Ldq*wm(T-1)*Iq(T-1);

        time_lapsed(T) = T*t;
        
        %Krishnan p.237 Permanent Magnet Synchronous Machines
        Ks = [cos(theta_a(T-1)) sin(theta_a(T-1));cos(theta_b(T-1)) sin(theta_b(T-1)); cos(theta_c(T-1)) sin(theta_c(T-1))];
        dq0_to_adc = Ks*[Vq_ref(T);Vd_ref(T)];

        Va_a(T) = dq0_to_adc(1);
        Vb_a(T) = dq0_to_adc(2);
        Vc_a(T) = dq0_to_adc(3);
        
        ea(T) = F_theta_e(theta_a(T-1));
        eb(T) = F_theta_e(theta_b(T-1));
        ec(T) = F_theta_e(theta_c(T-1));

        Ea(T) = kt * wm(T-1) * sin(theta_a(T-1));%ea(T);              %BEMF A, B and C
        Eb(T) = kt * wm(T-1) * sin(theta_b(T-1));%eb(T);
        Ec(T) = kt * wm(T-1) * sin(theta_c(T-1));%ec(T);

        %%%%%% SVM Modulation %%%%%% 13/02/18 FONTE: https://www.embedded.com/design/real-world-applications/4441150/2/Painless-MCU-implementation-of-space-vector-modulation-for-electric-motor-systems
%         Voltage_Phase = [Va_a(T) Vb_a(T) Vc_a(T)];    
%         min_V = min(Voltage_Phase);
%         max_V = max(Voltage_Phase);
%         V_neutral(T) = 0.5 * (max_V + min_V);
               
        Va(T) = Va_a(T);% - V_neutral(T);
        Vb(T) = Vb_a(T);% - V_neutral(T);
        Vc(T) = Vc_a(T);% - V_neutral(T);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        dIa(T) = (Va(T) - Rs * Ia(T-1) - Ea(T))*(t/Ldq);
        dIb(T) = (Vb(T) - Rs * Ib(T-1) - Eb(T))*(t/Ldq);
        dIc(T) = (Vc(T) - Rs * Ic(T-1) - Ec(T))*(t/Ldq);
               
        Ia(T) = Ia(T-1) + dIa(T);
        Ib(T) = Ib(T-1) + dIb(T);
        Ic(T) = Ic(T-1) + dIc(T);
        
        Ta(T) = kt * Ia(T) * ea(T);
        Tb(T) = kt * Ib(T) * eb(T);
        Tc(T) = kt * Ic(T) * ec(T);
        Te(T) = (Ta(T) + Tb(T) + Tc(T));
                  
        dwm(T) = (Te(T) - Tload - B*wm(T-1))*(t/J);
        wm(T) = wm(T-1) + dwm(T);
        dtheta_m(T) = wm(T) * t;
        theta_m(T) = normalize_angle(theta_m(T-1) + dtheta_m(T));
        theta_e(T) = normalize_angle(theta_e(T-1) + P_2 * dtheta_m(T));
        
        theta_a(T) = theta_e(T); %electrical angle A, B and C
        theta_b(T) = normalize_angle(theta_a(T) + (4*c_360)/6);
        theta_c(T) = normalize_angle(theta_a(T) + (c_360)/3);

        Ks_inv = (2/3)*([cos(theta_a(T)) cos(theta_b(T)) cos(theta_c(T)); sin(theta_a(T)) sin(theta_b(T)) sin(theta_c(T))]);
        adc_to_dqo = Ks_inv*[Ia(T); Ib(T); Ic(T)];

        Iq(T) = adc_to_dqo(1);
        Id(T) = adc_to_dqo(2);

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
        Id_ref = zeros(n,1);
        Iq_ref = zeros(n,1);
        Vd_ref = zeros(n,1);
        Vq_ref = zeros(n,1);
        err_wm = zeros(n,1);
        err_int_wm = zeros(n,1);
        err_Id = zeros(n,1);
        err_int_Id = zeros(n,1);
        err_Iq = zeros(n,1);
        err_int_Iq = zeros(n,1);
        V_alfa = zeros(n,1);
        V_beta = zeros(n,1);
        Va_a = zeros(n,1);
        Vb_a = zeros(n,1);
        Vc_a = zeros(n,1);
        ea = zeros(n,1);
        eb = zeros(n,1);
        ec = zeros(n,1);
        theta_a = zeros(n,1);
        theta_b = zeros(n,1);
        theta_c = zeros(n,1);

    end
end

subplot(3,1,1);
plot(time_lapsed,Ea,'color','g');
hold;
plot(time_lapsed,Va,'color','b');
subplot(3,1,2);
plot(time_lapsed,wm,'color','r');
subplot(3,1,3);
plot(time_lapsed,Te,'color','y');
