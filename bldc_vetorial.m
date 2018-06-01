close all;
clear all;
clc;

%************ Constants and Variables initialization ************%

n = 1000000;           %simulation lenght

Tload = 0.1;           %Nm
B = 2.8142e-4;         %Nms
J = 8.7e-4;            %Kgm^2
Rs = 4.65;             %ohms
P = 4;                 %polos
P_2 = P/2;             %pares de polos
Ldq = 67.6e-3;         % Ld = Lq = L-M %%%%CONFIRMAR ISSO%%%%
kt = 0.359;            %V*s/rad
t = 0.00001;           %passo de calculo
c_360 = 2*pi;

a = 0;
b = 0;
c = 0;
freq = 0;
V = 0;

Kp_wm = 0.0969;
Ki_wm = 0.12117;
Kp_Idq = 10.14;
Ki_Idq = 697;

%****************************************************************%

for(T = 1:n)
    
    if (T-1 > 0)
        
        if(T >= 1000000/2)
            Tload = 0.1;
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        if(T < 400000)
            a = a+1;
            if(a == 4000)
                a = 0;
                V = V + 0.1;
                freq = freq + 0.2;
            end
            
        elseif(T > 400000)
            c = c+1;
            wm_r(T) = 167;
            if(c == 200)
                c = 0;
                wm_ref(T) = 0.9975031210986267166*wm_ref(T-1)+0.0012484394506866417*(wm_r(T)+wm_r(T-1)); %Pre-Filtro
                err_wm(T) = wm_ref(T) - wm(T-1); %Erro
                Iq_ref(T) = Iq_ref(T-1) + 0.09702117*err_wm(T) - 0.09677883*err_wm(T-1); %eq. de diferenças wm
            else
                wm_ref(T) = wm_ref(T-1);
                err_wm(T) = err_wm(T-1);
                Iq_ref(T) = Iq_ref(T-1);
            end
            b = b+1;
            if(b == 20)
                b = 0;
                err_Iq(T) = Iq_ref(T) - Iq(T-1);
                err_Id(T) = Id_ref(T) - Id(T-1);
                Vq_ref(T) = Vq_ref(T-1) + 10.2097*err_Iq(T) - 10.0703*err_Iq(T-1);% + Ldq*wm(T-1)*Id(T-1) + wm(T-1)*kt; %eq. de diferenças Iq
                Vd_ref(T) = Vd_ref(T-1) + 10.2097*err_Id(T) - 10.0703*err_Id(T-1); % - Ldq*wm(T-1)*Iq(T-1); %eq. de diferenças Id
            else
                err_Iq(T) = err_Iq(T-1);
                err_Id(T) = err_Id(T-1);
                Vq_ref(T) = Vq_ref(T-1);
                Vd_ref(T) = Vd_ref(T-1);
            end
            
        elseif(T == 400000)             %Inicializacao dos PIs
            err_wm(T) = 2.68137153548973468681;   %Valor no final da rampa Iq = -0.2595A;
            err_Iq(T) = 0;              %Valor no final da rampa Vq = 0V;
            err_Id(T) = -0.9830888;     %Valor no final da rampa Vd = 9.9V;
            wm_r(T) = 19.80;
            wm_ref(T) = 19.80;
            Iq_ref(T) = -0.09677883*err_wm(T);
            Vq_ref(T) = -10.0703*err_Iq(T);
            Vd_ref(T) = -10.0703*err_Id(T);
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        theta_a = normalize_angle(theta_e(T-1));                 %electrical angle A, B and C
        theta_b = normalize_angle(theta_a + (4*c_360)/6);
        theta_c = normalize_angle(theta_a + (c_360)/3);
        
        ea = F_theta_e(theta_a);
        eb = F_theta_e(theta_b);
        ec = F_theta_e(theta_c);
        
        Ea(T) = kt * wm(T-1) * sin(theta_a);%ea;             %BEMF A, B and C
        Eb(T) = kt * wm(T-1) * sin(theta_b);%eb;
        Ec(T) = kt * wm(T-1) * sin(theta_c);%ec;
        
        %Krishnan p.237 Permanent Magnet Synchronous Machines
        Ks = [cos(theta_a) sin(theta_a);cos(theta_b) sin(theta_b); cos(theta_c) sin(theta_c)];
        dq0_to_adc = Ks*[Vq_ref(T); Vd_ref(T)];
        
        Va_a(T) = dq0_to_adc(1);
        Vb_a(T) = dq0_to_adc(2);
        Vc_a(T) = dq0_to_adc(3);
        
        %%%%%% SVM Modulation %%%%%% 13/02/18 FONTE: https://www.embedded.com/design/real-world-applications/4441150/2/Painless-MCU-implementation-of-space-vector-modulation-for-electric-motor-systems
        %         Voltage_Phase = [Va_a(T) Vb_a(T) Vc_a(T)];
        %         min_V = min(Voltage_Phase);
        %         max_V = max(Voltage_Phase);
        %         V_neutral(T) = 0.5 * (max_V + min_V);
        
        if(T >= 400000)
            Va(T) = Va_a(T);% - V_neutral(T);
            Vb(T) = Vb_a(T);% - V_neutral(T);
            Vc(T) = Vc_a(T);% - V_neutral(T);
        else
            Va(T) = V*sin(theta_a);
            Vb(T) = V*sin(theta_b);
            Vc(T) = V*sin(theta_c);
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         dIq(T) = (Vq_ref(T) - wm(T-1)*(kt+Ldq*Id(T-1)) - Rs*Iq(T-1))*(t/Ldq);
%         dId(T) = (Vd_ref(T) + wm(T-1)*Ldq*Iq(T-1) - Rs*Id(T-1))*(t/Ldq);
%         Iq(T) = Iq(T-1) + dIq(T);
%         Id(T) = Id(T-1) + dId(T);
%         
%         Idq0_to_abc = Ks*[Iq(T); Id(T)]; 
%         Ia(T) = Idq0_to_abc(1);
%         Ib(T) = Idq0_to_abc(2);
%         Ic(T) = Idq0_to_abc(3);
        dIa(T) = (Va(T) - Rs * Ia(T-1) - Ea(T))*(t/Ldq);
        dIb(T) = (Vb(T) - Rs * Ib(T-1) - Eb(T))*(t/Ldq);
        dIc(T) = (Vc(T) - Rs * Ic(T-1) - Ec(T))*(t/Ldq);
        
        Ia(T) = Ia(T-1) + dIa(T);
        Ib(T) = Ib(T-1) + dIb(T);
        Ic(T) = Ic(T-1) + dIc(T);
        
        Ks_inv = (2/3)*([cos(theta_a) cos(theta_b) cos(theta_c); sin(theta_a) sin(theta_b) sin(theta_c)]);
        adc_to_dqo = Ks_inv*[Ia(T); Ib(T); Ic(T)];
        
        Iq(T) = adc_to_dqo(1);
        Id(T) = adc_to_dqo(2);
        
        Vadc_to_dqo = Ks_inv*[Va(T); Vb(T); Vc(T)];
        Vq(T) = Vadc_to_dqo(1);
        Vd(T) = Vadc_to_dqo(2);

        Te(T) = 1.5*P_2*kt*Iq(T);
        
        dwm(T) = (Te(T) - Tload - B*wm(T-1))*(t/J);
        
        if(T >= 400000)
            wm(T) = wm(T-1) + dwm(T);
        else
            wm(T) = freq;
            wm_ref(T) = wm(T);
        end
        dtheta_m(T) = wm(T) * t;
        theta_m(T) = (theta_m(T-1) + dtheta_m(T));
        theta_m(T) = normalize_angle(theta_m(T));
        theta_e(T) = theta_e(T-1) + P_2 * dtheta_m(T);
        theta_e(T) = normalize_angle(theta_e(T));
        
        time_lapsed(T) = T*t;
        
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
        Vd = zeros(n,1);
        Vq = zeros(n,1);
        dIa = zeros(n,1);
        dIb = zeros(n,1);
        dIc = zeros(n,1);
        Te = zeros(n,1);
        time_lapsed = zeros(n,1);
        V_neutral = zeros(n,1);
        Id_ref = zeros(n,1);
        Iq_ref = zeros(n,1);
        Vd_ref = zeros(n,1);
        Vq_ref = zeros(n,1);
        err_wm = zeros(n,1);
        err_Id = zeros(n,1);
        err_Iq = zeros(n,1);
        V_alfa = zeros(n,1);
        V_beta = zeros(n,1);
        Va_a = zeros(n,1);
        Vb_a = zeros(n,1);
        Vc_a = zeros(n,1);
        ea = zeros(n,1);
        eb = zeros(n,1);
        ec = zeros(n,1);
        wm_ref = zeros(n,1);
        wm_r = zeros(n,1);
    end
end

% subplot(3,1,1);
plot(time_lapsed,Ea,'color','r');
hold;
plot(time_lapsed,Va,'color','b');
% subplot(3,1,2);
% plot(time_lapsed,wm,'color','r');
% subplot(3,1,3);
% plot(time_lapsed,Te,'color','y');
% plot(Id,'color','b');
% hold;
% plot(Iq,'color','g');
% grid on;