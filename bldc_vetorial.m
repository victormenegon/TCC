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
V_bus = 311; 
Fs = 5000;
Ts = 1/Fs;
rst_svm = 1;

dc_a = 0;
dc_b = 0;
dc_c = 0;
%****************************************************************%

for(T = 1:n)

if (T-1 > 0)
    
    time_lapsed(T) = T*t;
    V_barramento(T) = V_bus/2;
   
    theta_a = theta_e(T-1);                 %electrical angle A, B and C
    theta_b = theta_e(T-1) + (4*c_360)/6;
    theta_c = theta_e(T-1) + (c_360)/3;
    
    ea = F_theta_e(normalize_angle(theta_a));
    eb = F_theta_e(normalize_angle(theta_b));
    ec = F_theta_e(normalize_angle(theta_c));
            
    Ea(T) = kt * wm(T-1) * ea;              %BEMF A, B and C
    Eb(T) = kt * wm(T-1) * eb;
    Ec(T) = kt * wm(T-1) * ec;
        
    %%%%%% SVM Modulation %%%%%%
    
    r1 = sqrt(3)*sin((pi/3)-theta_m(T-1));
    r2 = sqrt(3)*sin(theta_m(T-1));
    r0 = 1-Ts*r1-Ts*r2;
    
    t1 = r1*Ts;
    t2 = r2*Ts;
    t0 =r0*Ts;
    
    tempo_acc = time_lapsed(T) * rst_svm;
    
%%%%% 1 Setor %%%%%    
    if(theta_m(T-1) > 0 & theta_m(T-1) <= pi/3)
        
        svm_a = 2/3;
        svm_b = -1/3;
        svm_c = 1/3;
        
        if(tempo_acc >= 2*(t1+t2)+t0)
            dc_a = 0;
            dc_b = 0;
            dc_c = 0;
            rst_svm = 0;
        else
            dc_a = 1;
            rst_svm = 1;
        end
        if(tempo_acc >= t1)
            if(tempo_acc >= 2*(t2)+t0)
                dc_b = 0;
            else
                dc_b = 1;
            end
        end
        if(tempo_acc >= t1+t2)
            if(tempo_acc >= t0)
                dc_c = 0;
            else
                dc_c = 1;
            end
        end
        
%%%%% 2 Setor %%%%%              
    elseif(theta_m(T-1) > (pi/3) & theta_m(T-1) <= (2*pi/3))
        
        svm_a = 1/3;
        svm_b = 1/3;
        svm_c = -2/3;
        
        if(tempo_acc >= 2*(t1+t2)+t0)
            dc_a = 0;
            dc_b = 0;
            dc_c = 0;
            rst_svm = 0;
        else
            dc_b = 1;
            rst_svm = 1;
        end
        if(tempo_acc >= t2)
            if(tempo_acc >= 2*(t1)+t0)
                dc_a = 0;
            else
                dc_a = 1;
            end
        end
        if(tempo_acc >= t1+t2)
            if(tempo_acc >= t0)
                dc_c = 0;
            else
                dc_c = 1;
            end
        end
        
%%%%% 3 Setor %%%%%        
    elseif(theta_m(T-1) > (2*pi/3) & theta_m(T-1) <= (pi))
        svm_a = -1/3;
        svm_b = 2/3;
        svm_c = -1/3;
                 
        if(tempo_acc >= 2*(t1+t2)+t0)
            dc_a = 0;
            dc_b = 0;
            dc_c = 0;
            rst_svm = 0;
        else
            dc_b = 1;
            rst_svm = 1;
        end
        if(tempo_acc >= t1)
            if(tempo_acc >= 2*(t2)+t0)
                dc_c = 0;
            else
                dc_c = 1;
            end
        end
        if(tempo_acc >= t1+t2)
            if(tempo_acc >= t0)
                dc_a = 0;
            else
                dc_a = 1;
            end
        end
        
%%%%% 4 Setor %%%%%          
    elseif(theta_m(T-1) > (pi) & theta_m(T-1) <= (4*pi/3))
        
        svm_a = -2/3;
        svm_b = 1/3;
        svm_c = 1/3;
        
        if(tempo_acc >= 2*(t1+t2)+t0)
            dc_a = 0;
            dc_b = 0;
            dc_c = 0;
            rst_svm = 0;
        else
            dc_c = 1;
            rst_svm = 1;
        end
        if(tempo_acc >= t2)
            if(tempo_acc >= 2*(t1)+t0)
                dc_b = 0;
            else
                dc_b = 1;
            end
        end
        if(tempo_acc >= t1+t2)
            if(tempo_acc >= t0)
                dc_a = 0;
            else
                dc_a = 1;
            end
        end
        
%%%%% 5 Setor %%%%%          
    elseif(theta_m(T-1) > (4*pi/3) & theta_m(T-1) <= (5*pi/3))
        
        svm_a = -1/3;
        svm_b = -1/3;
        svm_c = 2/3;
        
        if(tempo_acc >= 2*(t1+t2)+t0)
            dc_a = 0;
            dc_b = 0;
            dc_c = 0;
            rst_svm = 0;
        else
            dc_c = 1;
            rst_svm = 1;
        end
        if(tempo_acc >= t1)
            if(tempo_acc >= 2*(t2)+t0)
                dc_a = 0;
            else
                dc_a = 1;
            end
        end
        if(tempo_acc >= t1+t2)
            if(tempo_acc >= t0)
                dc_b = 0;
            else
                dc_b = 1;
            end
        end
        
 %%%%% 6 Setor %%%%%      
    else
        
        svm_a = 1/3;
        svm_b = -2/3;
        svm_c = 1/3;
        
        if(tempo_acc >= 2*(t1+t2)+t0)
            dc_a = 0;
            dc_b = 0;
            dc_c = 0;
            rst_svm = 0;
        else
            dc_a = 1;
            rst_svm = 1;
        end
        if(tempo_acc >= t2)
            if(tempo_acc >= 2*(t1)+t0)
                dc_c = 0;
            else
                dc_c = 1;
            end
        end
        if(tempo_acc >= t1+t2)
            if(tempo_acc >= t0)
                dc_b = 0;
            else
                dc_b = 1;
            end
        end
        
    end

    Va(T) = V_barramento(T)*svm_a * dc_a;
    Vb(T) = V_barramento(T)*svm_b * dc_b;
    Vc(T) = V_barramento(T)*svm_c * dc_c;
        
    
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
    
    Ks = (2/3)*([cos(theta_a) cos(theta_b) cos(theta_c); sin(theta_a) sin(theta_b) sin(theta_c); 0.5 0.5 0.5]);
    
    adc_to_dqo = Ks*[Ia(T); Ib(T); Ic(T)];
    
    Id(T) = adc_to_dqo(1);
    Iq(T) = adc_to_dqo(2);

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
    V_barramento = zeros(n,1);

end
end

plot(time_lapsed,Va,'color','g');

