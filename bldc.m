close all;
clear all;
clc;

%************ Constants and Variables initialization ************%

n = 10000000;       %simulation lenght

wm_ref = 167;       %rad/s
Tload = 0.0;          %Nm
B = 2.8142e-4;         %Nms
J = 8.7e-4;         %Kgm^2
Rs = 4.65;           %ohms
P = 4;              %polos
P_2 = P/2;          %pares de polos
Ldq = 69.7e-3;        % Ld = Lq = L-M %%%%CONFIRMAR ISSO%%%%
kt = 0.359;         %V*s/rad
t = 0.000001;       %passo de calculo
c_360 = 2*pi;
phase = -pi/6;      %para corrigir o defasamento entre Vx e Ex, 
                    %pq foi implementado com 30 de avanço.
r_on = 0.001;       %resistência quando chave conduz
r_off = 5000000;    %resistência quando chave não conduz

r1 = r_off;
r2 = r_off;
r3 = r_off;
r4 = r_off;
r5 = r_off;
r6 = r_off;

polo = 0.03668;
Kp_wm = 0.0146;
Ki_wm = Kp_wm/(polo);

V_pwm = 10;         %PWM Voltage reference
V_bus = 311;        %Bus Voltage
Fs = 5000;          %Switching frequency of inverter
Ts = 1/Fs;
gain = V_bus/V_pwm; %Gain used to bring V_ref from controller to PWM range

delta = V_bus/r_off;
%****************************************************************%

for(T = 1:n)

if (T-1 > 0)

    if(T >= 10000000/2)
            Tload = 0.1; 
    end
    err_wm(T) = wm_ref - wm(T-1);                   %Proportional error
    err_int_wm(T) = err_int_wm(T-1) + err_wm(T)*t;  %Integral error
    V_ref(T) =  Kp_wm * err_wm(T) + Ki_wm * err_int_wm(T);       %controller
    
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
        y(T) = 0;
        pwm_st(T) = 0;
    end
   
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
if(theta_e(T-1) <= (pi/6+phase))
        r1 = r_off;
        r2 = r_off; 
        r3 = r_off;
        r4 = r_on;
        r5 = r_on;
        r6 = r_off;

        if(pwm_st(T) == 0)
            if(Ic(T-1) >= delta)
            r3 = r_on;
            r5 = r_on;
            r4 = r_off;
            r6 = r_off;
            elseif (Ic(T-1) <= -delta)
            r4 = r_on;
            r6 = r_on;
            r3 = r_off;
            r5 = r_off;
            end
        end
            
    elseif(theta_e(T-1) > (pi/6+phase) && theta_e(T-1) <= (pi/2+phase))
            r1 = r_on;
            r2 = r_off;
            r3 = r_off;
            r4 = r_on;
            r5 = r_off;
            r6 = r_off;

        if(pwm_st(T) == 0)
            if(Ia(T-1) >= delta)
            r1 = r_on;
            r3 = r_on;
            r2 = r_off;
            r4 = r_off;
            elseif (Ia(T-1) <= -delta)
            r2 = r_on;
            r4 = r_on;
            r1 = r_off;
            r3 = r_off;
            end
        end
    elseif(theta_e(T-1) > (pi/2+phase) && theta_e(T-1) <= (5*pi/6+phase))
            r1 = r_on;
            r2 = r_off;
            r3 = r_off;
            r4 = r_off;
            r5 = r_off;
            r6 = r_on;
            
            if(pwm_st(T) == 0)   
                if(Ia(T-1) >= delta)
                r1 = r_on;
                r5 = r_on;
                r2 = r_off;
                r6 = r_off;
                elseif (Ia(T-1) <= -delta)
                r2 = r_on;
                r6 = r_on;
                r1 = r_off;
                r5 = r_off;
                end
            end
    elseif(theta_e(T-1) > (5*pi/6+phase) && theta_e(T-1) <= (7*pi/6+phase))
            r1 = r_off;
            r2 = r_off;
            r3 = r_on;
            r4 = r_off;
            r5 = r_off;
            r6 = r_on;
        
            if(pwm_st(T) == 0)
                if(Ib(T-1) >= delta)
                r3 = r_on;
                r5 = r_on;
                r4 = r_off;
                r6 = r_off;
                elseif (Ib(T-1) <= -delta)
                r4 = r_on;
                r6 = r_on;
                r3 = r_off;
                r5 = r_off;
        end
            end       
    elseif(theta_e(T-1) > (7*pi/6+phase) && theta_e(T-1) <= (3*pi/2+phase))
            r1 = r_off;
            r2 = r_on;
            r3 = r_on;
            r4 = r_off;
            r5 = r_off;
            r6 = r_off; 
        
            if(pwm_st(T) == 0)
                if(Ib(T-1) >= delta)
                r1 = r_on;
                r3 = r_on;
                r2 = r_off;
                r4 = r_off;
                elseif (Ib(T-1) <= -delta)
                r2 = r_on;
                r4 = r_on;
                r1 = r_off;
                r3 = r_off;
                end
            end
    elseif(theta_e(T-1) > (3*pi/2+phase) && theta_e(T-1) <= (33*pi/18+phase))  
            r1 = r_off;
            r2 = r_on;
            r3 = r_off;
            r4 = r_off;
            r5 = r_on;
            r6 = r_off;

            if(pwm_st(T) == 0)
                if(Ic(T-1) >= delta)
                r1 = r_on;
                r5 = r_on; 
                r2 = r_off;
                r6 = r_off;
                elseif (Ic(T-1) <= -delta)
                r2 = r_on;
                r6 = r_on;
                r1 = r_off;
                r5 = r_off; 
                end
            end
    else
            r1 = r_off;
            r2 = r_off;
            r3 = r_off;
            r4 = r_on;
            r5 = r_on;
            r6 = r_off;
                        
            if(pwm_st(T) == 0)
                if(Ic(T-1) >= delta)
                r3 = r_on;
                r5 = r_on; 
                r4 = r_off;
                r6 = r_off;
                elseif (Ic(T-1) <= -delta)
                r4 = r_on;
                r6 = r_on;
                r3 = r_off;
                r5 = r_off;
                end
            end
end
    
    if(r1 == r_off && r2 == r_off)
        if(Ia(T-1) >= delta)
            r1 = r_off;
            r2 = r_on;
        elseif (Ia(T-1) <= -delta)
            r1 = r_on;
            r2 = r_off;
        end
    end
    if(r3 == r_off && r4 == r_off)
        if(Ib(T-1) >= delta)
            r3 = r_off;
            r4 = r_on;
        elseif (Ib(T-1) <= -delta)
            r3 = r_on;
            r4 = r_off;
        end
    end
    if(r5 == r_off && r6 == r_off)
        if(Ic(T-1) >= delta)
            r5 = r_off;
            r6 = r_on;  
        elseif (Ic(T-1) <= -delta)
            r5 = r_on;
            r6 = r_off;
        end
    end
    
    k1(T) = (r1*r2)/(r1+r2);
    k2(T) = (r3*r4)/(r3+r4);
    k3(T) = (r5*r6)/(r5+r6);
    k4(T) = r2/(r1+r2);
    k5(T) = r4/(r3+r4);
    k6(T) = r6/(r5+r6);
    
    Ta(T) = kt * Ia(T-1) * ea;
    Tb(T) = kt * Ib(T-1) * eb;
    Tc(T) = kt * Ic(T-1) * ec;
    
    Te(T) = 2 *( Ta(T) + Tb(T) + Tc(T));
    
    dIa(T) = ((-2*k1(T)*Ia(T-1))+(-2*Rs*Ia(T-1))+(k2(T)*Ib(T-1))+(Rs*Ib(T-1))+(k3(T)*Ic(T-1))+(Rs*Ic(T-1))+((2*k4(T)*V_bus)-(k5(T)*V_bus)-(k6(T)*V_bus)+(Eb(T)+Ec(T)-2*Ea(T))))*(t/(3*Ldq));
    dIb(T) = ((-2*k2(T)*Ib(T-1))+(-2*Rs*Ib(T-1))+(k1(T)*Ia(T-1))+(Rs*Ia(T-1))+(k3(T)*Ic(T-1))+(Rs*Ic(T-1))+((2*k5(T)*V_bus)-(k4(T)*V_bus)-(k6(T)*V_bus)+(Ea(T)+Ec(T)-2*Eb(T))))*(t/(3*Ldq));
    dIc(T) = ((-2*k3(T)*Ic(T-1))+(-2*Rs*Ic(T-1))+(k1(T)*Ia(T-1))+(Rs*Ia(T-1))+(k2(T)*Ib(T-1))+(Rs*Ib(T-1))+((2*k6(T)*V_bus)-(k4(T)*V_bus)-(k5(T)*V_bus)+(Ea(T)+Eb(T)-2*Ec(T))))*(t/(3*Ldq));
    
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
    
    Va(T) = Rs*Ia(T) + Ldq*dIa(T) +  Ea(T);
    Vb(T) = Rs*Ib(T) + Ldq*dIb(T) +  Eb(T);
    Vc(T) = Rs*Ic(T) + Ldq*dIc(T) +  Ec(T);
    
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

 plot(time_lapsed,Ia,'color','g');
 hold;
 plot(time_lapsed,Ib,'color','r');
 plot(time_lapsed,Ic,'color','b');
%plot(time_lapsed,wm,'color','r');




