function[batt_k,motor_kk,esc_kkkk,weight_engine_cruise,weight_batt_cruise,work_cruise,span,weight_structure,alpha_cruise,spar_inertias1,spar_inertias2] = fixedwing_design(v_cruise,t_cruise,weight_design_fixedwing,cell_b,work_hover,motor_ii,dia_prop)
syms b 
weight_initial = 10e6;  %(g)
weight_condition = weight_initial;  %(g);
batt_fixedwing = importdata('databatt6600mAh.txt'); %call battdata (fixed batt in lab) you can change data file 
motor_fixedwing = importdata('datamotor_used_cruise.txt');    %call motordata (you can change data file)
esc_fixedwing = importdata('dataesc.txt');  %call escdata

%information for design wing
chord = 0.29;   %fixed chord(m)
cl = 1.5;   %cl of e421 at cl/cd max
alpha_cruise = 6    ;%angle at cl/cd max(degree)
AR_initial = 7; %AR intial for loop wing design
error_AR = 99;  %error intial for loop wing design
AR = AR_initial;
while error_AR > 0.00001
    CL = cl*(AR/(AR+2));    %find CL from cl
    s_wing = 2*(weight_design_fixedwing*9.81/1000)/(1.225*v_cruise^2*CL);   %find surface of wing
    span = s_wing/chord;    %span from S_wing
    AR_new = span/chord;    %find AR from span
    error_AR = abs((AR_new - AR)/AR_new)*100;   %check error
    AR = AR_new;    %update AR
end

%weight calculate part
foam_density = 32.04;   %(kg/m^3)
carbonrod_weightperm = 53.4;  %weight carbon in lab(g/m)
carbon_density = 1600;  %(kg/m^3)
tensile_strength = 0.85*570*10^6;   %carbon tensile strength(N/m^2)
weight_wing = ((span*(5/100)*chord*chord*foam_density)*1000  + 200)*1.1; %(g) 200 is 3d print rib and 10% is for lay fiber glass
% + (carbonrod_weightperm*span*2)
weight_tail = 0.1*weight_wing +80;   %(g) 10%is foam and 80 is 3d print
weight_body = 300 ;  %(g) 220(laser cut)+20(drop mechanic)+40(3d print) +20(aluminium motor)
T_hovermax = motor_fixedwing(motor_ii,16)*9.81/1000;    %call thrust max from quad(N) 
prop_dia = dia_prop*0.0254; % diameter in propeller(m)
inertia1 = (2*T_hovermax*(prop_dia/2+0.34/sqrt(2))*(b/2))/tensile_strength == (pi/4)*(((b^4/2)-(((b-0.002)^4)/2)));    %solve inertia equation in longitudinal beam(m^4)by the way 0.34/sqrt(2)= body width i
inertia2 = (T_hovermax*(prop_dia+0.29)/2*(b/2))/tensile_strength == (b^4-(b-0.002)^4)/12; %solve inertia equation in lateral beam(m^4)by the way 0.29 = chord
b1 = solve(inertia1,b); %find b1 from inertia1(m)
b2 = solve(inertia2,b); %find b2 from inertia2(m)
B1 = real(vpa(b1(3)));   % numerically approximate a root by using vpa command
B2 = real(vpa(b2(3)));
spar_inertias1 = (pi/4)*((((B1^4)/2)-(((B1-0.002)^4)/2)));
spar_inertias2 = ((B2)^4-(B2-0.002)^4)/12;
weight_b1 = (0.156 + prop_dia)*1.1*carbonrod_weightperm*2 ; %0.156 is body's width , +10% margin (middle spa)
weight_b2 = (chord + prop_dia)*1.1*carbonrod_weightperm*2 ; %spa adapter quad
weight_b3 = 0.8*(span - (0.156 + prop_dia))*1.1*carbonrod_weightperm*2;  %outer spa estimated 80%
%weight_b4 = .6*carbonrod_weightperm*1.1;
weight_structure = weight_wing + weight_tail + real(weight_b2) + weight_body + real(weight_b1) + weight_b3;% + weight_b4; %sum weight of wing structure(g)
%we haven't cal weight of bloom tail... cry 
%pls recheck carbon density
%weight ??????????????


%drag estimate part*

cd = 0.016; %cd of e421 at cl/cd max
cd_i = CL^2/(pi*0.85*AR);   %induced drag
cd_motor = 0.8;
cd_body = 0.8;
Drag = (0.5*1.225*(v_cruise^2)*s_wing*(cd + cd_i))+2*(0.5*1.225*(v_cruise^2)*0.0016*(cd_motor));%+(0.5*1.225*(v_cruise^2)*0.2672*(cd_body));%(N)
drag = double(Drag);

%cruise motorselection part
batt_n_fixedwing = size(batt_fixedwing,1);  %count batt number
motor_n_fixedwing = size(motor_fixedwing,1);    %count motor number
esc_n_fixedwing = size(esc_fixedwing,1);    %count esc number

for k=1:batt_n_fixedwing
    cell_b_fixedwing = batt_fixedwing(k,1); %call batt cell(g)
    mAh_b_fixedwing = batt_fixedwing(k,2);  %call capacity cell(g)
    c_dis_fixedwing = batt_fixedwing(k,3);  %call cdischarge(g)
    weight_b_fixedwing = batt_fixedwing(k,4);   %call weight batt(g)
    
    if cell_b ~= cell_b_fixedwing%if cell not match with multirotor continue to next batt
        continue
    end

    for kk = 1:motor_n_fixedwing
        weight_m_fixedwing = motor_fixedwing(kk,1); %call motor weight
        cell_m_fixedwing = motor_fixedwing(kk,2);   %call motor cell
        A_m_fixedwing = zeros(1,5); %creat ampere metrix
        T_m_fixedwing = zeros(1,5); %creat thrust metrix
        eff_m_fixedwing = zeros(1,5);   %creat efficientcy matrix
        
         for kkk = 1:5
           A_m_fixedwing(kkk) = motor_fixedwing(kk,kkk*3);  %call ampere to new metrix
           T_m_fixedwing(kkk) = motor_fixedwing(kk,kkk*3+1)*9.81/1000;  %call thrust to new metrix
           eff_m_fixedwing(kkk) = motor_fixedwing(kk,kkk*3+2)*9.81/1000;    %call efficient to new metrix  
         end
        
         if cell_b_fixedwing ~= cell_m_fixedwing %if cell not match continue to choose next motor
            continue
         end
         
          for kkkk =  1:esc_n_fixedwing
            fprintf('\nbatt No.%g, motor No.%g, esc No.%g',k,kk,kkkk);
            A_esc_fixedwing = esc_fixedwing(kkkk,1);    %call esc amp
            weight_esc_fixedwing = esc_fixedwing(kkkk,2);   %call esc weight(g)
            if T_m_fixedwing(5)< drag %check motor has enough thrust
                fprintf('\nnot enough thrust');
                break
            elseif A_esc_fixedwing < A_m_fixedwing(5)%check esc has enough Amp
                fprintf('\nesc low amp');
                continue
            elseif (mAh_b_fixedwing/1000)*c_dis_fixedwing < A_m_fixedwing(5)%check battery can use with all motor
                fprintf('\nbatt low amp');
                break
            end
            
            T2eff = fit(T_m_fixedwing',eff_m_fixedwing','poly2');   %fit curve thrust vs eff
            eff_drag = (T2eff(drag));   %find eff from drag
            Power_drag = drag/eff_drag; %find power (W)
            work_cruise_cal = Power_drag*t_cruise*60;   %find work cruise(J)
            work_total = work_cruise_cal + work_hover;  %find work total(J)
            work_b = mAh_b_fixedwing*3.7*cell_b_fixedwing*0.8*3600/1000; %find work batt
            
            if work_b < work_total%check batt has enough work
                fprintf('\nbatt low capacity');
                break   
            end
            weight_total = weight_b_fixedwing + weight_m_fixedwing + weight_esc_fixedwing; %sum weight
            
            if weight_total < weight_condition %condition to choose the best data
               weight_condition = weight_total;
               batt_k = k;
               motor_kk = kk ;
               esc_kkkk = kkkk;
               weight_engine_cruise = weight_m_fixedwing + weight_esc_fixedwing;
               weight_batt_cruise = weight_b_fixedwing;
               work_cruise = work_cruise_cal;
               fprintf('\ndata collected');
               break
            else fprintf('\nHeavier');
            end
            
          end
    end
    
end
