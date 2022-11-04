function[batt_i,motor_ii,esc_iiii,weight_engine,cell,work_hover] = motorbattesc_select(n_motor,weight_est,t_hover,work_cruise)
weight_initial = 10e6;  %(g)
weight_condition = weight_initial;
batt = importdata('databatt.txt');  %call battdata
motor = importdata('datamotor.txt');    %call motordata
esc = importdata('dataesc.txt');%call escdata
batt_n = size(batt,1);%count batt number
motor_n = size(motor,1);%count motor number
esc_n = size(esc,1);%count esc number


for i=1:batt_n
    cell_b = batt(i,1);%call batt cell
    mAh_b = batt(i,2);%call capacity cell
    c_dis = batt(i,3);%call cdischarge
    weight_b = batt(i,4);%call weight batt(g)
    
    for ii = 1:motor_n
        weight_m = motor(ii,1);%call motor weight
        cell_m = motor(ii,2);%call motor cell
        A_m = zeros(1,5);%creat ampere metrix
        T_m = zeros(1,5);%creat thrust metrix
        eff_m = zeros(1,5);%creat efficientcy matrix
        
        for iii = 1:5
           A_m(iii) = motor(ii,iii*3);%call ampere to new metrix
           T_m(iii) = motor(ii,iii*3+1)*9.81/1000;%call thrust to new metrix
           eff_m(iii) = motor(ii,iii*3+2)*9.81/1000;%call efficient to new metrix  
        end
        
        if cell_b ~= cell_m%if cell not match continue to choose next motor
            continue
        end
        
        for iiii =  1:esc_n
            fprintf('\nbatt No.%g, motor No.%g, esc No.%g',i,ii,iiii);
            A_esc = esc(iiii,1);%call esc amp
            weight_esc = esc(iiii,2);%call esc weight
            weight_engine_cal = weight_b+weight_m+weight_esc;
            T_req = (weight_engine_cal+weight_est)*9.81/(1000*n_motor);%find thrust req per motor(N)

            if T_m(5) < 2*T_req%check motor has enough thrust
                fprintf('\nnot enough thrust');
                break
            elseif A_esc < A_m(5)%check esc has enough Amp
                fprintf('\nesc low amp');
                continue
            elseif (mAh_b/1000)*c_dis < n_motor*A_m(5)%check battery can use with all motor
                fprintf('\nbatt low amp');
                break
            end
            
            T2eff = fit(T_m',eff_m','poly2');%fit curve thrust vs eff
            eff_Treq = (T2eff(T_req));%find eff from thrust req
            Power = T_req/eff_Treq;%find power (W)
            work_hover_cal = Power*(t_hover*60)*n_motor;%find work hover for all motor(J)
            work_total = work_hover_cal + work_cruise;%find work total which wcruise input(J)
            work_b = mAh_b*3.7*cell_b*0.8*3600/1000; %find work batt
            
            if work_b < work_total%check batt has enough work
                fprintf('\nbatt low capacity');
                break   
            end
            
            weight_total = weight_est + weight_engine_cal;%sum weight
            
            if weight_total < weight_condition %condition to choose the best data
               weight_condition = weight_total;
               batt_i = i;
               motor_ii = ii;
               esc_iiii = iiii;
               weight_engine = weight_engine_cal;
               cell = cell_b;
               work_hover = work_hover_cal;
               fprintf('\ndata collected');
               break
            else fprintf('\nHeavier');
            end 
        end
    end
end