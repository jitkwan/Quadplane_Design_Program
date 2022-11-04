clear all
clc

fprintf('welcome to quadplane design program by MJ');   %headline
fprintf('\n\nplease inform the data below this\n');
weight_payload = input('\nYour payload weight(g) = ');  %input paload
time_hover = input('\nYour hover time(min) = ');    %input cruise time
time_cruise = input('\nYour cruise time(min) = ');
n_motor = input('\nNumber of multirotor motor = '); %input number of motor

component = importdata('datacomponent.txt');    %call weight component (datail in pic)
weight_component = sum(component) +200 ;    %200 g for all of glue  
%next time dont forget landing gear na ka deaw jane deak hua
weight_struc = 500; %estimate weight struc(g)
weight_fixed = weight_component + weight_payload ; 
weight_wing = 500;  %estimate weight wing(g)
weight_tail = 0.1*weight_wing; %estimate weight tail= 0.1weight wing(g)
v_cruise = 11;  %chosen velocity(m/s)
weight_new = weight_fixed +weight_wing+weight_struc+weight_tail;
weight_engine = 0;  %weight engine initial(g)
weight_batt_cruise = 0; %intial weight batt quad(g)
weight_motor_cruise = 0;    %intial weight motor cruise(g)
work_cruise = 10000;    %work cruise intial(J)
error = 250;    %error(percentage)
Loop =1;

fprintf('\nCalculating part');
fprintf('\n-------------------------------------------------------------------------------------------\n');
while error > 0.00001
   fprintf('\n\n-- Iteration : %g --\n',Loop);
   weight_old = weight_new;
   weight_est = weight_old - weight_engine - weight_batt_cruise;
   fprintf('\n\nmultirotor design part\n');
   
   [batt_i,motor_ii,esc_iiii,weight_engine,weight_batt_quad,cell,work_hover,dia_prop] = multirotor_select(n_motor,weight_est,time_hover,work_cruise);
   
   weight_design_fixedwing = weight_est + weight_engine + weight_batt_quad + weight_motor_cruise;
   
   fprintf('\n\nfixedwing_design part\n');
   
   [batt_k,motor_kk,esc_kkkk,weight_engine_cruise,weight_batt_cruise,work_cruise,span,weight_structure,alpha_cruise,spar_inertias1,spar_inertias2] = fixedwing_design(v_cruise,time_cruise,weight_design_fixedwing,cell,work_hover,motor_ii,dia_prop);

   weight_new = weight_fixed + weight_engine + weight_batt_cruise + weight_engine_cruise + weight_structure;
     
   error = abs((weight_new-weight_old)/weight_new)*100;
   Loop = Loop+1; 
end
fprintf('\n\n==========================================================================\n');
fprintf('\nDESIGNED QUAPLANE');
fprintf('\niteration :%g rounds',Loop-1);
fprintf('\n\n**** SPECIFICATION ****\n');
fprintf('\nAll Up Weight = %g g',weight_new);
fprintf('\nCruise speed = %g m/s',v_cruise);
fprintf('\n\nWing part :');
fprintf('\nWing airfoil = e421');
fprintf('\nWing span = %g m',span);
fprintf('\nWing chord = 0.29 m');
fprintf('\nMain spar inertia = %g ',spar_inertias1);
fprintf('\nMulti motor spar inertia = %g ',spar_inertias2);
fprintf('\nAoA = %g deg',alpha_cruise);
fprintf('\n\nEngine part :');
fprintf('\nMulti Motor No.%g',motor_ii);
fprintf('\nMulti ESC No.%g',esc_iiii);
fprintf('\nFixed wing Motor No.%g',motor_kk);
fprintf('\nFixed wing ESC No.%g',esc_kkkk);
fprintf('\nBattery No.%g',batt_k);
fprintf('\n\nENJOY!!\n');
