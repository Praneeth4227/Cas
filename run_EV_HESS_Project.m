% Execute the reduced-order EV/HESS eight-case comparison.
projectRoot=fileparts(mfilename('fullpath'));
assert(strcmp(version('-release'),'2025b') && contains(version,'Update 3'),'MATLAB R2025b Update 3 required');
assert(license('test','Simulink'),'Simulink licence required');
originalFolder=pwd;folderCleanup=onCleanup(@()cd(originalFolder));cd(projectRoot);
disp(which('run_EV_HESS_Project','-all'));
cacheRoot=fullfile(fileparts(projectRoot),'EVHESS_CACHE');codeRoot=fullfile(fileparts(projectRoot),'EVHESS_CODEGEN');
Simulink.fileGenControl('set','CacheFolder',cacheRoot,'CodeGenFolder',codeRoot,'createDir',true);
modelFile=fullfile(projectRoot,'EV_HESS_Energy_Management.slx');
assert(isfile(modelFile) && isfolder(fullfile(projectRoot,'Data','ML_Predictions')), ...
 'EVHESS:NotExtracted','Extract the complete project ZIP into a permanent writable folder before running.');
if bdIsLoaded('EV_HESS_Energy_Management')
 loadedFile=get_param('EV_HESS_Energy_Management','FileName');
 if ~strcmpi(loadedFile,modelFile)
  assert(strcmp(get_param('EV_HESS_Energy_Management','Dirty'),'off'), ...
   'EVHESS:UnsavedDuplicate','Another copy of EV_HESS_Energy_Management has unsaved changes. Save or close that copy before running this project.');
  close_system('EV_HESS_Energy_Management',0);
 end
end
load_system(modelFile);open_system('EV_HESS_Energy_Management');
cycles=["EUDC","IM240"];cases=["BatteryOnly","AnalyticalHESS","LSTM_HESS","GRU_HESS"];
metrics=table();
for cycle=cycles
 for caseName=cases
  mode="Analytical";if caseName=="LSTM_HESS",mode="LSTM";elseif caseName=="GRU_HESS",mode="GRU";end
  out=fullfile(projectRoot,'Results','Simulation_Data',cycle,caseName);if ~isfolder(out),mkdir(out);end
  if exist('EVHESS_verifySaved','var') && EVHESS_verifySaved
   saved=load(fullfile(out,'trace.mat'));trace=saved.trace;audit=saved.audit;
  else
   [trace,audit]=simulate_case(cycle,mode,caseName~="BatteryOnly",string(out));
  end
  configuration=plant_parameters();configuration=rmfield(configuration,'root');save(fullfile(out,'trace.mat'),'trace','audit','configuration','-v7');
  m=calculate_metrics(trace,cycle,caseName);metrics=[metrics;m];
 end
end
baseline=readtable(fullfile(projectRoot,'Verification','reference_metrics.csv'),'TextType','string','VariableNamingRule','preserve');
fields=["peak_discharge_A","peak_charging_A","battery_RMS_A","battery_terminal_Wh","storage_adjusted_Wh","SOC_change_pp","SC_initial_Wh","SC_final_Wh","inductor_initial_Wh","inductor_final_Wh","converter_loss_Wh"];
for k=1:height(metrics)
 j=find(baseline.cycle==metrics.cycle(k) & baseline.case==metrics.case(k));assert(isscalar(j));
 for field=fields
  assert(abs(metrics.(field)(k)-baseline.(field)(j))<=1e-8*max(1,abs(baseline.(field)(j))),'Regression mismatch: %s',field);
 end
end
writetable(metrics,fullfile(projectRoot,'Results','Metrics','comparison.csv'));
for cycle=cycles
 t=metrics(metrics.cycle==cycle,:);t.peak_reduction_pct=100*(1-t.peak_discharge_A/t.peak_discharge_A(1));t.RMS_reduction_pct=100*(1-t.battery_RMS_A/t.battery_RMS_A(1));disp(cycle);disp(t);
end
show_results(projectRoot,metrics);
fprintf('PASS: all eight cases and full-precision regression checks. Current stress reductions do not establish net energy savings.\n');
load_system(fullfile(projectRoot,'EV_HESS_Energy_Management.slx'));open_system('EV_HESS_Energy_Management');
fid=fopen(fullfile(projectRoot,'Verification','RUNTIME_PASS.txt'),'w');fprintf(fid,'%s\n',version);fclose(fid);

function m=calculate_metrics(r,cycle,caseName)
 dt=r.interval_dt_s;a=dt>0;duration=sum(dt);terminal=sum(r.Pbattery_W.*dt)/3600;sc0=r.SC_energy_J(1)/3600;sc1=r.SC_energy_J(end)/3600;l0=r.inductor_energy_J(1)/3600;l1=r.inductor_energy_J(end)/3600;
 m=table(cycle,caseName,max(r.Ibattery_A(a)),min(r.Ibattery_A(a)),sqrt(sum(r.Ibattery_A.^2.*dt)/duration),100*(r.SOC(end)-r.SOC(1)),terminal,sc0,sc1,l0,l1,terminal+sc0-sc1+l0-l1,sum(r.converter_loss_W.*dt)/3600,min(r.SC_voltage_V),max(r.SC_voltage_V),max(abs(r.Isc_actual_A(a))),max(abs(r.system_balance_residual_W(a))),'VariableNames',{'cycle','case','peak_discharge_A','peak_charging_A','battery_RMS_A','SOC_change_pp','battery_terminal_Wh','SC_initial_Wh','SC_final_Wh','inductor_initial_Wh','inductor_final_Wh','storage_adjusted_Wh','converter_loss_Wh','SC_min_V','SC_max_V','SC_peak_absolute_A','system_power_residual_W'});
m.SC_tracking_RMSE_A=sqrt(sum((r.Isc_command_A-r.iL_state_A).^2.*dt)/duration);m.protection_intervals=nnz(r.reference_limited(a)|r.voltage_guard(a)|r.predictive_intervention(a));m.integrated_residual_J=sum((r.Pbattery_W-r.Pdc_load_W-r.converter_loss_W).*dt)-(r.SC_energy_J(end)-r.SC_energy_J(1))-(r.inductor_energy_J(end)-r.inductor_energy_J(1));
end
function show_results(root,m)
 cases=["BatteryOnly","AnalyticalHESS","LSTM_HESS","GRU_HESS"];labels=["Battery-only","Analytical HESS","LSTM HESS","GRU HESS"];
 for cycle=["EUDC","IM240"]
  for signal=["Ibattery_A","SOC","SC"]
   f=figure('Name',cycle+" "+signal,'Visible','on','Theme','light','Color','white');layout=tiledlayout(f,1,1);ax=nexttile(layout);hold(ax,'on');
   if signal=="SC",delete(ax);layout=tiledlayout(f,2,1);ax=nexttile(layout);hold(ax,'on');ax2=nexttile(layout);hold(ax2,'on');end
   for k=1:4
    d=load(fullfile(root,'Results','Simulation_Data',cycle,cases(k),'trace.mat'),'trace');r=d.trace;
    if signal=="SC"
     if k==1,continue;end
     plot(ax,r.time_s,r.SC_voltage_V,'DisplayName',labels(k));plot(ax2,r.time_s,r.Isc_actual_A,'DisplayName',labels(k));
    else,plot(ax,r.time_s,r.(signal),'DisplayName',labels(k));end
   end
   xlabel(ax,'Time (s)');ylabel(ax,signal);if signal=="Ibattery_A",ylabel(ax,'Battery current (A)');elseif signal=="SOC",ylabel(ax,'Battery SOC (fraction)');else,ylabel(ax,'SC voltage (V)');ylabel(ax2,'SC current (A)');xlabel(ax2,'Time (s)');legend(ax2,'Location','best');grid(ax2,'on');end
   title(ax,cycle+" "+strrep(signal,"_"," "));legend(ax,'Location','best');grid(ax,'on');set(ax,'FontSize',11);save_graph(f,root,cycle+"_"+signal);
  end
 end
 for field=["peak_discharge_A","battery_RMS_A","storage_adjusted_Wh"]
  f=figure('Name',field,'Visible','on','Theme','light');tiledlayout(f,1,1);ax=nexttile;values=[m.(field)(1:4),m.(field)(5:8)];bar(ax,values);xticks(ax,1:4);xticklabels(ax,labels);legend(ax,'EUDC','IM240');title(ax,strrep(field,'_',' '));ylabel(ax,field);grid(ax,'on');save_graph(f,root,field);
 end
end
function save_graph(f,root,name)
 savefig(f,fullfile(root,'Results','Figures',name+".fig"));exportgraphics(f,fullfile(root,'Results','Figures',name+".png"),'Resolution',150);
end

function [r,audit]=simulate_case(cycle,mode,hybrid,out)
%RUN_INTEGRATED_CASE Execute one fixed-source case through the shared frozen plant.
arguments
 cycle (1,1) string {mustBeMember(cycle,["EUDC","IM240"])}
 mode (1,1) string {mustBeMember(mode,["Analytical","LSTM","GRU"])}
 hybrid (1,1) logical = true
 out (1,1) string = ""
end
root=fileparts(mfilename('fullpath'));p=plant_parameters();v=p.values;model='EV_HESS_Energy_Management';
[l,~]=load_prediction(root,cycle,"LSTM");[g,~]=load_prediction(root,cycle,"GRU");c=load_drive_cycle(cycle,p);
load_system(fullfile(root,[model '.slx']));in=Simulink.SimulationInput(model);
in=in.setVariable('drive_cycle',[c.time_s,c.speed_mps,c.acceleration_mps2,c.raw_reference_A],'Workspace',model);
in=in.setVariable('ml_predictions',[l.time_s,l.raw_prediction_A,g.raw_prediction_A],'Workspace',model);
in=in.setVariable('reference_mode',find(["Analytical","LSTM","GRU"]==mode),'Workspace',model);
in=in.setVariable('hybrid_enabled',double(hybrid),'Workspace',model);
x0=[v.SC_initial_V;0;0];if ~hybrid,x0=zeros(3,1);end
in=in.setVariable('initial_storage_state',x0,'Workspace',model);
in=in.setVariable('initial_battery_SOC',v.battery_initial_SOC,'Workspace',model);
in=in.setModelParameter('StopTime',num2str(c.time_s(end)));
fprintf('SIMULATION_START %s %s hybrid=%d\n',cycle,mode,hybrid);tic;actual=sim(in);elapsed=toc;
ts=actual.get('plant_logged');t=ts.Time;data=matrix(ts);
names={'Pbattery_W','Ibattery_A','SOC','SC_voltage_V','iL_state_A','Isc_actual_A','Isc_command_A','duty','Psc_bus_W','converter_loss_W','SC_energy_J','inductor_energy_J','converter_balance_residual_W'};
r=array2table(data,'VariableNames',names);r=addvars(r,t,'Before',1,'NewVariableNames','time_s');
ref=aligned(actual.get('reference_references'),t);input=aligned(actual.get('reference_input'),t);state=aligned(actual.get('reference_state'),t);
r.speed_mps=input(:,1);r.acceleration_mps2=input(:,2);r.Pwheel_W=aligned(actual.get('reference_wheel'),t);r.Pdc_load_W=aligned(actual.get('reference_demand'),t);
r.analytical_reference_A=ref(:,1);r.raw_LSTM_A=ref(:,2);r.raw_GRU_A=ref(:,3);r.selected_raw_reference_A=ref(:,4);r.PI_integral_As=state(:,3);
r.source_index=min(floor(t+1e-10)+1,numel(c.time_s));r.interval_dt_s=[diff(t);0];r.Ibus_A=r.Psc_bus_W/v.Vdc_V;
rawDuty=r.SC_voltage_V/v.Vdc_V-v.Kp_per_A*(r.Isc_command_A-r.iL_state_A)-v.Ki_per_As*r.PI_integral_As;
limited=min(max(r.selected_raw_reference_A,-v.SC_current_limit_A),v.SC_current_limit_A);
r.reference_limited=(limited~=r.selected_raw_reference_A);
r.voltage_guard=(r.SC_voltage_V<=v.SC_min_V+v.SC_voltage_guard_V & limited>0) | (r.SC_voltage_V>=v.SC_max_V-v.SC_voltage_guard_V & limited<0);
r.raw_duty=rawDuty;r.duty_saturated=rawDuty<v.duty_min | rawDuty>v.duty_max;
r.predictive_intervention=abs(r.duty-min(max(rawDuty,v.duty_min),v.duty_max))>1e-12;
r.integrator_frozen=abs(r.duty-rawDuty)>1e-12 & (r.Isc_command_A-r.iL_state_A).*(r.duty-rawDuty)>0;
if ~hybrid,r.reference_limited(:)=false;r.voltage_guard(:)=false;r.duty_saturated(:)=false;r.predictive_intervention(:)=false;r.integrator_frozen(:)=false;end
r.battery_terminal_energy_J=[0;cumsum(r.Pbattery_W(1:end-1).*diff(t))];
r.load_energy_J=[0;cumsum(r.Pdc_load_W(1:end-1).*diff(t))];r.converter_loss_energy_J=[0;cumsum(r.converter_loss_W(1:end-1).*diff(t))];
r.system_balance_residual_W=[r.Pbattery_W(1:end-1)-r.Pdc_load_W(1:end-1)-diff(r.SC_energy_J)./diff(t)-diff(r.inductor_energy_J)./diff(t)-r.converter_loss_W(1:end-1);0];
active=r.interval_dt_s>0;idx=r.source_index;
expected=c.raw_reference_A;if mode=="LSTM",expected=l.raw_prediction_A;elseif mode=="GRU",expected=g.raw_prediction_A;end
checks=table();check('finite_model_telemetry',all(isfinite(r{:,vartype('numeric')}),'all'));
check('sample_count_and_time_grid',height(r)==round(c.time_s(end)/v.dt_s)+1 && max(abs(t-(0:height(r)-1)'*v.dt_s))<1e-9);
check('selected_reference_exact_alignment',max(abs(r.selected_raw_reference_A-expected(idx)))<=1e-6);
check('three_raw_reference_streams_exact',max(abs(r.raw_LSTM_A-l.raw_prediction_A(idx)))<=1e-6 && max(abs(r.raw_GRU_A-g.raw_prediction_A(idx)))<=1e-6 && max(abs(r.analytical_reference_A-c.raw_reference_A(idx)))<=1e-6);
check('speed_acceleration_exact_alignment',max(abs(r.speed_mps-c.speed_mps(idx)))<1e-12 && max(abs(r.acceleration_mps2-c.acceleration_mps2(idx)))<1e-12);
check('initial_battery_SOC',abs(r.SOC(1)-v.battery_initial_SOC)<1e-12);check('battery_SOC_bounds',all(r.SOC>=0 & r.SOC<=1));
if hybrid
 check('initial_storage_states',abs(r.SC_voltage_V(1)-v.SC_initial_V)<1e-12 && r.iL_state_A(1)==0 && r.PI_integral_As(1)==0);
 check('SC_voltage_bounds',all(r.SC_voltage_V>=v.SC_min_V-1e-9 & r.SC_voltage_V<=v.SC_max_V+1e-9));
 check('SC_state_current_envelope',all(abs(r.iL_state_A)<=v.SC_current_limit_A+1e-9));
 check('shared_protected_command_envelope',all(abs(r.Isc_command_A(active))<=v.SC_current_limit_A+1e-9));
 check('duty_bounds',all(r.duty(active)>=v.duty_min-1e-12 & r.duty(active)<=v.duty_max+1e-12));
else
 check('battery_only_preserved',all(r.Psc_bus_W==0 & r.Isc_actual_A==0 & r.SC_voltage_V==0));
end
check('SC_bus_current_distinction',max(abs(r.Ibus_A(active)-r.duty(active).*r.Isc_actual_A(active)))<1e-9);
check('nonnegative_converter_loss',all(r.converter_loss_W(active)>=0));
check('converter_power_balance',max(abs(r.converter_balance_residual_W(active)))<=v.power_balance_tolerance_W);
check('system_power_balance',max(abs(r.system_balance_residual_W(active)))<=v.power_balance_tolerance_W);
energy=sum((r.Pbattery_W-r.Pdc_load_W-r.converter_loss_W).*r.interval_dt_s)-(r.SC_energy_J(end)-r.SC_energy_J(1))-(r.inductor_energy_J(end)-r.inductor_energy_J(1));
check('integrated_energy_balance',abs(energy)<=v.energy_balance_tolerance_J);
check('controller_state_logging',max(abs(state(:,1)-r.SC_voltage_V))<1e-12 && max(abs(state(:,2)-r.iL_state_A))<1e-12);
if hybrid
 e=r.Isc_command_A-r.iL_state_A;increment=v.dt_s*e(1:end-1);increment(r.integrator_frozen(1:end-1))=0;
 check('unchanged_conditional_integrator_update',max(abs(diff(r.PI_integral_As)-increment))<1e-10);
end
tag=mode;if ~hybrid,tag="BatteryOnly";end
assert(isfolder(out),'project:Output','Missing project case directory.');
write_trace(r,fullfile(out,'trace.csv'));
checks.cycle=repmat(cycle,height(checks),1);checks.mode=repmat(tag,height(checks),1);writetable(checks,fullfile(out,'case_checks.csv'));
audit=struct('cycle',cycle,'mode',tag,'samples',height(r),'elapsed_s',elapsed,'tests_passed',nnz(checks.passed),'tests_failed',nnz(~checks.passed),'max_power_residual_W',max(abs(r.system_balance_residual_W(active))),'energy_residual_J',energy,'fallback_runs',0);
assert(all(checks.passed),'project:PlantValidation','Integrated plant validation failed.');
fprintf('SIMULATION_PASS %s %s %d checks; energy residual %.9g J\n',cycle,tag,height(checks),energy);
close_system(model,0);
 function check(name,passed)
  checks=[checks;table(string(name),logical(passed),'VariableNames',{'test','passed'})];
 end
end
function data=matrix(ts)
data=squeeze(ts.Data);if size(data,1)~=numel(ts.Time),data=data.';end
end
function data=aligned(ts,t)
assert(numel(ts.Time)==numel(t) && max(abs(ts.Time-t))<1e-9,'project:TelemetryTime','Logged signals are misaligned.');data=matrix(ts);
end

function p = plant_parameters()
%plant_parameters Fixed, auditable reduced-order scenario for MATLAB R2025b.
% No run-time settings, base-workspace dependencies, or input-file writes.
p.root = fileparts(mfilename('fullpath'));
p.values = struct();
rows = cell(0,6);
add('mass_kg',1070,'kg','D','Declared model specification / paper Table 4','Reported gross-weight number selected as run mass; no passenger load added.');
add('rotating_mass_factor',1.04,'1','A','Paper Table 4','Applying this factor to inertial force is a declared project choice.');
add('effective_mass_kg',1.04*1070,'kg','B','1.04*1070, conditional implementation','Eq. (5) itself uses m, without this factor.');
add('rho_kg_m3',1.2,'kg/m^3','A','Paper Table 4','Air density.');
add('Cd',0.29,'1','A','Paper Table 4','Aerodynamic drag coefficient.');
add('Crr',0.012,'1','A','Paper Table 4','Rolling resistance coefficient.');
add('width_m',1.475,'m','B','Paper Table 4: 1475 mm / 1000','Width converted to SI.');
add('height_m',1.655,'m','B','Paper Table 4: 1655 mm / 1000','Height converted to SI.');
add('frontal_area_factor',0.8,'1','D','Declared model specification','Geometric silhouette estimate, not measured Nissan frontal area.');
add('Af_m2',0.8*1.475*1.655,'m^2','D','0.8*width*height','1.9529 m^2; derived from an engineering estimate.');
add('frontal_sensitivity_fraction',0.20,'1','D','Declared model specification','Also run Af at 80% and 120% of this estimate.');
add('g_mps2',9.81,'m/s^2','A','Paper text accompanying Eq. (6)','Gravity.');
add('road_angle_rad',0,'rad','D','Declared model specification','Flat road; not Figure 8 input interpretation.');
add('wind_mps',0,'m/s','D','Declared model specification','Zero wind.');
add('motor_rated_power_W',47000,'W','A','Paper Table 4','Wheel-power magnitude feasibility check only; no silent clipping.');
add('motor_rated_torque_Nm',195,'N*m','A','Paper Table 4','Retained as provenance only; torque feasibility not checked.');
add('gear_ratio',NaN,'1','C','Not supplied','No torque-limit validation or motor-speed reconstruction.');
add('loaded_wheel_radius_m',NaN,'m','C','Not supplied','No radius invented from the tire designation.');
add('gear_efficiency_low',0.8,'1','A','Paper Table 4','Published gear efficiency range, not an aggregate drivetrain measurement.');
add('gear_efficiency_high',0.9,'1','A','Paper Table 4','Upper range bound.');
add('eta_drive',0.85,'1','D','Declared model specification, informed by 0.8-0.9 range','Aggregate traction conversion assumption.');
add('eta_regen',0.65,'1','A','Paper Table 4 / plant model placement choice','Applied once to negative wheel power; no duplicate converter efficiency factor.');
add('auxiliary_power_W',0,'W','D','Declared model specification','No auxiliary specification was supplied.');
add('battery_chemistry',"Lithium-ion",'category','A','Paper implementation section','Nominal ideal-voltage model; no cell dynamics inferred.');
add('battery_nominal_V',350,'V','A','Paper Table 4','Ideal stiff voltage; no sag or OCV-SOC curve.');
add('battery_rated_Wh',20000,'Wh','A','Paper Table 4','20 kWh.');
add('battery_Q_Ah',20000/350,'Ah','B','20000 Wh / 350 V','Energy/voltage charge equivalent, not measured cell capacity.');
add('battery_initial_SOC',0.8,'fraction','D','Declared model specification','Identical paired initial battery state.');
add('battery_min_SOC',0,'fraction','D','Physical SOC range / Declared model specification','Fail if crossing bounds; do not clip SOC while retaining impossible power.');
add('battery_max_SOC',1,'fraction','D','Physical SOC range / Declared model specification','No current/charge efficiency assumptions beyond ideal coulomb counting.');
add('battery_resistance_ohm',NaN,'ohm','C','Not supplied','Not used; battery voltage sag is not represented.');
add('battery_OCV_SOC_curve',"unavailable",'V versus SOC','C','Not supplied','No lookup curve invented. Chemistry: lithium-ion per paper.');
add('Cbank_F',32,'F','D','Approved Scenario A / Declared model specification','Conditional complete-bank interpretation.');
add('series_units',130,'count','A','Paper Table 4','Unit identity unknown.');
add('assumed_unit_max_V',2.7,'V','D','Approved Scenario A assumption','Not an identified Nissan cell/module specification.');
add('reference_max_V',351,'V','B','130*2.7','Reference-law maximum, distinct from actual operating voltage.');
add('SC_min_V',175.5,'V','D','Declared model specification','Actual capacitor operating lower bound.');
add('SC_max_V',333.45,'V','D','Declared model specification','Actual capacitor operating upper bound.');
add('SC_initial_V',280.8,'V','D','Declared model specification','Charging headroom; differs from Eq. (16) initial implied maximum.');
add('SC_ESR_ohm',0,'ohm','D','plant model initial ideal model','Ideal capacitor terminals; not measured ESR.');
add('SC_current_limit_A',100,'A','D','Declared model specification','Symmetric reference limit and predictive current-state envelope.');
add('Vdc_V',350,'V','D','Stiff-bus implementation using nominal battery voltage','No independent dynamic bus capacitor.');
add('paper_Cdc_F',1800e-6,'F','A','Paper Table 4','Recorded but not dynamically represented with a stiff voltage source.');
add('L_H',1e-3,'H','D','plant model converter design assumption','Averaged coupling inductance.');
add('RL_ohm',0.01,'ohm','D','plant model converter design assumption','Only modeled converter dissipation.');
add('dt_s',0.001,'s','D','Declared model specification','Internal plant/controller step; source signals held on [t_k,t_(k+1)).');
add('duty_min',0.02,'fraction','D','Declared model specification','High-side bus-connection fraction.');
add('duty_max',0.98,'fraction','D','Declared model specification','No switching PWM generated.');
add('zeta',0.707,'1','D','Declared model specification','Current-loop damping design.');
add('wn_rad_s',100,'rad/s','D','Declared model specification','Natural frequency design.');
add('Kp_per_A',(2*0.707*100*1e-3-0.01)/350,'1/A','B','(2*zeta*wn*L-RL)/Vdc','Positive error reduces high-side duty and increases positive iL.');
add('Ki_per_As',(100^2*1e-3)/350,'1/(A*s)','B','wn^2*L/Vdc','Integral gain in d=dff-u.');
add('protection_guard_time_constants',5,'count','D','Numerical supervisor design','Anticipate voltage boundary by five inverse-wn intervals; not a physical voltage limit.');
add('SC_voltage_guard_V',100/32*(5/100+0.001),'V','D','Ilimit/C*(5/wn+dt)','0.159375 V anticipation band; actual operating limits remain 175.5/333.45 V.');
add('state_bound_tolerance',1e-9,'SI state unit','D','Numerical validation choice','Tolerance only; states are never clamped.');
add('power_balance_tolerance_W',1e-5,'W','D','Predeclared numerical acceptance','Check interval converter and whole-system power residuals.');
add('energy_balance_tolerance_J',1e-3,'J','D','Predeclared numerical acceptance','Cumulative stiff-bus/storage/inductor/loss accounting.');
add('step_test_duration_s',2,'s','D','Validation design','Separate positive and negative current tests.');
add('step_test_current_A',20,'A','D','Validation design','Below current/voltage constraints.');
add('step_tracking_tolerance_A',0.1,'A','D','Validation design','Mean actual current over final 0.2 s of step test.');
add('refinement_relative_tolerance',0.02,'fraction','D','Validation design','1 ms versus 0.5 ms energy and RMS-current comparisons.');
p.audit = cell2table(rows,'VariableNames',{'Parameter','Value','Unit','Status','Source','Explanation'});
for column=string(p.audit.Properties.VariableNames)
    p.audit.(column)=string(p.audit.(column));
end

    function add(name,value,unit,status,source,explanation)
        p.values.(name) = value;
        displayValue=string(value);
        if isnumeric(value), displayValue=string(sprintf('%.17g',value)); end
        rows(end+1,:) = {name,displayValue,unit,status,source,explanation};
    end
end

function c = load_drive_cycle(name,p)
%load_drive_cycle Read approved project inputs and printed Scenario A labels.
arguments
    name (1,1) string {mustBeMember(name,["EUDC","IM240"])}
    p (1,1) struct = plant_parameters()
end
base = readtable(fullfile(p.root,'Data',name+"_processed.csv"),'VariableNamingRule','preserve');
ref = readtable(fullfile(p.root,'Data','Analytical_Reference',name+"_reference_scenario_A.csv"),'VariableNamingRule','preserve');
expected = 401; if name=="IM240", expected=241; end
assert(height(base)==expected && height(ref)==expected,'project:InputCount','Unexpected source row count.');
fields = {'time_s','speed_mps','acceleration_mps2'};
for j=1:numel(fields)
    assert(isequal(base.(fields{j}),ref.(fields{j})),'project:InputAlignment','project and 2 inputs differ.');
end
assert(all(diff(base.time_s)==1),'project:InputTime','Expected approved 1 s source timestamps.');
domain=lower(string(ref.mathematical_domain_feasible));
assert(all(domain=="true" | domain=="1"),'project:InputDomain','Scenario A must be feasible.');
assert(all(isfinite(ref.Isc_printed_A)),'project:InputTarget','Nonfinite printed references.');
assert(all(isfinite(base{:,fields}),'all'),'project:InputFinite','Nonfinite input data.');
c = struct('name',name,'time_s',base.time_s,'speed_mps',base.speed_mps, ...
    'acceleration_mps2',base.acceleration_mps2,'raw_reference_A',ref.Isc_printed_A);
% No acceleration regeneration, filtering, derived-law substitution, or ML call.
end

function [s,check]=load_prediction(root,cycle,architecture)
%load_prediction Validate every imported physical-unit sample; never substitute.
stem=fullfile(root,'Data','ML_Predictions',architecture+"_"+cycle+"_prediction");
assert(isfile(stem+".mat") && isfile(stem+".csv"),'project:MissingPrediction','Missing prediction trace.');
s=load(stem+".mat");r=readtable(stem+".csv",'TextType','string');
source=fullfile(root,'Data',cycle+"_processed.csv");x=readtable(source);
meta=jsondecode(fileread(fullfile(root,'ML_Assets',architecture,'metadata.json')));
info=jsondecode(s.metadata_json);n=401;if cycle=="IM240",n=241;end
assert(height(x)==n && height(r)==n && numel(s.raw_prediction_A)==n,'project:PredictionCount','Prediction sample count mismatch.');
assert(isequal(s.time_s(:),(0:n-1)') && max(abs(r.time_s-s.time_s))<1e-12,'project:PredictionTime','Prediction timestamps mismatch.');
assert(all(isfinite([s.speed_mps(:);s.acceleration_mps2(:);s.raw_prediction_A(:)])),'project:NonfinitePrediction','Nonfinite prediction data.');
assert(all(isfinite([r.time_s;r.speed_mps;r.acceleration_mps2;r.raw_prediction_A])),'project:NonfinitePrediction','Nonfinite CSV prediction data.');
assert(max(abs(r.speed_mps-s.speed_mps))<1e-12 && max(abs(r.acceleration_mps2-s.acceleration_mps2))<1e-12,'project:PredictionInput','CSV and MAT input features differ.');
assert(max(abs(x.speed_mps-s.speed_mps))<1e-12 && max(abs(x.acceleration_mps2-s.acceleration_mps2))<1e-12,'project:PredictionInput','Approved input alignment mismatch.');
assert(strcmp(info.cycle,cycle) && strcmp(info.architecture,architecture) && strcmp(info.model_identifier,"final_"+architecture));
assert(strcmp(info.output_unit,'A') && info.sample_spacing_s==1 && isequal(string(info.feature_order(:)),["speed_mps";"acceleration_mps2"]));
assert(strcmp(s.output_unit,'A') && s.sample_spacing_s==1 && isequal(string(info.feature_units(:)),["m/s";"m/s^2"]),'project:PredictionUnits','Physical-unit metadata mismatch.');
assert(strcmp(info.input_sha256,asset_sha256(source)) && strcmp(info.weights_sha256,meta.weights_sha256) && strcmp(info.scaler_sha256,meta.scaler_sha256));
assert(strcmp(info.metadata_sha256,asset_sha256(fullfile(root,'ML_Assets',architecture,'metadata.json'))));
assert(strcmp(meta.weights_sha256,asset_sha256(fullfile(root,'ML_Assets',architecture,'weights.pt'))));
assert(strcmp(meta.scaler_sha256,asset_sha256(fullfile(root,'ML_Assets',architecture,'scaler.json'))));
delta=max(abs(r.raw_prediction_A-s.raw_prediction_A));assert(delta<=1e-6,'project:TransferParity','Prediction transfer exceeded 1e-6 A.');
check=table(cycle,architecture,n,delta,1e-6,true,'VariableNames',{'cycle','architecture','samples','max_transfer_error_A','tolerance_A','passed'});
end

function h=asset_sha256(file)
%asset_sha256 Stream a file without modifying it.
md=java.security.MessageDigest.getInstance('SHA-256');f=fopen(file,'rb');assert(f>=0);clean=onCleanup(@()fclose(f));
while ~feof(f),b=fread(f,1024*1024,'*uint8');md.update(typecast(b,'int8'));end
b=typecast(md.digest(),'uint8');h=lower(reshape(dec2hex(b,2).',1,[]));
end

function write_trace(r,file)
%write_trace Preserve every binary64 telemetry value in decimal CSV.
f=fopen(file,'w');assert(f>=0,'project:TraceFile','Cannot open trace output.');cleanup=onCleanup(@()fclose(f));
fprintf(f,'%s\n',strjoin(r.Properties.VariableNames,','));format=[repmat('%.17g,',1,width(r)-1),'%.17g\n'];
for first=1:10000:height(r)
 rows=first:min(first+9999,height(r));values=table2array(r(rows,:));fprintf(f,format,double(values).');
end
end

