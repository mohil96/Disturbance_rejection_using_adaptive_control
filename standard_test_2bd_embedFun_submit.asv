1% Adaptive multiple narrow-band disturbance rejection applied to an active
% suspension.
% 2 narrow band disturbance
% Benchmark project by Ioan Landau.
% ============================================================
%   Copyright (c) 2008-, Xu Chen
%   Author(s): Xu Chen
%              University of Washington
%              Seattle, WA, 98115
% ============================================================
% 2012-09-09
% profile on
clear all
close all
%% Define Constants
FLAG_CONST_DIST_FREQ        = 1;
SW_EXPERIMENT               = 0;
FLAG_STEP_CHANGE_DIST_FREQ  = 0;
FLAG_CHIRP_DIST             = 2;
SW_SAMPLE_PLOT              = 0;
SW_ONE_SIMU_TEST            = 1;
SW_DIST_ON                  = 1; % default turn on const or step change disturbance frequency
SW_CHIRP_DIST               = 0; % default turn off chirp disturbance
SW_TUNE                     = 0;
SW_INVERSE_TABLE            = 0;
SW_BASELINE_CONTROL_SYS     = 0; % check the baseline system
SW_ADDITIONAL_PLOT          = 0;
SW_STEADY_STATE_CONTROL_SYS = 0;
Fs=800;     Ts=1/Fs;        Te=Ts;

bode_opt            = bodeoptions;
bode_opt.FreqUnits  = 'Hz';
bode_opt.FreqScale  = 'Linear';
bode_opt.xlim       = [0 400];
bode_opt.PhaseWrapping = 'On';

distID.THRESHOLD = 5e-3;

%%
% SELECT TEST OPTIONS HERE each time the test is run:
%       FLAG_DIST_FREQ = 0 ------ step changing disturbance frequency
%       FLAG_DIST_FREQ = 1 ------ constant disturbance frequency
%       FLAG_DIST_FREQ = 2 ------ chirp disturbance
disp('=============Multiple Narrow Band Disturbance Rejection============')
disp('===================================================================')
disp('SELECT TEST OPTIONS:')
disp('0 (default) ---- step changing disturbance frequency')
disp('1           ---- constant disturbance frequency')
disp('2           ---- chirp disturbance')
disp('   ')
disp('Press ENTER for default selection.')
while 1
    FLAG_DIST_FREQ = input(':');
    if isempty(FLAG_DIST_FREQ)
        FLAG_DIST_FREQ      = 0;
    end
    if FLAG_DIST_FREQ ~= 0 && FLAG_DIST_FREQ ~= 1 && FLAG_DIST_FREQ ~= 2
        disp('Wrong selection. Please re-select.')
    else
        break;
    end
end % while 1
% in case nothing selected
if ~exist('FLAG_DIST_FREQ','var')
    FLAG_DIST_FREQ = FLAG_STEP_CHANGE_DIST_FREQ;
end

disp('===================================================================')
disp('CHOOSE THE TEST LENGTH:')
disp('1 (default) ---- a quick sample test')
disp('0           ---- the entire frequency profile specified by the benchmark')
disp('   ')
disp('Press ENTER for default selection.')
while 1
    SW_ONE_SIMU_TEST = input(':');
    if isempty(SW_ONE_SIMU_TEST)
        SW_ONE_SIMU_TEST      = 1;
    end
    if SW_ONE_SIMU_TEST ~= 0 && SW_ONE_SIMU_TEST ~= 1
        disp('Wrong selection. Please re-select.')
    else
        break;
    end
end

disp('===================================================================')
disp('CHOOSE WHETHER OR NOT TO SAVE THE TEST DATA.')
while 1
    SW_SAVE_DATA = input('Save the test result?\n   1(default, press ENTER): yes\n   0: no\n:');
    if isempty(SW_SAVE_DATA)
        SW_SAVE_DATA        = 1;
    end
    if SW_SAVE_DATA ~= 0 && SW_SAVE_DATA ~= 1
        disp('Wrong selection. Please re-select.')
    else
        break;
    end
end

disp('===================================================================')
disp('ADAPTATION SCHEME.')
while 1
    SW_UNIFORM_ADAP = input('Uniform adaptation gain? (more conservative performance)\n   1(default, press ENTER): yes\n   0: no\n:');
    if isempty(SW_UNIFORM_ADAP)
        SW_UNIFORM_ADAP        = 1;
    end
    if SW_UNIFORM_ADAP ~= 0 && SW_UNIFORM_ADAP ~= 1
        disp('Wrong selection. Please re-select.')
    else
        break;
    end
end
FLAG_PERFORMANCE_EVAL      = 0;
%%
NBn         = 2; % number of narrow bands
% chirp distrubance parameters
chirp_dist.level            = 2;
chirp_dist.freq1_seq        = [50, 70];
chirp_dist.freq2_seq        = [75, 95];
chirp_dist.para1            = [...
    chirp_dist.freq1_seq(1);...
    chirp_dist.freq1_seq(2);...
    80];
chirp_dist.para2            = [...
    chirp_dist.freq2_seq(1);...
    chirp_dist.freq2_seq(2);...
    95];

chirp_dist.freq1_init_time  = 5;
chirp_dist.chirp_init_time  = 10;
chirp_dist.chirp_dur_time   = 4;
chirp_dist.freq2_dur_time   = 5;

load band_pass_filter_50To95 % 2010-09-26
denoise_filter = tf(BP_ss_simulink)*tf(BP_ss_simulink);%2012-08-03
%% Adaptation parameters
% forgetting factor
adap_method = 2;
theta1_init =  -2*cos(72*2*pi*Ts) - 2*cos(72*2*pi*Ts);
theta2_init =  2+2*cos(72*2*pi*Ts) * 2*cos(72*2*pi*Ts);

F1          = 1000;
F2          = 1000;
F_init = [F1, 0; 0, F2];
theta_init  = [theta1_init; theta2_init];

% for exponentially increasing forgetting factor
lambda_init = 0.92;
lambda_end  = 0.99;
% Band-pass Q filter parameter
alpha_init  = 0.8; % 0.95
alpha_end   = 0.865;%470; 
alpha       = alpha_end;

adap_init.alpha_init    = alpha_init;
adap_init.alpha_end     = alpha_end;
adap_init.F             = F_init;
adap_init.theta         = theta_init;
adap_init.lambda_init   = lambda_init;
adap_init.lambda_end    = lambda_end;
adap_init.SW_lambda     = adap_method;
adap_init.SW_2Stage     = 0;
%% define_plant_controllers
%/////////// primary path sys tf
load model_prim.mat Bp Ap %numerator and denominator of the primary path
%/////////// closed loop R/S controller
load RS_contr_sec R S
%/////////// secondary path sys tf
load model_sec.mat B A %numerator and denominator of the secondary path

SW_newSSModel = 0;
if SW_newSSModel
    %     load hinf_inv_landau_201211new36
    %     load hinf_inv_landau_201211new34
    %     load hinf_inv_landau_201211new33
    load hinf_inv_landau_201211new32
    [numINVP,denINVP] = tfdata(invP,'v');
    % save hinf_inv_landau_coef_new numINVP denINVP
    %     load hinf_inv_landau_201211new19
else
    load hinf_inv_landau
end

P_inv = tf(invP);
%/////////// loading noise values
load bruitbench

if SW_BASELINE_CONTROL_SYS
    figure;
    bodeplot(tf(R,S,Ts),bode_opt)
    grid on,zoom on
    title('Frequency response of the feedback controller')
    % figure,bodeplot(tf(B,A,Ts),bode_opt),xlim([0,400])
end

if SW_BASELINE_CONTROL_SYS
    L       = length(bruitbench);
    NFFT    = 2^nextpow2(L);
    [spec_bruitbench.f,spec_bruitbench.amp] =...
        spectre_psd_rms(bruitbench,Fs,NFFT);
    figure;
    plot(spec_bruitbench.f,spec_bruitbench.amp)
    xlabel('Frequency [Hz]')
    ylabel('dB [Vrms]')
    title('Spectral density of the measurement noise')
end

if SW_newSSModel
    load NF98;
    load NF46;
else
    numNF46 = 1;
    denNF46 = 1;
    numNF98 = 1;
    denNF98 = 1;
    numNF98_2 = 1;
    denNF98_2 = 1;
end

simuName    = 'simulator_bench_2bd_simuSubmit';

%% Narrow band disturbances define
% Frequencies to be tested
if SW_ONE_SIMU_TEST
%     freq_test1   = 60;
%     freq_test2   = 90;
    
    freq_test1   = 50;
    freq_test2   = 75;
else
    freq_test1   = 50:5:75;
    freq_test2   = 70:5:95;
end % SW_ONE_SIMU_TEST
% Define the figure numbers
FIG_NUMBER_STEP_CHANGE_DIST     = [120;121;122];
FIG_NUMBER2_STEP_CHANGE_DIST    = [125;126;127];
FIG_NUMBER_CONST_DIST_FREQ      = 100:100-1+length(freq_test1);
FIG_NUMBER2_CONST_DIST_FREQ     = 150:150-1+length(freq_test1);
FIG_NUMBER3_CONST_DIST_FREQ     = 250:250-1+length(freq_test1);
FIG_NUMBER_CHIRP_DIST           = 200:202;
FIG_NUMBER2_CHIRP_DIST          = 300:302;

%% Run the test
if SW_UNIFORM_ADAP
    adap_init.SW_2Stage     = 1;
    adap_init.SW_lambda     = 0;
    lambda_end              = 1;%0.999;
    adap_init.lambda_end    = lambda_end;
        
    adap_init.alpha_pre     = 0.98;
    alpha_end   = 0.88;
    adap_init.alpha_end = alpha_end;
end
adap_init.lambda_gain = 280;
%% CONSTANT UNKONWN DISTURBANCE FREQUENCY
if FLAG_DIST_FREQ == FLAG_CONST_DIST_FREQ
    if ~SW_UNIFORM_ADAP
        lambda_end              = 1;%0.992;
        adap_init.lambda_end    = lambda_end;
        adap_init.SW_2Stage     = 1;
        adap_init.SW_lambda     = 0;
    end
    data_cont_freq.readme = 'stores data in the case of constant disturbance frequencies';
    data_cont_freq.y{1,1} = 'openLoop';
    data_cont_freq.y{1,2} = 'closedLoop';
    
    for ii = 1:length(freq_test1)
        NBw         = [freq_test1(ii)*2*pi;freq_test2(ii)*2*pi];
        % true parameters (for result comparision later)
        lb1true     = 2*cos(NBw(1)*Ts);
        lb2true     = 2*cos(NBw(2)*Ts);
        theta1_true = -(lb1true+lb2true);
        theta2_true = 2+lb1true*lb2true;
        % simulink parameter definition
        % Narrow band disturbance injection time
        t_NBon      = 5;
        % compensation turn on time
        t_Qon       = t_NBon;
        t_AdapOn    = t_NBon;
        % narrow band disturbance duration
        t_NBdur     = 15/5;
        t_AdapOff   = t_AdapOn+t_NBdur*5;
        % 2010-09-26; For disturbance generator v2
        t_dur_lastDist = t_NBdur;
        
        % freq in Hz
        NBf         = NBw/2/pi;
        % simulink time
        t_sim       = 30;
        f           = NBf;
        dist_seq1   = [NBf(1); NBf(2); 0];
        dist_seq2   = [NBf(1); NBf(2); 0];
        dist_seq3   = [NBf(1); NBf(2); 0];
        
        for jj = 1:2
            if jj == 1 % open loop
                % run the test without compensation
                SW_COMP_ON = 0; % compensation off
                SW_CLOSE_LOOP = 0;
                % define figure names for later use
                fig_name_spec_residule = ...
                    ['level2_spec_residule_',...
                    num2str(freq_test1(ii)),'_',...
                    num2str(freq_test2(ii)),...
                    'Hz_openLoop'];
                fig_name_residule_time_trace = ...
                    ['level2_time_trace_residule_',...
                    num2str(freq_test1(ii)),'_',...
                    num2str(freq_test2(ii)),...
                    'Hz_openLoop'];
            else
                % run the test with compensation
                SW_COMP_ON = 1; % compensation on
                SW_CLOSE_LOOP = 1;
                fig_name_spec_residule = ...
                    ['level2_spec_residule_',...
                    num2str(freq_test1(ii)),'_',...
                    num2str(freq_test2(ii)),...
                    'Hz_closedLoop'];
                fig_name_residule_time_trace = ...
                    ['level2_time_trace_residule_',...
                    num2str(freq_test1(ii)),'_',...
                    num2str(freq_test2(ii)),...
                    'Hz_closedLoop'];
            end
            %% simulation start
            sim(simuName)
            data_cont_freq.narrow_band_freq(ii,:) = NBf';
            data_cont_freq.y{ii+1,jj} = y;
            if ii == 1 && jj == 1
                if SW_SAVE_DATA
                    if ~exist(['level2_test_result_',date],'dir')
                        mkdir(['level2_test_result_',date]);
                    end
                end
            end
            level2_test_data_analysis_submit;
            if SW_SAVE_DATA
                try
                    movefile('*.fig',['level2_test_result_',date])
                catch
                end
            end
        end
        pause(10); % let the CPU take a 10-sec rest
    end
    disp ('===================================================================')
    disp ('test results saved to: level2_time_dom_result_const_freq')
    disp ('                       level2_freq_dom_result_const_freq')
    disp ('raw data saved to:     data_cont_freq')
    try
        if SW_SAVE_DATA
            save (['level2_test_result_',date,'\level2_time_dom_result_const_freq'],...
                'level2_time_dom_result_const_freq');
            save (['level2_test_result_',date,'\level2_freq_dom_result_const_freq'],...
                'level2_freq_dom_result_const_freq');
            save (['level2_test_result_',date,'\data_cont_freq'],...
                'data_cont_freq');
        end
    catch
    end
    
    %% STEP CHANGE DISTURBANCE FREQUENCY
elseif FLAG_DIST_FREQ == FLAG_STEP_CHANGE_DIST_FREQ
    if ~SW_UNIFORM_ADAP
        adap_init.SW_2Stage     = 1;
        adap_init.SW_lambda     = 2;
        
        lambda_end              = 0.999;
        adap_init.lambda_end    = lambda_end;
        
        adap_init.alpha_pre     = 0.98;
        alpha_end               = 0.88;
        adap_init.alpha_end     = alpha_end;
    end
    t_NBon      = 5;               % NB Dist injection time
    t_Qon       = t_NBon;          % bandpass Q filter on time
    t_AdapOn    = t_NBon;
    t_NBdur     = 3;
    t_AdapOff   = t_AdapOn+5*t_NBdur;
    t_sim       = 40;
    t_dur_lastDist = 3;
    
    % the center frequencies of the three step changing disturbance
    % frequencies
    center_freq = [55, 75; % each row defines the one experiment group
        70,90];
    % the three step changing disturbance frquencies (in Hz)
    %     experiment 1
    freq_seq1   = [55,60,55,50,55]; % data added in the example paper from Landau
    freq_seq2   = [75,80,75,70,75];
    %     experiment 2
    freq_seq3   = [70,75,70,65,70]; % data in the initial benchmark file
    freq_seq4   = [90,95,90,85,90];
    
    freq_table  = [freq_seq1; freq_seq2; freq_seq3; freq_seq4;];
    
    data_step_freq.readme = 'stores the data for the case of step changing disturbance frequencies';
    data_step_freq.y{1,1} = 'openLoop';
    data_step_freq.y{1,2} = 'closedLoop';
    
    if SW_ONE_SIMU_TEST % perform just one test
        ITER_STEP = 1;
        % the center frequencies of the three step changing disturbance
        % frequencies
        center_freq = [55,75];
        % the three step changing disturbance frquencies (in Hz)
        freq_seq1   = [55,60,55,50,55]; % data added in the example paper from Landau
        freq_seq2   = [75,80,75,70,75];
        freq_table  = [freq_seq1; freq_seq2];
    else
        ITER_STEP = 2;
    end
    for ii = 1:ITER_STEP
        % define the three sets of test sequences
        dist_seq1   = [center_freq(ii,1);   center_freq(ii,2);   0];
        dist_seq2   = [center_freq(ii,1)+5; center_freq(ii,2)+5; 0];
        dist_seq3   = [center_freq(ii,1)-5; center_freq(ii,2)-5; 0];
        
        for jj = 1:2
            if jj == 1 % open loop
                SW_COMP_ON = 0; % compensation off
                SW_CLOSE_LOOP = 0;
                fig_name_residule_time_trace = ...
                    ['level2_time_trace_residule_center_freq_',...
                    num2str(center_freq(ii,1)),'_',num2str(center_freq(ii,2)),...
                    'Hz_openLoop'];
            else
                SW_COMP_ON = 1; % compensation on
                SW_CLOSE_LOOP = 1;
                fig_name_residule_time_trace = ...
                    ['level2_time_trace_residule_center_freq_',...
                    num2str(center_freq(ii,1)),'_',num2str(center_freq(ii,2)),...
                    'Hz_closedLoop'];
            end
            %% simulation start
            sim(simuName)
            data_step_freq.initial_freq(ii,:) = dist_seq1';
            data_step_freq.y{ii+1,jj} = y;
            if ii == 1 && jj == 1
                if SW_SAVE_DATA
                    if ~exist(['level2_test_result_',date],'dir')
                        mkdir(['level2_test_result_',date]);
                    end
                end
            end
            %% time domain result
            if jj == 2
                level2_time_dom_result_step_change_freq.readme =...
                    {'row: each row represents one step-changing disturbance sequence',...
                    'col: from column 1 to column 5: transient for the 1st to 5th dist '};
                for kk = 1:5
                    level2_time_dom_result_step_change_freq.transi_norm_square_3sec(ii,kk) = ...
                        sum(...
                        y.signals.values(...
                        (t_NBon+(kk-1)*t_NBdur)/Ts + 1 :...
                        (t_NBon+(kk-1)*t_NBdur+3)/Ts ).^2);
                    
                    level2_time_dom_result_step_change_freq.max_residule(ii,kk) =...
                        max(...
                        y.signals.values(...
                        (t_NBon+(kk-1)*t_NBdur)/Ts :...
                        (t_NBon+(kk-1)*t_NBdur+1)/Ts )...
                        );
                    [temp_TD, temp_maximum, temp_transient_norm_square] =...
                        transient_duration_step_changes_additional_output(...
                        y.signals.values(1:t_AdapOff*Fs),...
                        Fs,t_AdapOff,t_NBon,t_NBdur,t_NBdur,'PlotOff',kk);
                    level2_time_dom_result_step_change_freq.t_transient(ii,kk) =...
                        temp_TD;
                    level2_time_dom_result_step_change_freq.transient_norm_square(ii,kk) =...
                        temp_transient_norm_square;
                    level2_time_dom_result_step_change_freq.maximum_transient(ii,kk) =...
                        temp_maximum;
                end
                h = figure;
                plot(y.time,y.signals.values);grid;
                xlabel('Time [sec]');ylabel('Residual force [V]');
                figure_specific
                if SW_SAVE_DATA
                    hgsave(h,fig_name_residule_time_trace,'-v6')
                end
            end
            
            h = figure(FIG_NUMBER_STEP_CHANGE_DIST(ii));
            grid on;hold on;
            if jj == 1
                plot(y.time,y.signals.values,'r');
            else
                plot(y.time,y.signals.values,'k:');
                
                legend('open loop','closed loop')
                xlabel('Time [sec]');ylabel('Residual force [V]');
                figure_specific
                if SW_SAVE_DATA
                    hgsave(h,['level2_time_trace_residule_center_freq_',...
                        num2str(center_freq(ii,1)),'_',num2str(center_freq(ii,2)),...
                        'Hz_compare'],'-v6')
                end
            end
            
            h = figure(FIG_NUMBER2_STEP_CHANGE_DIST(ii));
            hold on;
            if jj == 1
                subplot(211)
                plot(y.time,y.signals.values,'r');
                legend 'Open loop';
                ylabel('Residual force [V]');
                figure_specific
            else
                subplot(212)
                plot(y.time,y.signals.values,'k');
                legend 'Closed loop';
                xlabel('Time [sec]');ylabel('Residual force [V]');
                figure_specific
                if SW_SAVE_DATA
                    hgsave(h,['level2_time_trace_residule_center_freq_',...
                        num2str(center_freq(ii,1)),'_',num2str(center_freq(ii,2)),...
                        'Hz_subplot_compare'],'-v6')
                end
            end
            %% parameter convergence
            if jj == 2
                eta1vector = theta_hat.signals.values(:,1);%eta1 = -lambda1-lambda2
                eta2vector = theta_hat.signals.values(:,2);%eta2 = 2+lambda1*lambda2
                
                eta1 = eta1vector(end);
                eta2 = eta2vector(end);
                etaroot = sqrt(eta1vector.^2-4*(eta2vector-2));
                lb1vector = (-eta1vector - etaroot)/2;% lambda = 2*cos(w*Ts)
                lb2vector = (-eta1vector + etaroot)/2;
                w1hat = abs(acos(lb1vector/2)/Ts);% Frequency in rad/s
                w2hat = abs(acos(lb2vector/2)/Ts);
                
                plottime = ones(length(theta_hat.time),1);
                
                figure;
                try
                    if SW_SAMPLE_PLOT
                        plot(theta_hat.signals.values);
                        xlabel('sample');
                    else
                        plot(theta_hat.time,theta_hat.signals.values);
                        xlabel('time [sec]');
                    end
                catch
                    plot(theta_hat.time,theta_hat.signals.values);
                    xlabel('time [sec]');
                end
                ylabel('Estimated parameters')
                grid on;
                if SW_SAVE_DATA
                    hgsave(['level2_para_converge_step_change_center_',...
                        num2str(center_freq(ii,1)),'_',num2str(center_freq(ii,2)),...
                        'Hz'],'-v6')
                end
                
                % parameter convergence (frequency perspective)
                figure;
                try
                    if SW_SAMPLE_PLOT
                        plot(w1hat/2/pi,'r')
                        hold on
                        plot(w2hat/2/pi,'k')
                        xlabel('sample');
                    else
                        plot(theta_hat.time,w1hat/2/pi,'r',...
                            theta_hat.time,w2hat/2/pi,'k')
                        xlabel('time (sec)');
                    end
                catch
                    plot(theta_hat.time,w1hat/2/pi,'r',...
                        theta_hat.time,w2hat/2/pi,'k')
                    xlabel('time (sec)');
                end
                ylabel('Estimated frequency (Hz)');
                grid on;
                if SW_SAVE_DATA
                    hgsave(['level2_freq_converge_step_change_center_',...
                        num2str(center_freq(ii,1)),'_',num2str(center_freq(ii,2)),...
                        'Hz'],'-v6')
                end
                
                if SW_ADDITIONAL_PLOT
                    figure;
                    plot(Ffactor.time,Ffactor.signals.values);
                    ylabel('Forgetting factor');
                    xlabel('sec');
                end
            end
        end
        pause(10); % let the CPU take a 10-sec rest
    end
    level2_time_dom_result_step_change_freq.freq_table =...
        freq_table;
    disp ('===================================================================')
    disp ('test results saved to: level2_time_dom_result_step_change_freq')
    disp ('raw data saved to:     data_step_freq')
    if SW_SAVE_DATA
        try
            movefile('*.fig',['level2_test_result_',date])
        catch
        end
        save(['level2_test_result_',date,'\level2_time_dom_result_step_change_freq'],...
            'level2_time_dom_result_step_change_freq');
        save (['level2_test_result_',date,'\data_step_freq'],...
            'data_step_freq');
    end
    
    %% TEST FOR CHIRP DISTURBANCE
elseif FLAG_DIST_FREQ == FLAG_CHIRP_DIST
    SW_CHIRP_DIST = 1;
    SW_DIST_ON    = 0;
    if ~SW_UNIFORM_ADAP
        adap_init.alpha_pre     = 0.98;
        alpha_end               = 0.88;
        adap_init.alpha_end     = alpha_end;
        
        lambda_end              = 0.999;
        adap_init.lambda_end    = lambda_end;
        adap_init.SW_lambda     = 3;
        adap_init.SW_2Stage     = 1;
        if 0
            alpha_init = 0.93;
            alpha_end = 0.945; % 0.78;
        else
            alpha_init = 0.92;
            alpha_end = 0.92; % 0.78;
        end
        adap_init.alpha_init    = alpha_init;
        adap_init.alpha_end     = alpha_end;
        alpha                   = adap_init.alpha_end;
    end
    t_sim       = chirp_dist.chirp_init_time+chirp_dist.chirp_dur_time*2+chirp_dist.freq2_dur_time*2;
    t_NBon      = 5;
    t_Qon       = t_NBon;
    t_AdapOn    = t_NBon;
    t_AdapOff   = t_sim;
    t_dur_lastDist = 5;
    
    % for consistency
    t_NBdur     = 0;
    dist_seq1   = [50; 0; 0];
    dist_seq2   = [65; 0; 0];
    dist_seq3   = [80; 0; 0];
    
    adap_init.F = 4e4*[1, 0; 0, 1];
    
    data_chirp_freq.readme = 'stores the data for the case of chirp changing disturbance frequencies';
    data_chirp_freq.y{1,1} = 'openLoop';
    data_chirp_freq.y{1,2} = 'closedLoop';
    
    if SW_ONE_SIMU_TEST % perform just one test
        ITER_STEP = 1;
    else
        ITER_STEP = length(chirp_dist.freq1_seq)/2;
    end
    for ii = 1:ITER_STEP
        chirp_dist.para1 = [chirp_dist.freq1_seq((ii-1)*2+1);...
            chirp_dist.freq1_seq((ii-1)*2+2);...
            0];
        chirp_dist.para2 = [chirp_dist.freq2_seq((ii-1)*2+1);...
            chirp_dist.freq2_seq((ii-1)*2+2);...
            0];
        for jj = 1:2
            if jj == 1
                SW_COMP_ON = 0; % compensation off
                SW_CLOSE_LOOP = 0;
                fig_name_residule_time_trace = ...
                    ['level2_time_trace_residule_chirp_dist_',...
                    num2str(chirp_dist.freq1_seq(ii)),'To',...
                    num2str(chirp_dist.freq2_seq(ii)),...
                    '&',...
                    num2str(chirp_dist.freq1_seq(ii+1)),'To',...
                    num2str(chirp_dist.freq2_seq(ii+1)),...
                    'Hz_openLoop'];
            else
                SW_COMP_ON = 1; % compensation on
                SW_CLOSE_LOOP = 1;
                
                fig_name_residule_time_trace = ...
                    ['level2_time_trace_residule_chirp_dist_',...
                    num2str(chirp_dist.freq1_seq(ii)),'To',...
                    num2str(chirp_dist.freq2_seq(ii)),...
                    '&',...
                    num2str(chirp_dist.freq1_seq(ii+1)),'To',...
                    num2str(chirp_dist.freq2_seq(ii+1)),...
                    'Hz_closedLoop'];
            end
            %% open simulinnk
            sim(simuName)
            data_chirp_freq.initial_freq(ii,:) = chirp_dist.para1';
            data_chirp_freq.y{ii+1,jj} = y;
            if ii == 1 && jj == 1
                if SW_SAVE_DATA
                    if ~exist(['level2_test_result_',date],'dir')
                        mkdir(['level2_test_result_',date]);
                    end
                end
            end
            %% time domain result
            if jj == 2
                level2_time_dom_result_chirp_freq.readme =...
                    {'col 1: init chirp freq 1;',...
                    'col 2: end chirp freq 1;',...
                    'col 3: chirp increase freq',...
                    'col 4: chirp decrease freq',...
                    'col 5: init chirp freq 2;',...
                    'col 6: end chirp freq 2;',...
                    };
                
                level2_time_dom_result_chirp_freq.transient_norm(ii,1) = ...
                    chirp_dist.freq1_seq(ii);
                level2_time_dom_result_chirp_freq.transient_norm(ii,2) = ...
                    chirp_dist.freq2_seq(ii);
                level2_time_dom_result_chirp_freq.transient_norm(ii,3) = ...
                    sqrt(...
                    sum(...
                    y.signals.values(...
                    chirp_dist.chirp_init_time/Ts :...
                    (chirp_dist.chirp_init_time + chirp_dist.chirp_dur_time)/Ts ).^2)...
                    );
                level2_time_dom_result_chirp_freq.transient_norm(ii,4) = ...
                    sqrt(...
                    sum(...
                    y.signals.values(...
                    (chirp_dist.chirp_init_time + chirp_dist.chirp_dur_time + 5)/Ts :...
                    (chirp_dist.chirp_init_time + chirp_dist.chirp_dur_time + 10)/Ts ).^2)...
                    );
                level2_time_dom_result_chirp_freq.transient_norm(ii,5) = ...
                    chirp_dist.freq1_seq(ii+1);
                level2_time_dom_result_chirp_freq.transient_norm(ii,6) = ...
                    chirp_dist.freq2_seq(ii+1);
                
                level2_time_dom_result_chirp_freq.transient_norm_square(ii,1) = ...
                    chirp_dist.freq1_seq(ii);
                level2_time_dom_result_chirp_freq.transient_norm_square(ii,2) = ...
                    chirp_dist.freq2_seq(ii);
                level2_time_dom_result_chirp_freq.transient_norm_square(ii,3) = ...
                    level2_time_dom_result_chirp_freq.transient_norm(ii,3)^2;
                level2_time_dom_result_chirp_freq.transient_norm_square(ii,4) = ...
                    level2_time_dom_result_chirp_freq.transient_norm(ii,4)^2;
                level2_time_dom_result_chirp_freq.transient_norm_square(ii,5) = ...
                    chirp_dist.freq1_seq(ii+1);
                level2_time_dom_result_chirp_freq.transient_norm_square(ii,6) = ...
                    chirp_dist.freq2_seq(ii+1);
                
                level2_time_dom_result_chirp_freq.max_residule(ii,1) = ...
                    chirp_dist.freq1_seq(ii);
                level2_time_dom_result_chirp_freq.max_residule(ii,2) = ...
                    chirp_dist.freq2_seq(ii);
                level2_time_dom_result_chirp_freq.max_residule(ii,3) =...
                    max(y.signals.values(...
                    chirp_dist.chirp_init_time/Ts :...
                    (chirp_dist.chirp_init_time + chirp_dist.chirp_dur_time)/Ts ));
                level2_time_dom_result_chirp_freq.max_residule(ii,4) =...
                    max(y.signals.values(...
                    (chirp_dist.chirp_init_time + chirp_dist.chirp_dur_time + 5)/Ts :...
                    (chirp_dist.chirp_init_time + chirp_dist.chirp_dur_time + 10)/Ts ));
                level2_time_dom_result_chirp_freq.max_residule(ii,5) = ...
                    chirp_dist.freq1_seq(ii+1);
                level2_time_dom_result_chirp_freq.max_residule(ii,6) = ...
                    chirp_dist.freq2_seq(ii+1);
            end
            h = figure;
            plot(y.time,y.signals.values);grid;
            xlabel('Time [sec]');ylabel('Residual force [V]');
            figure_specific
            if SW_SAVE_DATA
                hgsave(h,fig_name_residule_time_trace,'-v6')
            end
            
            h = figure(FIG_NUMBER_CHIRP_DIST(ii));
            grid on;hold on;
            if jj == 1
                plot(y.time,y.signals.values,'r');
            else
                plot(y.time,y.signals.values,'k:');
                legend('open loop','closed loop')
                xlabel('Time [sec]');ylabel('Residual force [V]');
                figure_specific
                if SW_SAVE_DATA
                    hgsave(h,['level2_time_trace_residule_chirp_dist_',...
                        num2str(chirp_dist.freq1_seq(ii)),'To',...
                        num2str(chirp_dist.freq2_seq(ii)),...
                        '&',...
                        num2str(chirp_dist.freq1_seq(ii+1)),'To',...
                        num2str(chirp_dist.freq2_seq(ii+1)),...
                        'Hz_compare'],'-v6')
                end
            end
            
            h = figure(FIG_NUMBER2_CHIRP_DIST(ii));
            hold on;
            if jj == 1
                subplot(211)
                plot(y.time,y.signals.values,'r');
                legend 'Open loop';
                ylabel('Residual force [V]');
                figure_specific
            else
                subplot(212)
                plot(y.time,y.signals.values,'k');
                legend 'Closed loop';
                xlabel('Time [sec]');ylabel('Residual force [V]');
                figure_specific
                if SW_SAVE_DATA
                    hgsave(h,['level2_time_trace_residule_chirp_dist_',...
                        num2str(chirp_dist.freq1_seq(ii)),'To',...
                        num2str(chirp_dist.freq2_seq(ii)),...
                        '&',...
                        num2str(chirp_dist.freq1_seq(ii+1)),'To',...
                        num2str(chirp_dist.freq2_seq(ii+1)),...
                        'Hz_subplot_compare'],'-v6')
                end
            end
            %% parameter convergence
            if jj == 2
                eta1vector = theta_hat.signals.values(:,1);%eta1 = -lambda1-lambda2
                eta2vector = theta_hat.signals.values(:,2);%eta2 = 2+lambda1*lambda2
                
                eta1 = eta1vector(end);
                eta2 = eta2vector(end);
                etaroot = sqrt(eta1vector.^2-4*(eta2vector-2));
                lb1vector = (-eta1vector - etaroot)/2;% lambda = 2*cos(w*Ts)
                lb2vector = (-eta1vector + etaroot)/2;
                w1hat = abs(acos(lb1vector/2)/Ts);% Frequency in rad/s
                w2hat = abs(acos(lb2vector/2)/Ts);
                
                plottime = ones(length(theta_hat.time),1);
                
                figure;
                try
                    if SW_SAMPLE_PLOT
                        plot(theta_hat.signals.values);
                        xlabel('time [sec]');
                    else
                        plot(theta_hat.time,theta_hat.signals.values);
                        xlabel('time [sec]');
                    end
                catch
                    plot(theta_hat.time,theta_hat.signals.values);
                    xlabel('time [sec]');
                end
                ylabel('Estimated parameters')
                grid on;
                xlim([0,t_sim])
                if SW_SAVE_DATA
                    hgsave(['level2_para_converge_chirp_dist_',...
                        num2str(chirp_dist.freq1_seq(ii)),'To',...
                        num2str(chirp_dist.freq2_seq(ii)),...
                        '&',...
                        num2str(chirp_dist.freq1_seq(ii+1)),'To',...
                        num2str(chirp_dist.freq2_seq(ii+1)),...
                        'Hz'],'-v6')
                end
                
                % para convergence (frequency perspective)
                figure;
                try
                    if SW_SAMPLE_PLOT
                        plot(w1hat/2/pi,'r')
                        hold on;
                        plot(w2hat/2/pi,'k');
                        xlabel('sample');
                    else
                        plot(theta_hat.time,w1hat/2/pi,'r',...
                            theta_hat.time,w2hat/2/pi,'k');
                        xlabel('time (sec)');
                    end
                catch
                    plot(theta_hat.time,w1hat/2/pi,'r',...
                        theta_hat.time,w2hat/2/pi,'k');
                    xlabel('time (sec)');
                end
                ylabel('Estimated frequency (Hz)');
                grid on;
                if SW_SAVE_DATA
                    hgsave(['level2_freq_converge_chirp_dist_',...
                        num2str(chirp_dist.freq1_seq(ii)),'To',...
                        num2str(chirp_dist.freq2_seq(ii)),...
                        '&',...
                        num2str(chirp_dist.freq1_seq(ii+1)),'To',...
                        num2str(chirp_dist.freq2_seq(ii+1)),...
                        'Hz'],'-v6')
                end
            end
        end
        pause(10); % let the CPU take a 10-sec rest
    end
    disp ('===================================================================')
    disp ('test results saved to: level2_time_dom_result_chirp_freq')
    disp ('raw data saved to:     data_chirp_freq')
    if SW_SAVE_DATA
        try
            movefile('*.fig',['level2_test_result_',date])
        catch
        end
        save (['level2_test_result_',date,'\level2_time_dom_result_chirp_freq'],...
            'level2_time_dom_result_chirp_freq');
        save (['level2_test_result_',date,'\data_chirp_freq'],...
            'data_chirp_freq');
    end
end