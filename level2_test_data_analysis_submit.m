% 2010-09-07:   added open loop and closed loop comparison
%               removed the 'v6' plot option
%% TIME DOMAIN RESULT
level2_time_dom_result_const_freq.readme =...
    {'col 1: frequency in Hz',...
    'col 2: w/ compensation',...
    'special case: for global attenution, col 3 and 4 are the openloop and closedloop raw data'};
if ii == 1
    level2_time_dom_result_const_freq.y_data{1,1} = 'freq';
    level2_time_dom_result_const_freq.y_data{1,2} = 'openLoop';
    level2_time_dom_result_const_freq.y_data{1,3} = 'closedLoop';
end
if jj == 1
    level2_time_dom_result_const_freq.t_transient{ii,1} = ...
        num2str(NBf);
    
    level2_time_dom_result_const_freq.transient_norm_square{ii,1} = ...
        num2str(NBf);
    
    level2_time_dom_result_const_freq.maximum_transient{ii,1} =...
        num2str(NBf);
    
    level2_time_dom_result_const_freq.global_attenuation{ii,1} = ...
        num2str(NBf);
    
    level2_time_dom_result_const_freq.residual_norm_square{ii,1} = ...
        num2str(NBf);
    level2_time_dom_result_const_freq.y_data{ii+1,1} =...
        num2str(NBf);
else
    % 2012-09-02: new active suspension stops disturbance at 20s        
    t_distOff = 20;
    if ii == 1 % plot once
        [level2_time_dom_result_const_freq.t_transient{ii,2},...
            level2_time_dom_result_const_freq.maximum_transient{ii,2},...
            level2_time_dom_result_const_freq.transient_norm_square{ii,2}...
            ] = ....
            transient_duration_plot_disabled(...
            y.signals.values(1:t_distOff*Fs),Fs,t_distOff,t_NBon,'PlotOn');
%             y.signals.values,Fs,t_sim,t_NBon,'PlotOn');
        [level2_time_dom_result_const_freq.newTran(ii).norm_residual,...
            level2_time_dom_result_const_freq.newTran(ii).norm_after_transient,...
            level2_time_dom_result_const_freq.newTran(ii).ratio,...
            level2_time_dom_result_const_freq.newTran(ii).percentage] = ...
            transient_evaluation(y.signals.values(1:t_distOff*Fs),Fs);
    else
        [level2_time_dom_result_const_freq.t_transient{ii,2},...
            level2_time_dom_result_const_freq.maximum_transient{ii,2},...
            level2_time_dom_result_const_freq.transient_norm_square{ii,2}...
            ] = ....
            transient_duration_plot_disabled(...
            y.signals.values(1:t_distOff*Fs),Fs,t_distOff,t_NBon,'PlotOff');
        [level2_time_dom_result_const_freq.newTran(ii).norm_residual,...
            level2_time_dom_result_const_freq.newTran(ii).norm_after_transient,...
            level2_time_dom_result_const_freq.newTran(ii).ratio,...
            level2_time_dom_result_const_freq.newTran(ii).percentage] = ...
            transient_evaluation(y.signals.values(1:t_distOff*Fs),Fs);
%             y.signals.values,Fs,t_sim,t_NBon,'PlotOff');
    end
    % 3 second transient
    level2_time_dom_result_const_freq.transi_norm_square_3sec(ii,2) = ...
        sum(...
        y.signals.values( t_NBon/Ts+1 : (t_NBon+3)/Ts ).^2);
end
level2_time_dom_result_const_freq.y_data{ii+1,jj+1} = y;

if jj == 2
    temp = level2_time_dom_result_const_freq.y_data{ii+1,2};
    t_distOff = 20; % 2012-09-02
    y_ol = temp.signals.values(1:t_distOff*Fs);
    y_cl = y.signals.values(1:t_distOff*Fs);
    [global_attenuation, norm_square_ol_in_dB, norm_square_cl_in_dB] =...
        global_attenuation_additional_output(...
        y_ol,y_cl,...
        Fs,t_distOff,t_NBon);
    level2_time_dom_result_const_freq.global_attenuation{ii,2} = ...
        global_attenuation;
    level2_time_dom_result_const_freq.global_attenuation{ii,3} = ...
        norm_square_ol_in_dB;
    level2_time_dom_result_const_freq.global_attenuation{ii,4} = ...
        norm_square_cl_in_dB;
    level2_time_dom_result_const_freq.residual_norm_square{ii,2} = ...
    	db2mag(norm_square_cl_in_dB);
    h = figure;
    plot(y.time,y.signals.values);grid;
    % plot('v6',y.time,y.signals.values);grid;
    xlabel('Time [sec]');ylabel('Residual force [V]');
    figure_specific
    if SW_SAVE_DATA
        hgsave(h,fig_name_residule_time_trace,'-v6')
    end
end

h = figure(FIG_NUMBER2_CONST_DIST_FREQ(ii));grid on;hold on;
if jj == 1
    plot(y.time,y.signals.values,'r');
else
    plot(y.time,y.signals.values,'k:');
    if SW_CLOSE_LOOP
        legend('open loop','closed loop')
    else
        legend('w/o compensation','w/ compensation')
    end
    xlabel('Time [sec]');ylabel('Residual force [V]');
    figure_specific
    if SW_SAVE_DATA
        hgsave(h,['level2_time_trace_residule_',...
            num2str(freq_test1(ii)),'_',...
            num2str(freq_test2(ii)),...
            'Hz_compare'],'-v6')
    end
end
h = figure(FIG_NUMBER3_CONST_DIST_FREQ(ii));hold on;
if jj == 1
    subplot(211)
    plot(y.time,y.signals.values,'r');
    if ~FLAG_PERFORMANCE_EVAL
        legend 'Open loop';
    else
        legend 'Without compensation';
    end
    ylabel('Residual force [V]');
    figure_specific
else
    subplot(212)
    plot(y.time,y.signals.values,'k');
    if ~FLAG_PERFORMANCE_EVAL
        legend 'Closed loop';
    else
        legend 'With compensation';
    end
    xlabel('Time [sec]');ylabel('Residual force [V]');
    figure_specific
    if SW_SAVE_DATA
        hgsave(h,['level2_time_trace_residule_',...
            num2str(freq_test1(ii)),'_',...
            num2str(freq_test2(ii)),...
            'Hz_subplot_compare'],'-v6')
    end
end
%% PSD ANALYSIS
% calculate the psd after convergence
SW_LONG_FFT = 0;
if SW_LONG_FFT
    SPEC_CAL_REGION = 12/Ts:13/Ts;
    L = length(y.signals.values(SPEC_CAL_REGION));
    NFFT = 2^nextpow2(L); % Next power of 2 from length of y
else
    SPEC_CAL_REGION = 1:4/Ts; %11/Ts:14/Ts; 2012-08-26
    NFFT = 512;
end
[specY.f,specY.amp] = spectre_psd_rms(y.signals.values(SPEC_CAL_REGION),Fs,NFFT);
if ii == 1 && jj == 1
    level2_freq_dom_result_const_freq.y_psd{1,1} = 'freq';
    level2_freq_dom_result_const_freq.y_psd{1,2} = 'openLoop';
    level2_freq_dom_result_const_freq.y_psd{1,3} = 'closedLoop';
end
level2_freq_dom_result_const_freq.y_psd{ii+1,jj+1} = specY;
if jj == 1
    level2_freq_dom_result_const_freq.readme =...
        {'col 1: freq in Hz';...
        'col 2,3: psd at narrow band w/o attenuation';...
        'col 4,5: psd at narrow band w/ attenuation';...
        'for narrow_band_attenuation: col 1--first band; col 2--second band'};
    level2_freq_dom_result_const_freq.narrow_band_psd{ii,1} =...
        num2str(NBf);
end

if jj == 1
    level2_freq_dom_result_const_freq.narrow_band_psd{ii,1+(jj-1)*2+1} =...
        max(specY.amp(abs(specY.f - NBf(1))<2)); % 2010-09-26
    level2_freq_dom_result_const_freq.narrow_band_psd{ii,1+(jj-1)*2+2} =...
        max(specY.amp(abs(specY.f - NBf(2))<2)); % 2010-09-26
    
    freqMax1 = specY.f(...
        level2_freq_dom_result_const_freq.narrow_band_psd{ii,1+(jj-1)*2+1}==...
        specY.amp);
    
    freqMax2 = specY.f(...
        level2_freq_dom_result_const_freq.narrow_band_psd{ii,1+(jj-1)*2+2}==...
        specY.amp);
end
if jj == 2
    level2_freq_dom_result_const_freq.narrow_band_psd{ii,1+(jj-1)*2+1} =...
        specY.amp(specY.f==freqMax1); % 2010-09-26
    level2_freq_dom_result_const_freq.narrow_band_psd{ii,1+(jj-1)*2+2} =...
        specY.amp(specY.f==freqMax2); % 2010-09-26
end

if jj == 2
    level2_freq_dom_result_const_freq.narrow_band_attenuation{ii,1} =...
        level2_freq_dom_result_const_freq.narrow_band_psd{ii,2} - ...
        level2_freq_dom_result_const_freq.narrow_band_psd{ii,4};
    
    level2_freq_dom_result_const_freq.narrow_band_attenuation{ii,2} =...
        level2_freq_dom_result_const_freq.narrow_band_psd{ii,3} - ...
        level2_freq_dom_result_const_freq.narrow_band_psd{ii,5};
    
    specY_ol = level2_freq_dom_result_const_freq.y_psd{ii+1,2};
    
    level2_freq_dom_result_const_freq.psd_amplify{ii,1} =...
        num2str(NBf);
    level2_freq_dom_result_const_freq.psd_amplify{ii,3} =...
        max(specY.amp(10:end) - specY_ol.amp(10:end));
    %     the index freq of the maximum amplification
    level2_freq_dom_result_const_freq.psd_amplify{ii,2} =...
        specY.f(...
        (specY.amp - specY_ol.amp)==...
        level2_freq_dom_result_const_freq.psd_amplify{ii,3}...
        );
end

h = figure(FIG_NUMBER_CONST_DIST_FREQ(ii));
grid on; hold on;
if jj == 1
    plot(specY.f,specY.amp,'r')
    %     plot('v6',specY.f,specY.amp,'r')
else
    plot(specY.f,specY.amp,'k--')
    %     plot('v6',specY.f,specY.amp,'k--')
    
    xlabel('Frequency [Hz]')
    ylabel('dB [Vrms]')
    title('Spectral density of the plant output')
    
    legend('open loop','closed loop')
    if SW_SAVE_DATA
        hgsave(h,['level2_spectrum_',...
            num2str(freq_test1(ii)),'_',...
            num2str(freq_test2(ii)),...
            'Hz_compare'])
    end
end
%% PARAMETER ESTIMATION
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
    
    figure;% parameter convergence
    try
        if SW_SAMPLE_PLOT
 	if SW_EXPERIMENT == 1
	    plot(1:length(theta_hat.time),theta_hat.signals.values);
            xlabel('sample');
	else
            plot(1:length(theta_hat.time),theta_hat.signals.values,...
                1:length(theta_hat.time),theta1_true*plottime,':',...
                1:length(theta_hat.time),theta2_true*plottime,':');
            xlabel('sample');
	end
        else
	if SW_EXPERIMENT == 1
		plot(theta_hat.time,theta_hat.signals.values);
            xlabel('time [sec]');
	else
            plot(theta_hat.time,theta_hat.signals.values,...
                theta_hat.time,theta1_true*plottime,':',...
                theta_hat.time,theta2_true*plottime,':');
            xlabel('time [sec]');
	end
        end
    catch
    if SW_EXPERIMENT == 1
    	plot(theta_hat.time,theta_hat.signals.values);
        xlabel('time [sec]');
    else
        plot(theta_hat.time,theta_hat.signals.values,...
            theta_hat.time,theta1_true*plottime,':',...
            theta_hat.time,theta2_true*plottime,':');
        xlabel('time [sec]');
    end
    end
    ylabel('Estimated parameters')
    grid on;
    if SW_SAVE_DATA
        hgsave(['level2_para_converge_',...
            num2str(freq_test1(ii)),'_',...
            num2str(freq_test2(ii)),...
            'Hz'],'-v6')
    end
    
    figure;% frequency estimation
    try
        if SW_SAMPLE_PLOT
            plot(1:length(theta_hat.time),w1hat/2/pi,'r',...
                1:length(theta_hat.time),w2hat/2/pi,'k',...
                1:length(theta_hat.time),NBf(1)*plottime,':',...
                1:length(theta_hat.time),NBf(2)*plottime,':'...
                );
            xlabel('time (sec)');
        else
            plot(theta_hat.time,w1hat/2/pi,'r',...
                theta_hat.time,w2hat/2/pi,'k',...
                theta_hat.time,NBf(1)*plottime,':',...
                theta_hat.time,NBf(2)*plottime,':'...
                );
            xlabel('time (sec)');
        end
    catch
        plot(theta_hat.time,w1hat/2/pi,'r',...
            theta_hat.time,w2hat/2/pi,'k',...
            theta_hat.time,NBf(1)*plottime,':',...
            theta_hat.time,NBf(2)*plottime,':'...
            );
        xlabel('time (sec)');
    end
    ylabel('Estimated frequency (Hz)');
    grid on;
    if SW_SAVE_DATA
        hgsave(['level2_freq_converge_',...
            num2str(freq_test1(ii)),'_',...
            num2str(freq_test2(ii)),...
            'Hz'],'-v6')
    end
    
end