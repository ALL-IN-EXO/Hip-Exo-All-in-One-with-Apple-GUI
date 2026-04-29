function result = run_pf_imu_varspeed_demo_v8(rootPath, csvFileName, legLabel, varargin)
%RUN_PF_IMU_VARSPEED_DEMO_V8
%
% PF+DS intent estimator test on continuous IMU variable-speed hip data.
%
% Data expected in CSV columns:
%   Time_ms, imu_LTx, imu_RTx, imu_Lvel, imu_Rvel
%
% This version is designed for the new variable-speed dataset:
%   210 s to 280 s: speed transition 1.25 -> 1.75 -> 0.75
%
% Key point:
%   No acceleration observation is required.
%   The PF likelihood uses velocity error only by default:
%
%       log w_i = -etaV * (dq - A_i(q-qstar_i))^2
%                 - sign/mode consistency penalties
%
% Intent model:
%   dq_int = A * (q - qstar),     A < 0
%
% Human-side exoskeleton policy tested offline:
%   tau_exo^h = tau_gc(q) + B(c)*dq_int_hat + D(c)*(dq_int_hat - dq)
%
% where:
%   c(t) = clip_[0,1] int_{t-T}^{t} (dConf - |dq_int_hat-dq|) ds
%
% Author: OpenAI ChatGPT

%% ----------------------------- Parse inputs --------------------------- %%
p = inputParser;
p.addRequired('rootPath', @(x) ischar(x) || isstring(x));
p.addRequired('csvFileName', @(x) ischar(x) || isstring(x));
p.addRequired('legLabel', @(x) ischar(x) || isstring(x));

% Data selection
p.addParameter('timeWindow', [210 280], @(x) isnumeric(x) && numel(x)==2);
p.addParameter('timeColumn', 'Time_ms', @(x) ischar(x) || isstring(x));
p.addParameter('timeUnit', 'seconds', @(x) ischar(x) || isstring(x)); % 'seconds','milliseconds','auto'
p.addParameter('angleUnit', 'deg', @(x) ischar(x) || isstring(x));    % 'deg' or 'rad'
p.addParameter('velocityUnit', 'deg/s', @(x) ischar(x) || isstring(x)); % 'deg/s' or 'rad/s'
p.addParameter('useSyncColumns', false, @(x) islogical(x) || isnumeric(x));
p.addParameter('downsampleFactor', 1, @(x) isnumeric(x) && isscalar(x) && x >= 1);

% Preprocessing
p.addParameter('smoothAngleWindow', 5, @(x) isnumeric(x) && isscalar(x) && x >= 1);
p.addParameter('smoothVelocityWindow', 5, @(x) isnumeric(x) && isscalar(x) && x >= 1);
p.addParameter('useMeasuredVelocity', true, @(x) islogical(x) || isnumeric(x));

% Attractor prior
p.addParameter('useAutoQstarPrior', true, @(x) islogical(x) || isnumeric(x));
p.addParameter('qLowRangeDeg', [-65 -20], @(x) isnumeric(x) && numel(x)==2);
p.addParameter('qHighRangeDeg', [-20 15], @(x) isnumeric(x) && numel(x)==2);
p.addParameter('priorMarginRatio', 0.10, @(x) isnumeric(x) && isscalar(x) && x >= 0);

% PF options
p.addParameter('numParticles', 900, @(x) isnumeric(x) && isscalar(x) && x >= 20);
p.addParameter('Amin', -35.0, @(x) isnumeric(x) && isscalar(x) && x < 0);
p.addParameter('Amax', -0.1, @(x) isnumeric(x) && isscalar(x) && x < 0);
p.addParameter('sigmaA', 0.25, @(x) isnumeric(x) && isscalar(x) && x >= 0);
p.addParameter('sigmaQstar', 0.004, @(x) isnumeric(x) && isscalar(x) && x >= 0);
p.addParameter('pSwitch', 0.003, @(x) isnumeric(x) && isscalar(x) && x >= 0 && x <= 1);
p.addParameter('switchEpsDeg', 2.5, @(x) isnumeric(x) && isscalar(x) && x >= 0);
p.addParameter('switchVelThreshDeg', 10.0, @(x) isnumeric(x) && isscalar(x) && x >= 0);

% Likelihood. No acceleration in this dataset, so etaA is set to 0 by default.
p.addParameter('etaV', 3.0, @(x) isnumeric(x) && isscalar(x) && x >= 0);
p.addParameter('etaA', 0.0, @(x) isnumeric(x) && isscalar(x) && x >= 0); % kept for compatibility, not used unless ddq enabled
p.addParameter('etaSign', 2.0, @(x) isnumeric(x) && isscalar(x) && x >= 0);

% Confidence
p.addParameter('dConf', 1.5, @(x) isnumeric(x) && isscalar(x) && x > 0);
p.addParameter('confWindowSec', 0.50, @(x) isnumeric(x) && isscalar(x) && x > 0);

% Controller policy:
% tau_exo^h = tau_gc + B(c)*dq_int_hat + D(c)*(dq_int_hat-dq)
p.addParameter('bMax', 2.0, @(x) isnumeric(x) && isscalar(x) && x >= 0);
p.addParameter('dMax', 1.0, @(x) isnumeric(x) && isscalar(x) && x >= 0);
p.addParameter('bGamma', 1.0, @(x) isnumeric(x) && isscalar(x) && x > 0);
p.addParameter('dGamma', 1.0, @(x) isnumeric(x) && isscalar(x) && x > 0);
p.addParameter('tauMax', 25.0, @(x) isnumeric(x) && isscalar(x) && x > 0);

% Optional gravity compensation model:
% tau_gc(q) = alphaG * (gSin*sin(q) + gCos*cos(q) + gBias)
% By default it is disabled because hardware parameters are unknown.
p.addParameter('useGravityComp', false, @(x) islogical(x) || isnumeric(x));
p.addParameter('alphaG', 0.0, @(x) isnumeric(x) && isscalar(x));
p.addParameter('gSin', 0.0, @(x) isnumeric(x) && isscalar(x));
p.addParameter('gCos', 0.0, @(x) isnumeric(x) && isscalar(x));
p.addParameter('gBias', 0.0, @(x) isnumeric(x) && isscalar(x));

% Plot/save
p.addParameter('makePlots', true, @(x) islogical(x) || isnumeric(x));
p.addParameter('showModeMeans', false, @(x) islogical(x) || isnumeric(x));
p.addParameter('savePrefix', '', @(x) ischar(x) || isstring(x));
p.addParameter('rngSeed', 1, @(x) isnumeric(x) && isscalar(x));

p.parse(rootPath, csvFileName, legLabel, varargin{:});
opt = p.Results;

rootPath = char(string(rootPath));
csvFileName = char(string(csvFileName));
legLabel = upper(char(string(legLabel)));
assert(strcmp(legLabel,'L') || strcmp(legLabel,'R'), 'legLabel must be L or R.');
assert(opt.Amin < opt.Amax && opt.Amax < 0, 'Need Amin < Amax < 0.');

rng(opt.rngSeed);

csvPath = csvFileName;
if ~isfile(csvPath)
    csvPath = fullfile(rootPath, csvFileName);
end
assert(isfile(csvPath), 'CSV file not found: %s', csvPath);

fprintf('\n=== PF+DS IMU variable-speed demo v8 ===\n');
fprintf('CSV file       : %s\n', csvPath);
fprintf('Leg            : %s\n', legLabel);
fprintf('Time window    : [%.2f, %.2f] s\n', opt.timeWindow(1), opt.timeWindow(2));
fprintf('Acceleration likelihood etaA = %.3f (0 means no acceleration observation)\n', opt.etaA);

%% ------------------------------- Load data ---------------------------- %%
TBL = readtable(csvPath);

timeCol = char(string(opt.timeColumn));
assert(ismember(timeCol, TBL.Properties.VariableNames), 'Time column %s not found.', timeCol);

tRaw = TBL.(timeCol);

timeUnit = lower(char(string(opt.timeUnit)));
switch timeUnit
    case 'seconds'
        t = tRaw;
    case 'milliseconds'
        t = tRaw / 1000;
    case 'auto'
        if max(tRaw) > 1000
            t = tRaw / 1000;
        else
            t = tRaw;
        end
    otherwise
        error('Unknown timeUnit: %s', timeUnit);
end

if logical(opt.useSyncColumns)
    qCol  = sprintf('sync_%sTx', legLabel);
    dqCol = sprintf('sync_%svel', legLabel);
else
    qCol  = sprintf('imu_%sTx', legLabel);
    dqCol = sprintf('imu_%svel', legLabel);
end

assert(ismember(qCol, TBL.Properties.VariableNames), 'Angle column %s not found.', qCol);
assert(ismember(dqCol, TBL.Properties.VariableNames), 'Velocity column %s not found.', dqCol);

qRaw = TBL.(qCol);
dqRaw = TBL.(dqCol);

idx = t >= opt.timeWindow(1) & t <= opt.timeWindow(2) & isfinite(t) & isfinite(qRaw) & isfinite(dqRaw);
t = t(idx);
q = qRaw(idx);
dqMeas = dqRaw(idx);

ds = round(opt.downsampleFactor);
if ds > 1
    t = t(1:ds:end);
    q = q(1:ds:end);
    dqMeas = dqMeas(1:ds:end);
end

% Re-zero time for plotting/numerics.
t = t(:);
t = t - t(1);
q = q(:);
dqMeas = dqMeas(:);

% Convert units.
if strcmpi(char(string(opt.angleUnit)), 'deg')
    q = deg2rad(q);
elseif strcmpi(char(string(opt.angleUnit)), 'rad')
    % do nothing
else
    error('Unknown angleUnit.');
end

if strcmpi(char(string(opt.velocityUnit)), 'deg/s')
    dqMeas = deg2rad(dqMeas);
elseif strcmpi(char(string(opt.velocityUnit)), 'rad/s')
    % do nothing
else
    error('Unknown velocityUnit.');
end

% Smooth angle and velocity mildly.
q = smooth_signal_local(q, opt.smoothAngleWindow);

if logical(opt.useMeasuredVelocity)
    dq = smooth_signal_local(dqMeas, opt.smoothVelocityWindow);
else
    qForDeriv = smooth_signal_local(q, max(opt.smoothAngleWindow, 5));
    dq = gradient(qForDeriv, t);
    dq = smooth_signal_local(dq, opt.smoothVelocityWindow);
end

dt = median(diff(t), 'omitnan');
fs = 1 / dt;

fprintf('Selected samples: %d, duration %.2f s, fs approx %.2f Hz\n', numel(t), t(end)-t(1), fs);
fprintf('Angle range     : [%.2f, %.2f] deg\n', rad2deg(min(q)), rad2deg(max(q)));
fprintf('Velocity range  : [%.2f, %.2f] deg/s\n', rad2deg(min(dq)), rad2deg(max(dq)));

%% ----------------------------- Build prior ---------------------------- %%
prior = build_qstar_prior_local(q, opt);

fprintf('\nQ-star prior\n');
fprintf('q_low range  : [%.3f, %.3f] rad = [%.1f, %.1f] deg\n', ...
    prior.qLowRange(1), prior.qLowRange(2), rad2deg(prior.qLowRange(1)), rad2deg(prior.qLowRange(2)));
fprintf('q_high range : [%.3f, %.3f] rad = [%.1f, %.1f] deg\n', ...
    prior.qHighRange(1), prior.qHighRange(2), rad2deg(prior.qHighRange(1)), rad2deg(prior.qHighRange(2)));
fprintf('A range      : [%.3f, %.3f] 1/s\n', opt.Amin, opt.Amax);

%% ------------------------------ Run PF -------------------------------- %%
pf = pf_hybrid_attractor_continuous_local(q, dq, t, prior, opt);

dqIntHat = pf.dqIntHat;
Ahat = pf.AHat;
qstarActive = pf.qstarActiveHat;
qstarHigh = pf.qstarHighHat;
qstarLow = pf.qstarLowHat;
modeProbHigh = pf.modeProbHigh;
modeHat = pf.modeHat;
ESS = pf.ESS;

%% ----------------------------- Metrics -------------------------------- %%
velErr = dq - dqIntHat;
velRMSE = sqrt(mean(velErr.^2));
velMAE = mean(abs(velErr));
signAgree = mean(sign_nonzero_local(dq) == sign_nonzero_local(dqIntHat));

dtVec = [diff(t); median(diff(t), 'omitnan')];
qOneStepPred = q + dtVec .* dqIntHat;
qNext = [q(2:end); q(end)];
oneStepErr = qNext - qOneStepPred;
oneStepRMSE = sqrt(mean(oneStepErr.^2));
oneStepMAE = mean(abs(oneStepErr));

% Continuous integration prediction for visualization, reset every N seconds
% to avoid drift dominating the plot.
resetIntervalSec = 3.0;
qRollPred = rolling_integrate_prediction_local(q, dqIntHat, t, resetIntervalSec);

fprintf('\nPF+DS intent metrics\n');
fprintf('Velocity RMSE             : %.4f rad/s = %.2f deg/s\n', velRMSE, rad2deg(velRMSE));
fprintf('Velocity MAE              : %.4f rad/s = %.2f deg/s\n', velMAE, rad2deg(velMAE));
fprintf('Velocity sign agreement   : %.2f %%\n', 100*signAgree);
fprintf('One-step position RMSE    : %.6f rad = %.4f deg\n', oneStepRMSE, rad2deg(oneStepRMSE));
fprintf('One-step position MAE     : %.6f rad = %.4f deg\n', oneStepMAE, rad2deg(oneStepMAE));

%% -------------------- Confidence and offline controller ---------------- %%
eV = abs(velErr);
confidence = compute_confidence_paper_window_local(eV, t, opt);

B = opt.bMax * confidence.^opt.bGamma;
D = opt.dMax * confidence.^opt.dGamma;

if logical(opt.useGravityComp)
    tauGC = opt.alphaG * (opt.gSin*sin(q) + opt.gCos*cos(q) + opt.gBias);
else
    tauGC = zeros(size(q));
end

tauFF = B .* dqIntHat;
tauDamp = D .* (dqIntHat - dq);
tauExo = tauGC + tauFF + tauDamp;
tauExoSat = min(max(tauExo, -opt.tauMax), opt.tauMax);

Pexo = tauExoSat .* dq;
Wpos = trapz(t, max(Pexo,0));
Wneg = trapz(t, max(-Pexo,0));
Wnet = trapz(t, Pexo);
posRatio = Wpos / max(Wpos+Wneg, eps);

fprintf('\nConfidence and proposed exoskeleton policy\n');
fprintf('c(t) range                : [%.3f, %.3f]\n', min(confidence), max(confidence));
fprintf('B(c) range                : [%.3f, %.3f]\n', min(B), max(B));
fprintf('D(c) range                : [%.3f, %.3f]\n', min(D), max(D));
fprintf('tau_ff range              : [%.3f, %.3f]\n', min(tauFF), max(tauFF));
fprintf('tau_damp range            : [%.3f, %.3f]\n', min(tauDamp), max(tauDamp));
fprintf('tau_exo saturated range   : [%.3f, %.3f]\n', min(tauExoSat), max(tauExoSat));
fprintf('P_exo positive work       : %.4f\n', Wpos);
fprintf('P_exo negative work mag.  : %.4f\n', Wneg);
fprintf('P_exo net work            : %.4f\n', Wnet);
fprintf('P_exo positive ratio      : %.4f\n', posRatio);

%% -------------------------------- Plot -------------------------------- %%
fig = [];
if logical(opt.makePlots)
    fig = figure('Color','w','Name',sprintf('PF+DS IMU variable speed %s leg', legLabel));
    set(fig, 'Position', [40 40 1500 950]);

    subplot(5,1,1);
    hold on;
    add_qstar_bands_local(gca, t, prior);
    plot(t, q, 'k', 'LineWidth', 1.1);
    plot(t, qstarActive, 'b', 'LineWidth', 1.0);
    if logical(opt.showModeMeans)
        plot(t, qstarHigh, '--', 'Color', [0.1 0.55 0.1], 'LineWidth', 0.75);
        plot(t, qstarLow, '--', 'Color', [0.85 0.33 0.10], 'LineWidth', 0.75);
        legend({'low qstar prior band','high qstar prior band','measured q','active qstar','high-mode qstar mean','low-mode qstar mean'}, ...
            'Location','eastoutside', 'Interpreter','none');
    else
        legend({'low qstar prior band','high qstar prior band','measured q','active qstar'}, ...
            'Location','eastoutside', 'Interpreter','none');
    end
    ylabel('Angle/target (rad)', 'Interpreter','none');
    title('Local attractor hypothesis: qstar is a target, not a q estimate', 'Interpreter','none');
    grid on;

    subplot(5,1,2);
    hold on;
    plot(t, dq, 'k', 'LineWidth', 1.0);
    plot(t, dqIntHat, 'b', 'LineWidth', 1.0);
    ylabel('Velocity (rad/s)', 'Interpreter','none');
    title(sprintf('Measured dq vs estimated intent velocity dq_int_hat | RMSE %.3f rad/s, sign %.1f%%', ...
        velRMSE, 100*signAgree), 'Interpreter','none');
    legend({'measured dq from IMU','dq_int_hat'}, 'Location','eastoutside', 'Interpreter','none');
    grid on;

    subplot(5,1,3);
    hold on;
    plot(t, rad2deg(q), 'k', 'LineWidth', 0.95);
    plot(t, rad2deg(qOneStepPred), '.', 'MarkerSize', 2);
    plot(t, rad2deg(qRollPred), 'b', 'LineWidth', 0.95);
    ylabel('Angle (deg)', 'Interpreter','none');
    title(sprintf('One-step prediction q_{k+1}=q_k+dt*dq_int_hat | RMSE %.4f deg', rad2deg(oneStepRMSE)), ...
        'Interpreter','none');
    legend({'measured q','one-step predicted next q','rolling integrated prediction'}, ...
        'Location','eastoutside', 'Interpreter','none');
    grid on;

    subplot(5,1,4);
    yyaxis left;
    plot(t, confidence, 'm', 'LineWidth', 1.0);
    ylabel('confidence', 'Interpreter','none');
    ylim([-0.05 1.05]);
    yyaxis right;
    plot(t, B, 'Color', [0.47 0.67 0.19], 'LineWidth', 1.0); hold on;
    plot(t, D, 'Color', [0.00 0.45 0.74], 'LineWidth', 1.0);
    ylabel('B(c), D(c)', 'Interpreter','none');
    xlabel('Time from 210s window start (s)', 'Interpreter','none');
    title('Paper-style confidence and confidence-dependent gains', 'Interpreter','none');
    legend({'confidence','B(c)','D(c)'}, 'Location','eastoutside', 'Interpreter','none');
    grid on;

    subplot(5,1,5);
    hold on;
    plot(t, tauGC, 'Color', [0.3 0.3 0.3], 'LineWidth', 0.9);
    plot(t, tauFF, 'Color', [0.47 0.67 0.19], 'LineWidth', 0.9);
    plot(t, tauDamp, 'Color', [0.00 0.45 0.74], 'LineWidth', 0.9);
    plot(t, tauExoSat, 'r', 'LineWidth', 1.0);
    yline(0,'k--');
    xlabel('Time from 210s window start (s)', 'Interpreter','none');
    ylabel('Torque proxy', 'Interpreter','none');
    title('Offline proposed human-side torque policy: tau_g + B(c)dq_int + D(c)(dq_int-dq)', ...
        'Interpreter','none');
    legend({'tau_g','B(c)dq_int','D(c)(dq_int-dq)','tau_exo^h'}, ...
        'Location','eastoutside', 'Interpreter','none');
    grid on;

    sgtitle(sprintf('PF+DS variable-speed IMU test | %s leg | %s', legLabel, csvFileName), ...
        'Interpreter','none');
end

%% -------------------------------- Save -------------------------------- %%
if strlength(string(opt.savePrefix)) > 0
    prefix = char(string(opt.savePrefix));
    if logical(opt.makePlots)
        saveas(fig, [prefix '_plots.png']);
    end
    save([prefix '_result.mat'], ...
        't','q','dq','dqIntHat','velErr','velRMSE','velMAE','signAgree', ...
        'qOneStepPred','qRollPred','oneStepErr','oneStepRMSE','oneStepMAE', ...
        'Ahat','qstarActive','qstarHigh','qstarLow','modeProbHigh','modeHat','ESS', ...
        'confidence','B','D','tauGC','tauFF','tauDamp','tauExo','tauExoSat','Pexo', ...
        'Wpos','Wneg','Wnet','posRatio','prior','opt','csvPath','legLabel');
end

%% ------------------------------- Output ------------------------------- %%
result = struct();
result.csvPath = csvPath;
result.legLabel = legLabel;
result.t = t;
result.q = q;
result.dq = dq;
result.dqIntHat = dqIntHat;
result.velErr = velErr;
result.velRMSE = velRMSE;
result.velMAE = velMAE;
result.signAgree = signAgree;
result.qOneStepPred = qOneStepPred;
result.qRollPred = qRollPred;
result.oneStepRMSE = oneStepRMSE;
result.oneStepMAE = oneStepMAE;
result.AHat = Ahat;
result.qstarActive = qstarActive;
result.qstarHigh = qstarHigh;
result.qstarLow = qstarLow;
result.modeProbHigh = modeProbHigh;
result.modeHat = modeHat;
result.ESS = ESS;
result.confidence = confidence;
result.B = B;
result.D = D;
result.tauGC = tauGC;
result.tauFF = tauFF;
result.tauDamp = tauDamp;
result.tauExo = tauExoSat;
result.Pexo = Pexo;
result.Wpos = Wpos;
result.Wneg = Wneg;
result.Wnet = Wnet;
result.posRatio = posRatio;
result.prior = prior;
result.opt = opt;
end

%% ============================= Local helpers ========================== %%
function prior = build_qstar_prior_local(q, opt)
if logical(opt.useAutoQstarPrior)
    qDeg = rad2deg(q(:));
    p01 = percentile_local(qDeg, 1);
    p45 = percentile_local(qDeg, 45);
    p55 = percentile_local(qDeg, 55);
    p99 = percentile_local(qDeg, 99);
    amp = max(p99 - p01, 1);
    margin = opt.priorMarginRatio * amp;

    qLowDeg = [p01 - margin, p45];
    qHighDeg = [p55, p99 + margin];

    if qLowDeg(1) >= qLowDeg(2)
        qLowDeg = [min(qDeg)-margin, median(qDeg)];
    end
    if qHighDeg(1) >= qHighDeg(2)
        qHighDeg = [median(qDeg), max(qDeg)+margin];
    end
else
    qLowDeg = opt.qLowRangeDeg;
    qHighDeg = opt.qHighRangeDeg;
end

prior.qLowRange = sort(deg2rad(qLowDeg(:)'));
prior.qHighRange = sort(deg2rad(qHighDeg(:)'));
prior.qLowNom = mean(prior.qLowRange);
prior.qHighNom = mean(prior.qHighRange);
end

function pf = pf_hybrid_attractor_continuous_local(q, dq, t, prior, opt)
q = q(:);
dq = dq(:);
t = t(:);

N = numel(q);
P = opt.numParticles;

particles = zeros(P,3); % [A, qstar, mode]; mode +1 high, -1 low
if dq(1) >= 0
    probHigh0 = 0.75;
else
    probHigh0 = 0.25;
end
mode = ones(P,1);
mode(rand(P,1) > probHigh0) = -1;

A0 = opt.Amin + (opt.Amax - opt.Amin) * rand(P,1);

qstar0 = zeros(P,1);
idxHigh = mode == 1;
idxLow  = mode == -1;
qstar0(idxHigh) = prior.qHighRange(1) + diff(prior.qHighRange) * rand(sum(idxHigh),1);
qstar0(idxLow)  = prior.qLowRange(1)  + diff(prior.qLowRange)  * rand(sum(idxLow),1);

particles(:,1) = A0;
particles(:,2) = qstar0;
particles(:,3) = mode;

weights = ones(P,1) / P;

switchEps = deg2rad(opt.switchEpsDeg);
switchVelThresh = deg2rad(opt.switchVelThreshDeg);

AHat = zeros(N,1);
qstarActiveHat = zeros(N,1);
qstarHighHat = zeros(N,1);
qstarLowHat = zeros(N,1);
modeProbHigh = zeros(N,1);
modeHat = zeros(N,1);
dqIntHat = zeros(N,1);
ESS = zeros(N,1);

for k = 1:N
    if k > 1
        % Random walk for A and qstar.
        particles(:,1) = clip_local(particles(:,1) + opt.sigmaA * randn(P,1), opt.Amin, opt.Amax);

        highIdx = particles(:,3) == 1;
        lowIdx  = particles(:,3) == -1;

        particles(highIdx,2) = clip_local(particles(highIdx,2) + opt.sigmaQstar * randn(sum(highIdx),1), ...
            prior.qHighRange(1), prior.qHighRange(2));
        particles(lowIdx,2) = clip_local(particles(lowIdx,2) + opt.sigmaQstar * randn(sum(lowIdx),1), ...
            prior.qLowRange(1), prior.qLowRange(2));

        % Mode switching:
        % high -> low near high target or when measured velocity clearly negative.
        % low  -> high near low target or when measured velocity clearly positive.
        swRandom = rand(P,1) < opt.pSwitch;

        highIdx = particles(:,3) == 1;
        lowIdx  = particles(:,3) == -1;

        swDown = highIdx & ( (q(k) >= particles(:,2) - switchEps) | (dq(k) < -switchVelThresh) );
        swUp   = lowIdx  & ( (q(k) <= particles(:,2) + switchEps) | (dq(k) >  switchVelThresh) );

        sw = swRandom | swDown | swUp;
        if any(sw)
            particles(sw,3) = -particles(sw,3);
            newHigh = sw & particles(:,3) == 1;
            newLow  = sw & particles(:,3) == -1;
            particles(newHigh,2) = prior.qHighRange(1) + diff(prior.qHighRange) * rand(sum(newHigh),1);
            particles(newLow,2)  = prior.qLowRange(1)  + diff(prior.qLowRange)  * rand(sum(newLow),1);
        end
    end

    A = particles(:,1);
    qstar = particles(:,2);
    mode = particles(:,3);

    dqPred = A .* (q(k) - qstar);

    wrongMode = (mode == 1 & dqPred < 0) | (mode == -1 & dqPred > 0);
    wrongSignObs = sign_nonzero_local(dq(k)) ~= sign_nonzero_local(dqPred);

    % No acceleration observation here.
    logw = -opt.etaV * (dq(k) - dqPred).^2 ...
           -opt.etaSign * double(wrongMode) ...
           -0.5 * opt.etaSign * double(wrongSignObs);

    logw = logw - max(logw);
    weights = weights .* exp(logw);
    weights = normalize_weights_local(weights);

    highMask = mode == 1;
    lowMask = mode == -1;
    pHigh = sum(weights(highMask));
    pLow = sum(weights(lowMask));

    modeProbHigh(k) = pHigh;
    if pHigh >= 0.5
        modeHat(k) = 1;
    else
        modeHat(k) = -1;
    end

    AHat(k) = sum(weights .* A);
    dqIntHat(k) = sum(weights .* dqPred);

    if pHigh > eps
        qstarHighHat(k) = sum(weights(highMask) .* qstar(highMask)) / pHigh;
    else
        qstarHighHat(k) = NaN;
    end

    if pLow > eps
        qstarLowHat(k) = sum(weights(lowMask) .* qstar(lowMask)) / pLow;
    else
        qstarLowHat(k) = NaN;
    end

    if modeHat(k) == 1
        qstarActiveHat(k) = qstarHighHat(k);
    else
        qstarActiveHat(k) = qstarLowHat(k);
    end

    ESS(k) = 1 / sum(weights.^2);

    if ESS(k) < 0.5 * P
        idx = systematic_resample_local(weights);
        particles = particles(idx,:);
        weights = ones(P,1)/P;
    end
end

pf = struct();
pf.AHat = AHat;
pf.qstarActiveHat = qstarActiveHat;
pf.qstarHighHat = qstarHighHat;
pf.qstarLowHat = qstarLowHat;
pf.modeProbHigh = modeProbHigh;
pf.modeHat = modeHat;
pf.dqIntHat = dqIntHat;
pf.ESS = ESS;
end

function confidence = compute_confidence_paper_window_local(eV, t, opt)
eV = eV(:);
t = t(:);
N = numel(eV);
confidence = zeros(N,1);

for k = 1:N
    idx0 = find(t >= t(k) - opt.confWindowSec, 1, 'first');
    if isempty(idx0)
        idx0 = 1;
    end
    tt = t(idx0:k);
    ee = eV(idx0:k);
    if numel(tt) >= 2
        cRaw = trapz(tt, opt.dConf - ee);
    else
        cRaw = 0;
    end
    confidence(k) = min(max(cRaw, 0), 1);
end
end

function qPred = rolling_integrate_prediction_local(q, dqHat, t, resetIntervalSec)
q = q(:);
dqHat = dqHat(:);
t = t(:);
N = numel(q);
qPred = zeros(N,1);
qPred(1) = q(1);
lastReset = t(1);

for k = 2:N
    if t(k) - lastReset >= resetIntervalSec
        qPred(k) = q(k);
        lastReset = t(k);
    else
        dt = t(k)-t(k-1);
        qPred(k) = qPred(k-1) + dt*dqHat(k-1);
    end
end
end

function y = smooth_signal_local(x, win)
x = x(:);
win = max(1, round(win));
if win <= 1
    y = x;
    return;
end
if mod(win,2)==0
    win = win + 1;
end
try
    y = smoothdata(x, 'movmean', win);
catch
    kernel = ones(win,1)/win;
    y = conv(x, kernel, 'same');
end
end

function pct = percentile_local(x, p)
x = sort(x(isfinite(x)));
if isempty(x)
    pct = NaN;
    return;
end
p = max(0, min(100, p));
pos = 1 + (numel(x)-1) * p/100;
lo = floor(pos);
hi = ceil(pos);
if lo == hi
    pct = x(lo);
else
    pct = x(lo) + (x(hi)-x(lo))*(pos-lo);
end
end

function weights = normalize_weights_local(weights)
weights = weights(:);
weights(~isfinite(weights)) = 0;
weights(weights < 0) = 0;
s = sum(weights);
if ~isfinite(s) || s <= 0
    weights = ones(numel(weights),1) / numel(weights);
else
    weights = weights / s;
end
end

function idx = systematic_resample_local(w)
w = normalize_weights_local(w);
N = numel(w);
positions = ((0:N-1)' + rand()) / N;
cumsumW = cumsum(w);
cumsumW(end) = 1;
idx = zeros(N,1);
j = 1;
for i = 1:N
    while j < N && positions(i) > cumsumW(j)
        j = j + 1;
    end
    idx(i) = j;
end
end

function s = sign_nonzero_local(x)
s = sign(x);
s(s==0) = 1;
end

function x = clip_local(x, xmin, xmax)
x = min(max(x, xmin), xmax);
end

function add_qstar_bands_local(ax, t, prior)
if isempty(t)
    return;
end
x1 = t(1);
x2 = t(end);

patch(ax, [x1 x2 x2 x1], ...
    [prior.qLowRange(1) prior.qLowRange(1) prior.qLowRange(2) prior.qLowRange(2)], ...
    [0.85 0.33 0.10], 'FaceAlpha', 0.08, 'EdgeColor', 'none');

patch(ax, [x1 x2 x2 x1], ...
    [prior.qHighRange(1) prior.qHighRange(1) prior.qHighRange(2) prior.qHighRange(2)], ...
    [0.47 0.67 0.19], 'FaceAlpha', 0.08, 'EdgeColor', 'none');

try
    uistack(findobj(ax,'Type','patch'),'bottom');
catch
end
end
