%% Aufgabe 4: Bode aus Simulation (alles in GRAD, Sweep in Hz) – Variante B (Fast Restart an, FixedStep konstant)
mdl = "linear_Einspurmodel";
load_system(mdl)

% Modellparameter
v = 18; % m/s
assignin('base','v', v);

% Sweep-Parameter (Frequenz in Hz)
AmpDeg = 1;        % Eingangsamplitude in GRAD
fmin   = 0.05;     % Hz
fmax   = 100;      % Hz
Npts   = 60;       % Anzahl Frequenzpunkte (logarithmisch)

fList = logspace(log10(fmin), log10(fmax), Npts);
wList = 2*pi*fList;   % rad/s

% Ergebnis
mag = nan(size(wList));     % Betrag |beta/delta| (deg/deg, einheitenfrei)
ph  = nan(size(wList));     % Phase in Grad

% ---- Fixed-step EINMAL festlegen (darf in Fast Restart nicht verändert werden!) ----
% Bei fmax=100 Hz ist Tmin=0.01 s. Mit 200 Punkten/Periode: dt=5e-5 s.
pointsPerPeriod = 200;
dtFixed = (1/fmax) / pointsPerPeriod;   % 5e-5 s


% Fast Restart einschalten
set_param(mdl, "FastRestart", "off");

set_param(mdl, ...
    "SolverType","Fixed-step", ...
    "Solver","ode4", ...
    "FixedStep", num2str(dtFixed));



for k = 1:numel(wList)
    w = wList(k);          % rad/s
    T = 2*pi/w;            % Periodendauer (s)

    % Simulationsdauer (Perioden)
    Ntotal = 20;
    Nskip  = 10;
    tStop  = Ntotal * T;

    % Zeitvektor mit FESTER Schrittweite
    t = (0:dtFixed:tStop)';

    % Eingang delta(t) in GRAD
    delta_deg = AmpDeg * sin(w*t);

    % External Input [t, u]
    u_ext = [t delta_deg];
    assignin('base','u_ext',u_ext);

    set_param(mdl, 'LoadExternalInput', 'on');
    set_param(mdl, 'ExternalInput', 'u_ext');

    % StopTime darf in FastRestart geändert werden
    simOut = sim(mdl, 'StopTime', num2str(tStop));

    % Output beta in GRAD aus logsout
    beta_ts  = simOut.logsout.getElement('beta').Values;  % timeseries
    beta_deg = beta_ts.Data(:);
    t_beta   = beta_ts.Time(:);

    % delta auf Zeitbasis von beta (zur Sicherheit)
    delta_deg_i = interp1(t, delta_deg, t_beta, 'linear', 'extrap');

    % Einschwingteil weg: ab t >= Nskip*T
    idx = t_beta >= (Nskip*T);
    te = t_beta(idx);
    de = delta_deg_i(idx);
    be = beta_deg(idx);

    % --- Amplitude & Phase per Sinus-Fit ---
    % Fit: y(t) = a*sin(w t) + b*cos(w t) + c
    X = [sin(w*te), cos(w*te), ones(size(te))];

    pD = X \ de;   % delta-fit
    pB = X \ be;   % beta-fit

    ampD = hypot(pD(1), pD(2));       % Amplitude in GRAD
    ampB = hypot(pB(1), pB(2));       % Amplitude in GRAD

    phiD = atan2(pD(2), pD(1));       % Phase in rad
    phiB = atan2(pB(2), pB(1));       % Phase in rad

    mag(k) = ampB / ampD;             % deg/deg -> einheitenfrei

    dphi = phiB - phiD;
    dphi = atan2(sin(dphi), cos(dphi)); % wrap [-pi,pi]
    ph(k) = dphi * 180/pi;            % Phase in Grad
end

% Fast Restart aus
set_param(mdl, "FastRestart", "off");

% Phase entwirren (optional schöner Plot)
ph_unwrap = unwrap(ph*pi/180) * 180/pi;

% Bode plots (x-Achse in Hz)
mag_dB = 20*log10(mag);

figure;
semilogx(fList, mag_dB, "LineWidth", 1.5); grid on;
xlabel('f (Hz)');
ylabel('Magnitude 20log_{10}(|\beta/\delta|) (dB)');
title('Bode aus Simulation (Grad-System): \beta/\delta');

figure;
semilogx(fList, ph_unwrap, "LineWidth", 1.5); grid on;
xlabel('f (Hz)');
ylabel('Phase(\beta) - Phase(\delta) (deg)');
title('Phase aus Simulation (Grad-System): \beta/\delta');
