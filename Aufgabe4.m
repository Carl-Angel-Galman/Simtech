%% Frequenzgang des linearen Einspurmodells (0.05 Hz – 100 Hz) – gleiche Plotgröße, exakter Bereich

mdl = "linear_Einspurmodel";
load_system(mdl);

% Fahrzeuggeschwindigkeit
v = 18;
assignin("base","v",v);

A = 1; % Lenkwinkelamplitude [deg]

% Frequenzbereich in Hz (exakt)
f_min = 0.05;
f_max = 100;
f = logspace(log10(f_min), log10(f_max), 60);
w = 2*pi*f;  % rad/s

mag   = zeros(size(f));
phase = zeros(size(f));

for k = 1:length(w)
    wk = w(k);
    T  = 2*pi/wk;

    % Simulationszeit (mehrere Perioden)
    t_end = 12*T;
    t = linspace(0, t_end, 4000)';

    % Sinuslenkung
    u = A*sin(wk*t);
    u_ext = [t u];

    % Simulation
    simIn = Simulink.SimulationInput(mdl);
    simIn = simIn.setVariable("u_ext", u_ext);
    simIn = simIn.setVariable("v", v);
    simIn = simIn.setModelParameter( ...
        "LoadExternalInput","on", ...
        "ExternalInput","u_ext", ...
        "StopTime", num2str(t_end), ...
        "SignalLogging","on", ...
        "SignalLoggingName","logsout");

    simOut = sim(simIn);

    % Output beta
    beta_ts = simOut.logsout.getElement("beta").Values;
    y = beta_ts.Data;
    t = beta_ts.Time;

    % stationären Bereich auswerten
    idx = t > 6*T;
    t = t(idx);
    y = y(idx);

    % Sinusfit -> Betrag & Phase
    S = sin(wk*t);
    C = cos(wk*t);
    p = [S C]\y;

    A_out = hypot(p(1), p(2));
    phi   = atan2(p(2), p(1));

    mag(k)   = 20*log10(A_out/A);
    phase(k) = rad2deg(phi);
end

%% Plot-Layout (beide Figuren exakt gleich groß)
figPos = [100 100 900 520];   % [x y breite höhe] – beliebig, aber gleich für beide

% Verstärkung
figure('Position', figPos);
semilogx(f, mag, 'LineWidth', 1.5);
grid on;
xlim([f_min f_max]);          % Bereich exakt 0.05..100 Hz
xlabel('f [Hz]');
ylabel('Verstärkung [dB]');
title('Frequenzgang Betrag');

% Phase
figure('Position', figPos);
semilogx(f, phase, 'LineWidth', 1.5);
grid on;
xlim([f_min f_max]);          % Bereich exakt 0.05..100 Hz
xlabel('f [Hz]');
ylabel('Phase [deg]');
title('Frequenzgang Phase');