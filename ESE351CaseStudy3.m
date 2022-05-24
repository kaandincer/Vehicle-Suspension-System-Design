%% Setup and Generating Basic Graphs
close all;
load=1200; % Load from 0 to 1200kg
m=400+load/4; % Total mass per wheel
k=40000; % Spring constant
b=2*(k*m)^(1/2); % Dampin constant when zeta=1

zeta=b/(2*(k*m)^(1/2)); % Damping Ratio
wn=(k/m)^(1/2); % Natural Frequency
t = linspace(0,1.3,500); % Define time vector
A=[0 1;-wn^2 -2*zeta*wn];
B=[0;wn^2];
C=[1 0];
D=0;
sys=ss(A,B,C,D);
figure
impulse(sys)
figure
step(sys)
figure
pzmap(sys)

% Frequency Response
a = [1 2*zeta*wn wn^2];
b = [wn^2];
w = logspace(0,2);

h = freqs(b,a,w);
mag = 20*log10(abs(h));
phase = angle(h);
figure
subplot(2,1,1)
plot(w,mag)
set(gca,'XScale','log');
grid on
title("Magnitude, Zeta=" + zeta + ", Wn=" + wn)
xlabel('Frequency (rad/s)')
ylabel('Magnitude')

subplot(2,1,2)
semilogx(w,phase)
title('Phase')

grid on
xlabel('Frequency (rad/s)')
ylabel('Phase')


%% Generate road surfaces
totalDist = 50; % simulation distance in m
v = 15;         % Vehicle speed in m/s, between 0 and 40 m/s

N = 251; % number of samples in road surface

% road surface simulation
x = linspace(0,totalDist,N)'; % roadSurface sample spatial locations, in m
dx = totalDist/(N-1); % simulation interval in m

ts = x/v; % Time vector
dt = dx/v; % simulation interval in seconds
T = totalDist/v;    % Simulation length in seconds

%% Potholes

    % random road surface with potholes
    bumpiness = 10;            % Amplitude of road noise in cm
    pothole_depth = 25;        % Depth of potholes in cm
    pothole_width = 30; % 30;       % Width of potholes in cm
    pothole_locations = [20 30];      % location in m
    
    %Generate terrain
    %     roadSurface = generateTerrain(T, dt, v, bumpiness, pothole_depth, pothole_width, pothole_location);
    potholeSamp = round(pothole_width/(100*dx)); % pothole width in samples
    potholes = zeros(N,1);
    potholes(round(pothole_locations/dx)) = 1;
    potholes = conv(potholes,ones(potholeSamp,1),"same")*(-pothole_depth/100); % pothole model
    
    roadPothole = 2*(rand(N,1)-.5)*(bumpiness/100)+potholes;
    
    % start and end with smooth surface
    roadPothole(x<10) = 0;
    roadPothole(x>totalDist-10) = 0;
    
%% Trapezoidal Bump    
    roadTrap = zeros(N,1);
    bmpStart = 15; rampUp = 5; rampLength = 25;
    bmpEnd = bmpStart + rampLength;
    rampAmp = .5;
    rampUpDur = find(x>bmpStart & x <= bmpStart + rampUp);
    roadTrap(rampUpDur) = rampAmp*(0:1/length(rampUpDur):1-1/length(rampUpDur));
    roadTrap(x>=bmpStart+rampUp & x<=bmpEnd-rampUp)=rampAmp;
    rampDownDur = find(x>bmpEnd-rampUp & x<bmpEnd);
    roadTrap(rampDownDur) = rampAmp*(1-1/length(rampUpDur):-1/length(rampDownDur):0);
    
%% Sine Speed Bumps    
    spDist = 4; % 4; % spatial period of speed bump
    rippleHeight = 0.1; % height in m
    roadSin = rippleHeight*sin(2*pi*x/spDist);
    roadSin(x<10) = 0;
    roadSin(x>totalDist-10) = 0;
    

%% Custom Surface: Stairs
    customSurface=zeros(length(x),1);
    heights=[0 1 2 -4 2 1 -1 -4 4 0]/8;
    lengths=[30 25 15 25 30 30 25 15 25 31];
    
    for i=1:length(heights)
        if i==1
            customSurface(1:lengths(1))=heights(1)*ones(lengths(1),1);
        else
            customSurface((sum(lengths(1:(i-1)))+1):sum(lengths(1:i)))=+heights(i)*ones(lengths(i),1);
        end
   
    end
    
    % start and end with smooth surface
    customSurface(x<10) = 0;
    customSurface(x>totalDist-10) = 0;
    
    
  
%% Car Displacement on the Various Surfaces
lx=length(x);
figure
subplot(4,1,1)
hold on
PotHole= lsim([wn^2], [1 2*zeta*wn wn^2], roadPothole, ts); % Car Body Displacement
plot(ts,PotHole+ones(lx,1))
plot(ts,roadPothole)
title('Car Body Versus Road Surface Displacement (15 meters/sec)')
xlabel('Time(s)')
ylabel('Displacement(meters)')

subplot(4,1,2)
hold on
Sin= lsim([wn^2], [1 2*zeta*wn wn^2], roadSin, ts); % Car Body Displacement
plot(ts,Sin+ones(lx,1))
plot(ts,roadSin)
xlabel('Time(s)')
ylabel('Displacement(meters)')
anss=sum(abs(Sin));

subplot(4,1,3)
hold on;
Trap= lsim([wn^2], [1 2*zeta*wn wn^2], roadTrap, ts); % Car Body Displacement
plot(ts,Trap+ones(lx,1))
plot(ts,roadTrap)
hold off
xlabel('Time(s)')
ylabel('Displacement(meters)')

subplot(4,1,4)
hold on;
custom= lsim([wn^2], [1 2*zeta*wn wn^2], customSurface, ts); % Car Body Displacement
plot(ts,custom+ones(lx,1))
plot(ts,customSurface)
hold off
legend('Car Body','Road Surface')
xlabel('Time(s)')
ylabel('Displacement(meters)')

%% Animate Car
figure
figure
hold on
animateCar(Sin,roadSin,v,dt,T);  
hold off

