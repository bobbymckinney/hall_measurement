% Heated Tube in an Enclosure
% Bobby McKinney
% 2-5-16

% Known Values
Tin = 600 ;%C
Twall = 30 ;%C
r = .00644; %m
t = .0008; %m
L = 0.25 ;%m
k = 330 ;%W/mK
sigma = 5.67*10^-8 ;%W/m^2K^4
epsilon = 0.05 ;% (Emissivity of rod)

% Discretize the rod into nodes
N = 1000;% (Number of nodes)
x = zeros(N,1) ;% (Location of each node)
dx = L/(N-1) ;% (Distance between each node)
for i = 1:N
    x(i) = dx*(i-1) ;
end

% Initialize Matrix Inversion Method
A = zeros(N) ;
b = zeros(N,1) ;
T = zeros(N,1) ;

% Guess a temperature profil
Tguess = zeros(N,1) ;
for i  = 1:N
    Tguess(i) = Tin ;
end

% Run a while loop until the guess matches the calculated temperature
while sum(abs(Tguess-T)) > 0.01   
    % Node 1
    A(1,1) = 1 ;
    b(1) = Tin ;

    % Nodes i = 2:N-1
    for i = 2:N-1
        A(i,i-1) = k*pi*t*(2*r-t)/dx ;
        A(i,i) = -2*k*pi*t*(2*r-t)/dx ;
        A(i,i+1) = k*pi*t*(2*r-t)/dx ;
        b(i) = sigma*epsilon*2*pi*r*dx*((Tguess(i)+273)^4 - (Twall+273)^4) ;
    end

    % Node N
    A(N,N-1) = k*pi*t*(2*r-t)/dx ;
    A(N,N) = -k*pi*t*(2*r-t)/dx ;
    b(N) = sigma*epsilon*(2*pi*r*dx+pi*t*(2*r-t))*((Tguess(i)+273)^4 - (Twall+273)^4) ;

    % Calculate Temperatures
    T = A\b ;
    Tguess = Tguess + (T-Tguess)*0.1 ;
end

% Plot Results
plot(x,T,'k')
xlabel('x (m)')
ylabel('Temperature (C)')
title('Tube Temperature Profile')
hold on