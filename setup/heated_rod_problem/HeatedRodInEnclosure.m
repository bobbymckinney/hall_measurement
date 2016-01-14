% Heated Rod in an Enclosure
% Jon Rea
% 4-18-15

% Known Values
Tin = 500 ;%C
Twall = 30 ;%C
s = 0.015 ;%m
L = 0.5 ;%m
k = 5000 ;%W/mK
sigma = 5.67*10^-8 ;%W/m^2K^4
epsilon = 0.5 ;% (Emissivity of rod)

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

% Guess a temperature profile
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
        A(i,i-1) = k*s^2/dx ;
        A(i,i) = -2*k*s^2/dx ;
        A(i,i+1) = k*s^2/dx ;
        b(i) = sigma*epsilon*4*s*dx*((Tguess(i)+273)^4 - (Twall+273)^4) ;
    end

    % Node N
    A(N,N-1) = k*s^2/dx ;
    A(N,N) = -k*s^2/dx ;
    b(N) = sigma*epsilon*(4*s*dx+s^2)*((Tguess(i)+273)^4 - (Twall+273)^4) ;

    % Calculate Temperatures
    T = A\b ;
    Tguess = Tguess + (T-Tguess)*0.1 ;
end

% Plot Results
plot(x,T,'k')
xlabel('x (m)')
ylabel('Temperature (C)')
title('Rod Temperature Profile')
hold on