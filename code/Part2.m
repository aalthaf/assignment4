%% Part 2
% In this part ,we plot Vin and Vo by varying Vin as a step , sine function
% and a gaussian pulse. Then , the frequency response of each is plotted.

close all
clear

R1 = 1;
G1 = 1/R1;
c = 0.25;
R2 = 2;
G2 = 1/R2;
L = 0.2;
R3 = 10;
G3 = 1/R3;
a = 100;
R4 = 0.1;
G4 = 1/R4;
Ro = 1000;
Go = 1/Ro;

C = [0 0 0 0 0 0 0
    -c c 0 0 0 0 0
    0 0 -L 0 0 0 0
    0 0 0 0 0 0 0
    0 0 0 0 0 0 0
    0 0 0 0 0 0 0
    0 0 0 0 0 0 0];

G = [1 0 0 0 0 0 0
    -G2 G1+G2 -1 0 0 0 0
    0 1 0 -1 0 0 0
    0 0 -1 G3 0 0 0
    0 0 0 0 -a 1 0
    0 0 0 G3 -1 0 0
    0 0 0 0 0 -G4 G4+Go];
F_on = [1
    0
    0
    0
    0
    0
    0];

F_off = [0
    0
    0
    0
    0
    0
    0];



%With step function input
ts = 1000;
V1 = zeros(7, ts);
Vn_1 = zeros(7, 1);
dt = 1e-3;
for i = 1:ts
    % We solve the equations now and store in V1
    if i < 30
        V1(:,i) = (C./dt+G)\(F_off+C*Vn_1/dt);
    elseif i == 30
        V1(:,i) = (C./dt+G)\(F_on+C*Vn_1/dt);
    else
        V1(:,i) = (C./dt+G)\(F_on+C*Vold/dt);
    end
    Vold = V1(:, i);
    
end
figure(1)
plot(1:ts, V1(7,:), 'r')
hold on
plot(1:ts, V1(1,:), 'b')
title('Vin and Vout with 0.03 step')
xlabel('Time (ms)')
ylabel('Voltage (V)')
grid on

% Now with sine function input

V2 = zeros (7 , ts);
Fsine = zeros (7, 1);

for j= 1:ts
    Fsine(1,1) = sin(2*pi*(1/0.03)*j/ts);
    
    if j==1
        V2(:,j) = (C./dt+G)\(Fsine+C*Vn_1/dt);
    else
        V2(:,j) = (C./dt+G)\(Fsine+C*Vold/dt);
    end
    Vold = V2(:, j);
end

figure(2)
plot(1:ts, V2(7,:), 'r')
hold on
plot(1:ts, V2(1,:), 'b')
title('Vin and Vout with f = 1/0.03')
xlabel('Time (ms)')
ylabel('Voltage (V)')
grid on

% Now we try with a Gaussian pulse

V3 = zeros (7 ,ts);
Fgauss = zeros(7 , 1);

for z = 1:ts
    Fgauss(1,1) = exp (-1/2 * ((z/ts - 0.06)/0.06^2)^2);
    
    if z == 1
        V3 (:,z) = (C./dt+G)\(Fgauss+C*Vn_1/dt);
    else
        V3 (:,z) = (C./dt+G)\(Fgauss+C*Vold/dt);
    end
    Vold = V3(:,z);
end
figure(3)
plot(1:ts, V3(7,:), 'r');
hold on;
plot(1:ts, V3(1,:), 'b');
title('Vin and Vout with gaussian function');
xlabel('Time (ms)');
ylabel('Voltage (V)');
grid on;


% Frequency reponses for each of the inputs
f = (-ts/2:ts/2-1);             

% Frequency response for the step function input
fV1_in = fft(V1.');
fV1_out = fft(V1(7, :));
fsV1_in = fftshift(fV1_in);
fsV1_out = fftshift(fV1_out);
figure(4)
plot(f, abs(fsV1_in), 'r')
hold on
plot(f, abs(fsV1_out), 'b')
title('Vin and Vout in frequency domain with a step function')
xlabel('Frequency ')
ylabel('Voltage ')
grid on

% Frequency response for the sine function input
fV2 = fft(V2.');
fsV2 = fftshift(fV2);
figure(5)
plot(f, abs(fsV2(:, 1)), 'r');
hold on
plot(f, abs(fsV2(:, 7)), 'b');
title('Vin and Vout in frequency domain with a sin function')
xlabel('Frequency')
ylabel('Voltage ')
grid on

% Frequency response for the guassian function input
fV3 = fft(V3.');
fsV3 = fftshift(fV3);
figure(6)
plot(f, abs(fsV3(:, 1)), 'r')
hold on
plot(f, abs(fsV3(:, 7)), 'b')
title('Vin and Vout in frequency domain with a Gaussian function')
xlabel('Frequency')
ylabel('Voltage')
grid on

