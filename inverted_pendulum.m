clear;
clc;
close all;

% 아두이노 조이스틱과 연결
% Arduino with joystick Y axis, analog pin 1
ANALOG_PIN = 'A1';
% Arduino Com port number
COM_NUMBER = 'COM5';

%declare arduino joystick
fprintf('connect arduino with port %s, analog pin %s\n\n', COM_NUMBER, ANALOG_PIN);
joystick = arduino(COM_NUMBER, 'UNO');

%1초 동안 10번정도 입력받아 스틱이 중간에 있을때 평균값 구해서 보정치로 사용
fprintf('Joystick neutral position correction');
joystick_middle = 0;
for i = 1 : 10
    joystick_middle = joystick_middle + readVoltage(joystick, ANALOG_PIN);
    pause(0.1);
end
joystick_middle = joystick_middle/10;

% cart inverted pendulum 모델 파라미터
%M = .5;     % mass of the cart = 0.5(kg)
M = 0.2;     % mass of the cart = 0.2(kg)
m = 0.2;    % mass of the pendulum = 0.2(kg)
b = 0.1;    % friction of the cart = 0.1(N/m/sec)
i = 0.006;  % inertia of the pendulum = 0.006(kg*m^2)
g = 9.8;
%l = 0.3;   % length to the pendulum's center of mass = 0.3(m)
l = 5;      % length to the pendulum's center of mass = 5(m)
r = 1;    % 그림그릴때 pole 두깨 조절

% 모델 state space 행렬
p = i*(M+m)+M*m*l^2; %denominator for the A and B matricies

A = [0      1              0           0;
     0 -(i+m*l^2)*b/p  (m^2*g*l^2)/p   0;
     0      0              0           1;
     0 -(m*l*b)/p       m*g*l*(M+m)/p  0];

B = [     0; 
     (i+m*l^2)/p;
          0;
        m*l/p];

% 입력
U = 0;

%시뮬레이션 주기
H = 0.02;

% cart 위치, 속도, 가속도
pos_x = 0;
pos_x_dot = 0;
pos_x_dot_dot = 0;

% pendulum 기울어진 각도, 각속도, 각가속도
angle = 2*pi/180;
angle_dot = 0;
angle_dot_dot = 0;

% 시뮬레이션마다 plot시 랙걸림.
% 조금 인터벌을 주기 위함
display_count = 0;

% 초기 모델 그리기
grid on
hold on

x_cart=[-1,-1,1,1];
y_cart=[-0.5,0.5,0.5,-0.5];
cart = fill(x_cart, y_cart, 'g','EraseMode','normal');

x_pole = [0, 0];
y_pole = [0, l];
pole = plot(x_pole,y_pole,'b','EraseMode','normal');
set(pole, 'LineWidth',r*2);

hold off
figure_x_max = 20;
figure_y_max = 20;
axis([-figure_x_max figure_x_max -figure_y_max figure_y_max]);

for i = 1:10/H
    % analog joystick을 통해 입력으로 cart에 가해질 힘 받음
    U = 2*(readVoltage(joystick, ANALOG_PIN) - joystick_middle);
    
    % cart 가속도 및 pole 이 기울어지는 각가속도 구하기
    temp = A*[pos_x;pos_x_dot;angle;angle_dot] + U*B;

    pos_x_dot_dot = temp(2);
    angle_dot_dot = temp(4);
    
    % 가속도, 각가속도를 통해 속도, 각속도 구하기
    pos_x_dot = pos_x_dot + pos_x_dot_dot*H;
    angle_dot = angle_dot + angle_dot_dot*H;
    
    % 속도, 각속도를 통해 위치, 각도 구하기
    pos_x = pos_x + pos_x_dot*H;
    angle = angle + angle_dot*H;
    
    %0.05초마다 계산 결과 그리기
    display_count = display_count + 1;
    if display_count*H >= 0.05
        display_count = 0;
        
        x_cart = [pos_x-1,pos_x-1,pos_x+1,pos_x+1];
        set(cart,'Xdata',x_cart);
        
        x_pole = [pos_x, pos_x + l*sin(-angle)];
        y_pole = [0, l*cos(-angle)];
        set(pole,'Xdata',x_pole,'Ydata',y_pole);
        
        %fprintf('%f m\n%f degree\n\n', pos_x, angle*180/pi);
    end
    
    %완전히 넘어지면 나가기
    if(abs(angle*180/pi) > 90)
        close all;
        break;
    end
    %figure 범위에서 벗어나도 나가기
    if(abs(pos_x) > figure_x_max)
        close all;
        break;
    end
    
    pause(H);
end