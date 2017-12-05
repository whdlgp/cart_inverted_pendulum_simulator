clear;
clc;
close all;

% �Ƶ��̳� ���̽�ƽ�� ����
% Arduino with joystick Y axis, analog pin 1
ANALOG_PIN = 'A1';
% Arduino Com port number
COM_NUMBER = 'COM5';

%declare arduino joystick
fprintf('connect arduino with port %s, analog pin %s\n\n', COM_NUMBER, ANALOG_PIN);
joystick = arduino(COM_NUMBER, 'UNO');

%1�� ���� 10������ �Է¹޾� ��ƽ�� �߰��� ������ ��հ� ���ؼ� ����ġ�� ���
fprintf('Joystick neutral position correction');
joystick_middle = 0;
for i = 1 : 10
    joystick_middle = joystick_middle + readVoltage(joystick, ANALOG_PIN);
    pause(0.1);
end
joystick_middle = joystick_middle/10;

% cart inverted pendulum �� �Ķ����
%M = .5;     % mass of the cart = 0.5(kg)
M = 0.2;     % mass of the cart = 0.2(kg)
m = 0.2;    % mass of the pendulum = 0.2(kg)
b = 0.1;    % friction of the cart = 0.1(N/m/sec)
i = 0.006;  % inertia of the pendulum = 0.006(kg*m^2)
g = 9.8;
%l = 0.3;   % length to the pendulum's center of mass = 0.3(m)
l = 5;      % length to the pendulum's center of mass = 5(m)
r = 1;    % �׸��׸��� pole �α� ����

% �� state space ���
p = i*(M+m)+M*m*l^2; %denominator for the A and B matricies

A = [0      1              0           0;
     0 -(i+m*l^2)*b/p  (m^2*g*l^2)/p   0;
     0      0              0           1;
     0 -(m*l*b)/p       m*g*l*(M+m)/p  0];

B = [     0; 
     (i+m*l^2)/p;
          0;
        m*l/p];

% �Է�
U = 0;

%�ùķ��̼� �ֱ�
H = 0.02;

% cart ��ġ, �ӵ�, ���ӵ�
pos_x = 0;
pos_x_dot = 0;
pos_x_dot_dot = 0;

% pendulum ������ ����, ���ӵ�, �����ӵ�
angle = 2*pi/180;
angle_dot = 0;
angle_dot_dot = 0;

% �ùķ��̼Ǹ��� plot�� ���ɸ�.
% ���� ���͹��� �ֱ� ����
display_count = 0;

% �ʱ� �� �׸���
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
    % analog joystick�� ���� �Է����� cart�� ������ �� ����
    U = 2*(readVoltage(joystick, ANALOG_PIN) - joystick_middle);
    
    % cart ���ӵ� �� pole �� �������� �����ӵ� ���ϱ�
    temp = A*[pos_x;pos_x_dot;angle;angle_dot] + U*B;

    pos_x_dot_dot = temp(2);
    angle_dot_dot = temp(4);
    
    % ���ӵ�, �����ӵ��� ���� �ӵ�, ���ӵ� ���ϱ�
    pos_x_dot = pos_x_dot + pos_x_dot_dot*H;
    angle_dot = angle_dot + angle_dot_dot*H;
    
    % �ӵ�, ���ӵ��� ���� ��ġ, ���� ���ϱ�
    pos_x = pos_x + pos_x_dot*H;
    angle = angle + angle_dot*H;
    
    %0.05�ʸ��� ��� ��� �׸���
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
    
    %������ �Ѿ����� ������
    if(abs(angle*180/pi) > 90)
        close all;
        break;
    end
    %figure �������� ����� ������
    if(abs(pos_x) > figure_x_max)
        close all;
        break;
    end
    
    pause(H);
end