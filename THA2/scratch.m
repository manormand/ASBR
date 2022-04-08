clear; clc

x = [0.15 1.04 1.44 1.84 2.24 2.64 3.04 3.44 3.84 4.24];
y = [7.5 5.6 4.4 3.6 3.0 2.5 2.2 1.9 1.5 1.1];

figure()
plot(x,y,'ko')
hold on;

P = polyfit(x,y,1)

y_new = P(1)*x + P(2);
% y_new = P(1)*x.^2 + P(2)*x + P(3);

plot(x, y_new, 'r')

sum((y_new - y).^2)