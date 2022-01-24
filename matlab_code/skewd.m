clear all; clc; close all;

gaussian = @(x, mu, sigma) 1/(sigma*sqrt(2*pi))*exp(-0.5*((x-mu)./sigma).^2);
% skewedgaussian = @(x, mu, sigma, alpha) 2*gaussian(x, mu, sigma).*normcdf(alpha*x, mu, sigma);
skewedgaussian = @(x, mu, sigma, alpha) 2*gaussian(x, mu, sigma).*normcdf(alpha*(x-mu)/sigma);
% x =-20:0.001:100;
% y = skewedgaussian(x, 2, 1, 20);
% figure
% hold on
% plot(x, y)
% trapz(x,y)/2

y=0:0.2:1;
y=[y,2];
ny = length(y);
x = -0.5:0.01:2;
s = zeros(6);
figure()
hold on
for j=1:6
    pdfy = skewedgaussian(x, min(1.2, y(j+1)), 0.2, 0.5);
    plot(x, pdfy)
    for i=1:6
        xi = y(i):0.001:y(i+1);
        yi = skewedgaussian(xi, min(1.2, y(j+1)), 0.2, 0.5);
        s(j, i) = trapz(xi, yi);
    end
end
aa = [y(1), y(6)];
figure
imagesc(aa, aa, s)
colormap(flipud(gray))
colorbar()
s

%%
nfail = 0:6;
sigmo = 1./(1+exp(-0.8*(nfail-2)));
figure
plot (nfail, sigmo, '.', 'MarkerSize',15)

%%
close all
naction = 0:20;
sigm_action = 0.5*1./(1+exp(-0.3*(naction-5)))
figure
plot(naction, sigm_action, '.', 'MarkerSize', 15)

        
    
    
    
    
    
    
% xs = 0;
% x = xs:xs+5;
% npx = sum(x>=0)
% y = binopdf(x,npx,0.5);
% sum(y)
% figure
% bar(x,y,1)
% xlabel('Observation')
% ylabel('Probability')


% for i=1:6
%     for j=1:6
        