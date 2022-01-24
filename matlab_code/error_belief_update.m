clear all; clc; close all
y = 0:0.1:1;
t_final = 20;
t= 1:t_final;
ny = length(y);
py = ones(1,ny)/ny;
% py1 = exp(-1*y);
% py = py1/sum(py1);


% py_temp = py;
[tm, ftm] = creat_trans_matrix(y);

ah_hist = -ones(length(t),1);
k_hist = 3;
figure(1)
% pause()
for i=t
    [ah, pe] = human_action(i-1, y);  
%     ah = saved_ah(i);
    py_temp = py;
    for j=1:ny
        p_obs = 1;
        pp=0;
        for k = 1:ny
            p1 = pyp(y, tm, ftm, y(j), y(k), ah);
            p2 = pih(ah, ah_hist, y(j), i, k_hist);
            pp = p1*p2*py_temp(k) + pp;
        end
        unnorm_p(j) = pp*p_obs;
    end
    py = unnorm_p/sum(unnorm_p);
%     [~, ii] = max(py);
%     epy = y(ii)
    epy = sum(py.*y);
    [~, ii] = min(abs(y-epy));
    epy = y(ii);
    figure(1)
    plot(i, epy, '*b', 'MarkerSize', 10)
    hold on
    plot(i, pe, 'or', 'MarkerSize', 15)
    ylim([0,1])
    yticks(0:0.2:1)

%     plot(y, py, 'linewidth', 2)
%     xlabel('\alpha')
%     ylabel('p(\alpha)')
%     hold on
    pause(0.1)
    ah_hist(i) = ah; 
    [~,pval(i)] = max(py);
    
end
    xlabel('n')
    ylabel('p_e')


function [ah, pe] = human_action(i, y)
    pe = 0.8*1/(1+exp(-0.5*(i-3)));
    de = abs(y-pe);
    [~, ii] = min(de);
    pe = y(ii);
    
%     pe = 0.8;
    if rand > pe
        ah = 1;
    else
        ah = 0;
    end
end

function p = pih(ah, x_hist, yj, step, k_hist)
    hs = step-(1:k_hist);
%     hs = hs(end-k_hist+1:end);
    hi = hs>0;
    if any(hi)
        histk = x_hist(hs(hi));
        nhist =length(histk);
        n_success = sum(histk==1);
        n_fail= nhist - n_success;
%         p_fail = 1./(1+exp(-0.8*(n_fail-2)));
        p_fail = n_fail/nhist;

        if ah == 1
%             p =max(0.001, (1-p_fail)*(1-yj));
            p =max(0.001, (1-p_fail));
        else
%             p = max(0.001, p_fail*yj);
            p = max(0.001, p_fail);
        end

    else
        p=1;
    end
end
        
function p = pyp(y, tm, ftm, y1, y2, ah)
    i1 = y==y1;
    i2 = y==y2;
    if ah ==1
%         if y1==y2-0.1
%             p = 0.3;
%         elseif y1==y2
%             p = 0.2;
%         elseif y1==y2-0.2
%             p=0.2;
%         elseif y1==y2-0.3
%             p=0.2;
%         else
%             p = 0.01;
%         end
        p = ftm(i2, i1);

%         fprintf('p11=%f  p=%f\n', p11, p)
    else
        p = tm(i2, i1);
    end
    
end

function [tm, ftm] = creat_trans_matrix(y)
    gaussian = @(x, mu, sigma) 1/(sigma*sqrt(2*pi))*exp(-0.5*((x-mu)./sigma).^2);
    skewedgaussian = @(x, mu, sigma, alpha) 2*gaussian(x, mu, sigma).*normcdf(alpha*(x-mu)/sigma);
    ny = length(y);
    y=[y,2];
%     ny = length(y);
    tm = zeros(ny);
%     figure()
%     hold on
    x = -0.5:0.01:2;
    figure (3)
    hold on
    for j=1:ny
        pdfy = skewedgaussian(x, min(1.1, y(j+1)), 0.2, 0.5);
        figure(3)
        plot(x, pdfy)
        for i=1:ny
            xi = y(i):0.001:y(i+1);
            yi = skewedgaussian(xi, min(1.1, y(j+1)), 0.1, 0.1);
            tm(j, i) = trapz(xi, yi);
        end
    end
    aa = [y(1), y(ny)];
    figure(2)
    imagesc(aa, aa, tm)
    xticks(y(1:ny))
    colormap(flipud(gray))
    colorbar()
    ftm = flip(tm, 1);
    ftm = flip(ftm, 2);
    
    figure(4)
    imagesc(aa, aa, ftm)
    xticks(y(1:ny))
    colormap(flipud(gray))
    colorbar()
end

