clear all; clc; close all
y = 0:0.2:1;
t_final = 20;
t= 1:t_final;
ny = length(y);
py = ones(1,ny);
% py (ny+1) = 0.1;
py(1:ny) = (1)/ny;


% py_temp = py;

y_h = 0.8;
ah_hist = -ones(length(t),1);
k_hist = 5;
saved_ah = [0;1;1;1;0;0;1;0;0;1;1;0;1;0;0;1;1;1;0;1];
figure()
plot(t, y_h*ones(1,t_final), '--r', 'LineWidth',2)
xlabel('n')
ylabel('\alpha')
ylim([0,1])
hold on
pause()
for i=t
    ah = human_action(y_h);  
%     ah = saved_ah(i);
    py_temp = py;
    for j=1:ny
        p_obs = 1;
        pp=0;
        for k = 1:ny
            p1 = pyp(y(j), y(k));
            p2 = pih(ah, ah_hist, y(k), i, k_hist);
            pp = p1*p2*py_temp(k) + pp;
        end
        unnorm_p(j) = pp*p_obs;
    end
    py = unnorm_p/sum(unnorm_p);
    yest = sum(py.*y);
%     plot(y, py, 'linewidth', 2)
%     xlabel('\alpha')
%     ylabel('p(\alpha)')
plot(i, yest, '*b', 'MarkerSize', 10)
    pause(0.1)
%     pause()
    ah_hist(i) = ah; 
    [~,pval(i)] = max(py);
    
end
% plot(t,y(pval))



function ah = human_action(y_h)
    if rand <y_h
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
        n_picked = sum(histk==1);
        n_not_picked = nhist - n_picked;
%         p = max(0.02, yj*(n_picked/nhist));
        if ah == 1
            p = max(0.001, yj*(n_picked/nhist));
%             p = p;
        else
            p = max(0.001, (1-yj)*(n_not_picked/nhist));
%             p=1-p;
        end
    else
        p=1;
    end
end
        
function p = pyp(y1,y2)
    if y1==y2
        p = 1;
    else
        p = 0.00;
    end
end

