function P2_alg(A,R_vector,H_vector,fign)
B = 1;
Q = 1;
C = 1;

if fign == 1
    figtitle = 'x(t+1)=1,2x(t)+u(t)';
else
    figtitle = 'x(t+1)=0,8x(t)+u(t)';
end

for i=1:length(R_vector)
    R = R_vector(i);
    [KLQ_vector(i),S_vector(i),eignLQ_vector(i)] = dlqr(A,B,Q,R);
    
    for j=1:length(H_vector)
        clear H W PI M e1
        H = H_vector(j);
        W = zeros(H,H);
        for jj=1:H
            for jjj=1:jj
                W(jj,jjj) = C*A^(jj-jjj)*B;
            end
            PI(jj,1) = C*A^jj;
        end
        M = W'*W + R*eye(H);
        e1 = zeros(1,H);
        e1(1) = 1;
        KRH_vector(j) = e1*M^(-1)*W'*PI;
        eignRH_vector(j) = eig(A-B*KRH_vector(j));    
    end
    
    figure(fign)
    subplot(3,2,i)
    plot(H_vector,KRH_vector,'b.','MarkerSize',10)
    hold on
    plot(H_vector,zeros(1,length(H_vector))+KLQ_vector(i),'r.','MarkerSize',5)
    title(sprintf('R = %.1f',R))
    legend('K_{RH}','K_{LQ}','Location','Southeast')
    xlabel('H')
    ylabel('Gain')
    sgtitle(figtitle)
    
    figure(fign+1)
    subplot(3,2,i)
    plot(H_vector,eignRH_vector,'b.','MarkerSize',10)
    hold on
    plot(H_vector,zeros(1,length(H_vector))+eignLQ_vector(i),'r.','MarkerSize',5)
    hold on
    plot(H_vector,zeros(1,length(H_vector))+1,'g','Linewidth',1.5)
    title(sprintf('R = %.1f',R))
    legend('|\lambda_{RH}|','|\lambda_{LQ}|','boundary')
    xlabel('H')
    ylabel('|\lambda|')
    sgtitle(figtitle)
    
end
end