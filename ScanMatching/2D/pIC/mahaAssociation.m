function [out, A] = mahaAssociation(refPoints,newPoints,motion)
% Function mahaAssociation
% Computes correspondences between two scans
% In:
%   refPoints
%   newPoint
%   motion: [x y yaw]
%   prec: refPoint's precomputed stuff
% Out:
%   out:
%   A: associations


confidence = 0.95;
% Mahalanobis distance^2
x = motion.estimation(1);
y = motion.estimation(2);
yaw =  motion.estimation(3);
siny = sin(yaw);
cosy = cos(yaw);

% newPoints referenced to reference coordinate system
rTn = [cosy -siny x; siny cosy y; 0 0 1];

% Look for points in Sref statistically compatible with pi
chi2value = chi2inv(confidence,2);

A = [];
Jp = [-cosy siny; -siny -cosy];
% hold on
% axis equal
% displayPoints(newPoints.cart','b');
% B = multRow( [newPoints.cart' repmat(1, size(newPoints.cart,2) , 1) ],rTn);
% displayPoints(refPoints.cart','g');
% displayPoints(B,'r');

for j = 1:size(newPoints.cart,2)
    refp = rTn*[newPoints.cart(:,j);1]; % New point to Sref
    Jq = [-1 0 newPoints.cart(1,j)*siny+newPoints.cart(2,j)*cosy; 0 -1 -newPoints.cart(1,j)*cosy+newPoints.cart(2,j)*siny];
    Pcompos = Jq*motion.P*Jq'+ Jp*newPoints.P(:,2*j-1:2*j)*Jp';
    refNewP = Pcompos; % Composition q-p covariance 
    
    Eassoc = [];
    Cov =[];
    for i = 1:size(refPoints.cart,2)
         d = refPoints.cart(:,i)-refp(1:2,1);
         C = Pcompos+refPoints.P(:,2*i-1:2*i);
         dist = d'*inv(C)*d
         if (dist < chi2value)
             Eassoc = [Eassoc refPoints.cart(:,i)]; % Sref points statically compatible
             Cov = [Cov C];
             %Passoc = [Passoc refPoints.P(:,2*i-1:2*i)];
         end
    end
    
    Pr_temp = zeros(2,2,size(Eassoc,2));
    for i = 1:size(Eassoc,2)
      Pr_temp(:,:,i) = refPoints.P(:,2*i-1:2*i);
    end
    
    % Build association struct
    if (size(Eassoc,2) > 0)
        association.newPoint = newPoints.cart(:,j);
        association.newP = newPoints.P(:,2*j-1:2*j);
        association.refNewPoint = refp(1:2,1);
        association.Cov = Cov;
        association.refNewP = refNewP;
        association.Jq = Jq;
        %association.refPoint = Eassoc;
        %association.refP = Passoc;
        
        prob_a =[];
        for i = 1:size(Eassoc,2)
            %prob_a = [prob_a mvnpdf(Eassoc(:,i),association.newPoint,association.Cov(:,2*i-1:2*i))];
            prob_a = [prob_a mvnpdf(Eassoc(:,i)',association.refNewPoint',association.Cov(:,2*i-1:2*i))];
        end
        eta = 1/sum(prob_a);
        prob_a = eta*prob_a;
        
        association.a = [0;0];
        for i = 1:size(Eassoc,2)
            association.a = association.a+Eassoc(:,i)*prob_a(i);
        end
        
        association.Pa = zeros(2);
        for i = 1:size(Eassoc,2)
            diff_a = Eassoc(:,i)-association.a;
            association.Pa = association.Pa+diff_a*diff_a'*prob_a(i);
        end
        
%         % Display correspendences
% %         figure;
% %         hold on;
% %         plot(refp(1,1),refp(2,1),'r.',Eassoc(1,:),Eassoc(2,:),'b.');
% %         draw_ellipse(refp(1:2,1)',refNewP,'r');
% %         draw_ellipse(Eassoc',Pr_temp,'c');
% %         plot(association.a(1,1),association.a(2,1),'g.');
% %         draw_ellipse(association.a',association.Pa,'g');
% %         hold off;
% %         pause;
%         figure;
%         hold on;
%         plot(refp(1,1),refp(2,1),'r.',Eassoc(1,:),Eassoc(2,:),'b.');
%         plot(association.a(1,1),association.a(2,1),'g.');
%         hold off;
%         pause;
        
        A = [A association];
    end
end

out = 0;

