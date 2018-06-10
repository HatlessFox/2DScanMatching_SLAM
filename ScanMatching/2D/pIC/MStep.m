unction [out,solution,errorK1,nConverged] = pICRegistration(assoc,motion,errorK1,nConverged,tmp)
global PARAM

if (size(assoc,2) == 0)
    error('No associations avaiable.');
    out =  -1;
    return
end

E = []; H = [];
C = [];
for i = 1:size(assoc,2)
    e = assoc(i).a-assoc(i).refNewPoint;
    E = [E; (-e+assoc(i).Jq*motion.estimation')];
    H = [H; assoc(i).Jq];
    C = blkdiag(C,assoc(i).Pa+assoc(i).refNewP);
end

invC = inv(C);
qmin = inv(H'*invC*H)*H'*invC*E;

tx = qmin(1,1); ty = qmin(2,1); yaw = qmin(3,1);
cosy = cos(yaw); siny = sin(yaw);

err = 0;
for i = 1:size(assoc,2)
    errorX = assoc(i).newPoint(1,1)*cosy-assoc(i).newPoint(2,1)*siny+tx-assoc(i).a(1,1);
    errorY = assoc(i).newPoint(1,1)*siny-assoc(i).newPoint(2,1)*cosy+ty-assoc(i).a(2,1);
    err = err+errorX^2+errorY^2;
end

errorRatio = err/errorK1;
dist = motion.estimation - qmin'
ERR = 1-errorRatio
if (abs(1-errorRatio) <= PARAM.errorRatio) || abs(dist(1)) < PARAM.error(1) && abs(dist(2)) < PARAM.error(2) && abs(dist(3) < PARAM.error(3))
    nConverged = nConverged+1;
else
    nConverged = 0;
end

% Build solution
errorK1 = err;
solution = qmin;

% Smooth convergence criterion
if (nConverged > PARAM.nIterationSmoothConvergence)
    out = 1;
else
    out = 0;
end
