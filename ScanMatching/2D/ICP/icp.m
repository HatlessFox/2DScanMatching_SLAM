% The original ICP algorithm’s association
% method (often called nearest neighbour or the Closest
% Point Rule) considers the euclidean distance between
% points to associate every point in Snew with its nearest
% point in Sref . Once the association set is found, the
% minimum rigid transformation is calculated using the
% Unit Quaternions method.

% [1] P. Besl and N. McKay, “A method for registration of 3-D shapes,” IEEE
% Transactions on pattern analysis and machine intelligence, pp. 239–256,
% 1992.
function [R, t, NI] = icp(Ai, Bi, motion)
% In:
%           Ai: localCart[2xN], points cartesian
%           Bi: localCart[2xN], points cartesian
%           MOTION: CON.U[d psi theta], motion estimation, translational,
%           rotational and orientation
% Out:
%           R: rotational corrrection in quaternion
%           t: translational corrrection in quaternion
%           NI: number of iterations

global Opt DEBUG
% save the options
opts = Opt;

%we only used cartesian points
Al.cart = Ai.localCart;
Bl.cart = Bi.localCart;
Al.polar = Ai.localPolar;
Bl.polar = Bi.localPolar;

itMax = opts.scanmatcher.iterations;
it=1;       %iteration control

% S New initialization
B.cart(:,1:2) = Bl.cart(:,1:2);
B.polar(:,1:2) = Bl.polar(:,1:2);

%Motion estimation q0
u = motion.con.u;
pose(3) = 0;
lrangle = u(2);

% x0 =  cos(lrangle)*u(1);
% y0 =  sin(lrangle)*u(1);
% yaw0 = u(3);

x0 = u(1);
y0 = u(2);
yaw0 = u(3);

x=x0;
y=y0;
yaw=yaw0;


% some initialization and plotting
lasterror = [];
corr=[];
rejval = opts.scanmatcher.Br(2);

if DEBUG.cpAssociation || DEBUG.all
    scrsz = get(0,'ScreenSize');
    opts.fighandle=figure('Position',scrsz,'doublebuffer','on');
    axis equal;
    xlabel('X (m)'); ylabel('Y (m)');
    hold all
    opts.plot_r = plot(NaN,NaN,'.r','MarkerSize',6);
    opts.plot_n = plot(NaN,NaN,'.b','MarkerSize',6);
    opts.plot_a = plot(NaN,NaN,'-m','LineWidth',1);
end

while it < itMax
    
    %% Apply q
    %return if convergence is achieved
    if checkConv(lasterror, corr)
        break;
    end
    
    % apply the motion to S Ref
    [Bnf Bf Af] = transPolarCartScan(B, Al, yaw, [x y], 1, rejval);
    B.cart(:,3) =  pose(3); % Delete 1's from normalized 2D point
    Bf.cart(:,3) =  pose(3);
    
    if opts.scanmatcher.projfilter
        BA = Bf;
        AlA = Af;
    else
        BA = Bnf;
        AlA = Al;
    end
    
    %% Association
    % Compute the association set
    [merr assp ] = cpAssociation(BA,AlA, opts);
    
    % Use rejection rule?
    if ~isempty(opts.scanmatcher.rejection_rule)
        [assp ] = opts.scanmatcher.rejection_rule(assp);
    end
    
%     assp.new = assp.new';
%     assp.ref = assp.ref';
    
    if DEBUG.all
        displayAssociations(assp,opts.plot_a);    
    end
    
    %% Registration
    % Compute the registration only if we have a minimum number of
    % associations
    if size(assp.ref,1) < 5
        break;
    end
    [R t] = regist_besl(assp.new', assp.ref');
    
    
    % update the motion estimation
    yawR = R2e(R);
    yaw = normAngle(yaw + yawR(3));
    x = x + t(1);
    y = y + t(2);
    
    it=it+1;
    
    lastyaw = yawR(3);
    
    % update the error estimation
    lasterror = [lasterror 1/merr];
    corr = [corr; t(1) t(2) lastyaw];
    
    if size(lasterror,2) > opts.scanmatcher.niterconv
        lasterror = lasterror(2:end);
        corr = corr(2:end,:);
    end
    
end

%% Result
% Returns the minimizing Q = [t R]
R = ([normAngle(yaw-yaw0)] );
t = [x y] - [x0 y0];
NI = it;
