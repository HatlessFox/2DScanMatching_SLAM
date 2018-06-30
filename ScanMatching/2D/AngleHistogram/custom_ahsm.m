function [ R t NI] = custom_ahsm( ScanRef, ScanNew, init_pose )
%ahsm  This methods uses a one dimensional signal-type representation of the scan
% points. The vector delimited by every two consecutive
% points of the scan is computed and represented in polar
% coordinates with respect to the global frame of reference.
% Then, a histogram of their angles is computed as a 1D
% signal which is invariant to translation but not to rotation.
% Given two rotated scans, the rotation angle can be
% computed as the signal shift which maximizes the cross
% correlation of the corresponding scan-signals. Once the
% rotation has been solved, the same procedure may be
% applied for the x and y to solve for the translation.


% Keeping Pack of Position and Orientation
% of Moving Indoor Systems
% by Correlation of Range-Finder Scans
% Gerhard We$, Christopher Wetzler, Ewald von Puttkamer

global Opt;
%Select which points to use, in this case local cartesian point in
Al.cart = ScanRef.localCart;
Al.polar = ScanRef.localPolar;

Bl.cart = ScanNew.localCart;
Bl.polar = ScanNew.localPolar;

%Calculate the motion parameters q0
x0 = init_pose(1);
y0 = init_pose(2);
yaw0 = init_pose(3);

x=x0;
y=y0;
yaw=yaw0;
NI = 1;

        

% Apply the initial estimaion q
[B Bf Af] = transPolarCartScan(Bl, Al, yaw, [x y], 1, Opt.scanmatcher.Br(2));

% Apply projection filter?
if Opt.scanmatcher.projfilter
    BA = Bf;
    AlA = Af;
else
    BA = B;
    AlA = Al;
end

global DEBUG
if DEBUG.ahsm || DEBUG.all
    
    figure;
    hold on
    axis equal
    grid on
    displayPoints(BA.cart,'g',0);
    displayPoints(AlA.cart,'r',0);
    
end

    t = [0 0 0];
    R = e2q([0 0 0]);

% Generate the Angle histogram of both scans and normalize
Bh = makeAnglehistogram(BA.cart,yaw0);
Ah = makeAnglehistogram(AlA.cart,yaw0);
Bh = Bh./max(Bh);
Ah = Ah./max(Ah);

if DEBUG.ahsm || DEBUG.all
    figure
    plot(1:size(Bh,2),Bh);
    figure
    plot(1:size(Ah,2),Ah);
end

% Circular cross correlation and shift to have the 0 frequency in the
% middle
corrBA=zeros(1,size(Ah,2));
for j=1:size(Ah,2)
    for k=1:size(Bh,2)
        corrBA(j) = corrBA(j) + Bh(k)*Ah( mod(j+k,size(Ah,2))+1 );
    end
end
sizec = size(corrBA,2);
corrBA = arrayshift(corrBA,round(sizec/2));

% Normalize the result and cut off the borders which usually contains
% artifacts
[mm ii] = max(corrBA);
corrBA=(corrBA.*hann(length(corrBA))' )./mm;

% corrBA(1:30) = 0;
% corrBA(sizec-30:end) = 0;

% Applying a fast smoothing
corrBASmooth = fastsmooth(corrBA,10);
if DEBUG.ahsm || DEBUG.all
    figure
    plot(1:size(corrBA,2),corrBA);
end

% Locate local maxima and extract rotational hypotheses
maxs = findLocalMaxima(corrBASmooth,3);
angle_hyp = deg2rad((maxs - round(size(corrBA,2)/2) ))

% Locate local maxima in the reference scan to extract the direction of
% correction
maxb = findLocalMaxima(Bh,1);
maxb = deg2rad((maxb - round(size(Bh,2)/2) ));


BAt = BA;
AAt = AlA;

for q = size(maxb,1)
    
    % Transform both scan according to the direction of correction
    [BAt] = transPolarCartScan(BAt, AlA, normAngle(maxb(q)+angle_hyp(1)), [0 0] , 1,Opt.scanmatcher.Br(2));
    [AAt] = transPolarCartScan(AAt, AlA, maxb(q)  , [0 0] , 1,Opt.scanmatcher.Br(2));

    if DEBUG.ahsm || DEBUG.all
        
        figure;
        hold on
        axis equal
        grid on
        displayPoints(BAt.cart,'g',0);
        displayPoints(AAt.cart,'r',0);
        
    end
    
bounds = [ max(max(BAt.cart(:,1)),min(AAt.cart(:,1))), min(min(BAt.cart(:,1)),min(AAt.cart(:,1)));
           max(max(BAt.cart(:,2)),min(AAt.cart(:,2))), min(min(BAt.cart(:,2)),min(AAt.cart(:,2)))];

    
    % Create the Axis histogram for the X and Y components of the points
    bxhist = makeAxishistogram(BAt.cart,'x',bounds);
    byhist = makeAxishistogram(BAt.cart,'y',bounds);
    
    axhist = makeAxishistogram(AAt.cart,'x',bounds);
    ayhist = makeAxishistogram(AAt.cart,'y',bounds);

    
    % smooth the histograms and do the cross correlation
%     bxhist =fastsmooth(bxhist,2);
%     axhist =fastsmooth(axhist,2);
%     byhist =fastsmooth(byhist,2);
%     ayhist =fastsmooth(ayhist,2);
 
    corrX=zeros(1,size(bxhist,2));
    for j=1:size(bxhist,2)
        for k=1:size(axhist,2)
            corrX(j) = corrX(j) + axhist(k)*bxhist( mod(j+k,size(bxhist,2))+1 );
        end
    end
    
    corrY=zeros(1,size(byhist,2));
    
    
    for j=1:size(byhist,2)
        for k=1:size(ayhist,2)
            corrY(j) = corrY(j) + ayhist(k)*byhist( mod(j+k,size(byhist,2))+1 );
        end
    end
    
    corrY = circularCrossCorrelation(byhist,ayhist,1);
    corrX = circularCrossCorrelation(bxhist,axhist,1);
    
    if DEBUG.ahsm || DEBUG.all
        figure
        subplot(2,1,1);plot(1:size(bxhist,2),bxhist);
        subplot(2,1,2);plot(1:size(axhist,2),axhist);
        figure
        subplot(2,1,1);plot(1:size(byhist,2),byhist);
        subplot(2,1,2);plot(1:size(ayhist,2),ayhist);
    end
        
    % shift the results to have the 0 frequency in the middle
    midvx = round( size(corrX,2) / 2 );
    corrX = arrayshift(corrX,midvx);
    
    midvy = round( size(corrY,2) / 2 );
    corrY = arrayshift(corrY,midvy);
    
    
    if DEBUG.ahsm || DEBUG.all
        figure
        plot(1:size(corrX,2),corrX);
        figure
        plot(1:size(corrY,2),corrY);
    end

    
    % Find the translation from the local maxima of the cross correlation
    % bouding the result according to the user input Opt.scanmatcher.Br(2)
    Y = (findLocalMaxima(corrX,2) - midvx )*(Opt.scanmatcher.Br(2)/(midvx*2));
    X = (findLocalMaxima(corrY,2) - midvy )*(Opt.scanmatcher.Br(2)/(midvy*2));

    if isempty(X) || isempty(Y)
       return 
    end

    yaw = -maxb(q);
    rt = [cos(yaw) -sin(yaw); sin(yaw) cos(yaw)];
    
    % Finally Q = [t R]
    t = (rt*[X(1); Y(1)])' ;
    R = -angle_hyp(1);
end

end
