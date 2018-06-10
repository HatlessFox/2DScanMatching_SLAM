function [ O ] = createPlot(  )
%CREATEPLOT Generates the plot to show the SLAM 

global Opt


scrsz = get(0,'ScreenSize');
Opt.plot.figure=figure('Position',scrsz, 'doublebuffer','on');
%axis([-20 20  -20 20]);
axis equal;
xlabel('X (m)'); ylabel('Y (m)');
hold all

if Opt.plot.ground_truth
    Opt.plot.ground_truth_h = plot(NaN,NaN,'r');
    Opt.plot.ground_truth_h_rob = fill(NaN,NaN,'r');
end
if Opt.plot.dead_reckoning
    Opt.plot.dead_reckoning_h = plot(NaN,NaN,'k');
    Opt.plot.dead_reckoning_h_rob = fill(NaN,NaN,'k');
end
if Opt.plot.corrected
    Opt.plot.corrected_h = plot(NaN,NaN,'b');
    Opt.plot.corrected_h_rob = fill(NaN,NaN,'b');
end

if Opt.plot.correction_uncertainty
   Opt.plot.correction_uncertainty_h = plot(NaN,NaN,'--b');
end

if Opt.plot.points_gt
    Opt.plot.points_gt_h = plot(NaN,NaN,['.r'], 'MarkerSize', 6);
end
if Opt.plot.points_dr
    Opt.plot.points_dt_h = plot(NaN,NaN,['.k'], 'MarkerSize', 6);
end
if Opt.plot.points_cr
    Opt.plot.points_cr_h = plot(NaN,NaN,['.b'], 'MarkerSize', 6);
end



end

