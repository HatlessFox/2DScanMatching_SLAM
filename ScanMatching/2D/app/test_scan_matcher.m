close all
clear


global Opt;
global DEBUG;

# single algorithm debug option
# fine-grain opts:
#   - p2lAssociation, cpAssociation, mpAssociation, mbAssociation,
#     mahaAssociation, normAssociation
#   - gmapping, montecarlo, ga, houghSM, fmtsm, NDT, lf_sog, ahsm
#   - all
DEBUG.ahsm = 0;
DEBUG.all = 0;

Opt.random.seed = sum(100*clock);
fprintf('Random seed: %6.0f.\n',Opt.random.seed);

# Possible values: icp, IDC, 
#   -- pIC,MbICP,ga,gmapping,montecarlo,ahsm,fmtsm,houghSM,NDT,lf_sog, icpbSM.
#   custom_ahsm
Opt.scanmatcher.handle = @custom_ahsm;
# If using mixed algorithm 'icpbSM'
# you have to fill the next two with function handles.
#   Possible values: mahaAssociation (mahalanobis distance),
#                    mbAssociation (metric based ),
#                    cpAssociation (closest Point),
#                    p2lAssociation (point to line),
#                    normAssociation (normal lines association),
#                    mpAssociation (range based association).
Opt.scanmatcher.associate = @mahaAssociation;
#   Possible Values: regist_besl (Besley Unit Vector),
#                    register_martinez (Closed form Least Square),
#                    register_matlab (MATLAB minimization),
#                    registerSVD (Singular Value Decomposition),
#                    registerCensi (Censi's Lagrange Multiplier).
Opt.scanmatcher.register = @ga;
Opt.scanmatcher.rejection_rule = []; # rejection rule handling
Opt.scanmatcher.projfilter = 0; # use projection filter?
Opt.scanmatcher.iterations = 50; # max nm of iterations
Opt.scanmatcher.Br = [0.1 0.5]; # angular and radian tresholds
Opt.scanmatcher.map_res = 0.5; # map resolution
Opt.scanmatcher.convalue = 0.00001; # below the value the result is ok
Opt.scanmatcher.niterconv = 3; # min number of iters before convergence check
Opt.scanmatcher.chival = chi2inv(0.95, 0.1); # tolerance value (?)

Opt.scan.maxscanpoints = 1; # min num of pts before scan composition
Opt.error.display_result = 1;
Opt.map.resolution = Opt.scanmatcher.map_res;
Opt.plot.robot_scale = 1;

ww = wall_world();
world = init_world(ww);
sensor = init_sensor();
# TODO: createMap

NI= [];

ref_pose = [0 0 0];
cur_pose = [1 1 30];

ref_scan = get_scan_view(ref_pose, sensor, world);
cur_scan = get_scan_view(cur_pose, sensor, world);

# filterScan(..) ?

[R t NI] = Opt.scanmatcher.handle(ref_scan, cur_scan, ref_pose);
est_pose = [t'; rad2deg(R)];

fprintf('== DONE ==\n')
fprintf('[GT Pose] x: %.2f; y: %.2f; th: %.2f.\n',
        cur_pose(1), cur_pose(2), cur_pose(3));
fprintf('[SM Translation] x: %.2f; y: %.2f; th: %.2f.\n',
        est_pose(1), est_pose(2), est_pose(3));

#------------------------------------------------------------------------------#
# Plotting

# blue - ref
# green - groud truth (curr)
# red - corrected

# draw map
f = figure;
axis equal;

plot(ww.walls(1,:), ww.walls(2,:), 'LineWidth', 1, 'color', 'k');
hold on;
display_scan(ref_scan, ref_pose, 'b');
display_scan(cur_scan, cur_pose, 'g');
display_scan(cur_scan, est_pose, 'r');

ginput(1);
#input("press to continue");

return
        

    %% PLOTTING
    
    if (size(Rob.Map.grid,1) > 0) && Opt.error.display_result == 1
        
        if Opt.plot.points_cr
            set(Opt.plot.points_cr_h,'XData',Rob.Map.grid(:,1),'YData',Rob.Map.grid(:,2));
        end
        
        if Opt.plot.ground_truth
            Opt.plot.ground_truth_data = [Opt.plot.ground_truth_data; Rob.state.gt(:,end)' ];
            set(Opt.plot.ground_truth_h,'XData',Opt.plot.ground_truth_data(:,1),'YData',Opt.plot.ground_truth_data(:,2));
            grob = graphicsRobot(Opt.plot.ground_truth_data(end,:)');
            set(Opt.plot.ground_truth_h_rob,'XData',grob(1,:),'YData',grob(2,:));
        end
        
        if Opt.plot.dead_reckoning
            Opt.plot.dead_reckoning_data = [Opt.plot.dead_reckoning_data ; frameRef(Rob.state.dr(:,end),Rob.state0([1 2 6]),0)'];
            set(Opt.plot.dead_reckoning_h,'XData',Opt.plot.dead_reckoning_data(:,1),'YData',Opt.plot.dead_reckoning_data(:,2));
            grob = graphicsRobot(Opt.plot.dead_reckoning_data(end,:)');
            set(Opt.plot.dead_reckoning_h_rob,'XData',grob(1,:),'YData',grob(2,:));
        end
        
        if Opt.plot.corrected
            Opt.plot.corrected_data = [Opt.plot.corrected_data; frameRef(Rob.state.x(:,end),Rob.state0([1 2 6]),0)'];
            set(Opt.plot.corrected_h,'XData',Opt.plot.corrected_data(:,1),'YData',Opt.plot.corrected_data(:,2));
            grob = graphicsRobot(Opt.plot.corrected_data(end,:)');
            set(Opt.plot.corrected_h_rob,'XData',grob(1,:),'YData',grob(2,:));
            
            if Opt.plot.correction_uncertainty
                draw_ellipse(Opt.plot.corrected_data(end,:),Rob.state.P,'r',Opt.plot.correction_uncertainty_h);
            end
        end
        
        drawnow;
    end
    out = 1;


if ~isempty(NI)
    error_gt =  errorReport(Rob,SimSen,Opt, NI);
    save('std_last','error_gt');
end
