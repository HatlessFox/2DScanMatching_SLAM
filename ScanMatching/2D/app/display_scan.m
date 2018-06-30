function [] = display_scan(scan, pose, c)
  th = deg2rad(pose(3));
  tr = [pose(1) pose(2)]';
  rot = [cos(th) -sin(th); sin(th) cos(th)];
  for i = 1:size(scan.localCart, 1)
    transformed_scan(i, :) = rot * scan.localCart(i, :)' + tr;
  end
  displayPoints(transformed_scan, c);
  hold on;
  g_rob = graphicsRobot([tr; th]);
  fill(g_rob(1, :), g_rob(2, :), c);
  hold on;
end
