function [scan] = get_scan_view(pose, sensor, world)
  robot.state.gt = [pose(1); pose(2); deg2rad(pose(3))];
  [raw _] = simObservation(robot, sensor, world);
  scan.localCart = raw.data.localCart;
  scan.localPolar = raw.data.localPolar;
end
