%% Random world generator
%% SYNTAX  POLYGON = wall_generator(0.3);figure; axis equal; plot(POLYGON(1,:),POLYGON(2,:))

classdef wall_generator
  properties
    POLYGON=[];
    TRAJ = [];
  end
  methods

    function obj = wall_generator(varargin) %constructor
      if nargin == 0
        obj.POLYGON=wall_generator.random_world(0.1);
        return
      end

      obj.POLYGON = wall_generator.random_world_unstruct(varargin{1});
      #obj.POLYGON = wall_generator.random_world(0.6);
      if 1 < nargin && varargin{2}==1
        obj.TRAJ = read_trajectory(obj);
      end
    end

    function [traj] = read_trajectory(obj)
      button = 1;
      traj = [];
      while (button == 1)
        f = update_prompt_plot(obj, traj);
        [x, y, button] = ginput(1);
        close(f);
        traj = vertcat(traj, [x, y, 0, 0, 0, 0]);
        pts_n = size(traj, 1);
        if (pts_n < 2)
          continue;
        end
        # setup theta
        diff = traj(pts_n, :) - traj(pts_n-1,:);
        if (sum(diff(1:2)) ~= 0)
          traj(pts_n, 6) = atan2(diff(1,2), diff(1,1));
        else
          traj(pts_n, 6) = traj(pts_n-1, 6);
        end
      endwhile
      traj = traj';
    end

    function [f] = update_prompt_plot(obj, traj)
      f = figure;
      if (1 < size(traj, 1))
        plot(obj.POLYGON(1,:),obj.POLYGON(2,:),'LineWidth',3,
             traj(:, 1),traj(:, 2));
      else
        plot(obj.POLYGON(1,:),obj.POLYGON(2,:),'LineWidth',3)
      end
      axis equal;
    end

    function disp(obj)
      figure;
      plot(obj.POLYGON(1,:),obj.POLYGON(2,:),'LineWidth',3);
      axis equal;
    end
  end # non-static methods
    
  methods (Static)
    %Generator
    function polygon = random_world_unstruct(ell_sigma)
      last_p = [(rand*3)-1;(rand*3)-1];
      last_angle = 0;
      angle = pi*2;
      polygon = last_p;
      init_p=last_p;
      for l = 1:1 %round(rand*2)+2
        npoly = 40;
        for n = 1:npoly
          angle = rand*(pi*2);
          while angle>last_angle+pi/1.5 || angle < last_angle-pi/1.5
            angle = rand*(pi*2);
          end
          last_angle = angle;
          d = rand*2;

          polygon_x = last_p(1) + d*cos(angle);
          polygon_y = last_p(2) + d*sin(angle);
          polygon=[polygon [polygon_x;polygon_y] ];

          last_p = [polygon_x; polygon_y];
        end
        polygon=[polygon [NaN;NaN] ];

        last_p = init_p + [(rand*8)-4;(rand*8)-4];
        init_p = last_p;

        last_angle = 0;
        angle = pi*2;
        polygon = [polygon last_p];
      end
    end

    #######################
    # Structured generation

    function [poly] = convpoly2cw(poly)
      # TODO: generic version
      xs = poly(:, 1)';
      ys = poly(:, 2)';
      angles = atan2(ys - mean(ys), xs - mean(ys));
      [~, order] = sort(angles);
      poly = [xs(order); ys(order)]';
    end

    function [pol] = draw_ellipse(x0,y0,a,b,theta)
      # TODO: fix angle rotation
      % angular positions of vertices
      t = linspace(0, 2*pi, 30);
      for i = 1:length(x0)
        % pre-compute rotation angles (given in degrees)
        cot = cosd(theta);
        sit = sind(theta);
        % compute position of points used to draw current ellipse
        xt = x0(i) + a(i) * cos(t) * cot - b(i) * sin(t) * sit;
        yt = y0(i) + a(i) * cos(t) * sit + b(i) * sin(t) * cot;
      end
      pol = wall_generator.convpoly2cw([xt;yt]');
      pol = vertcat(pol, pol(1, :));
    end

    function [pol] = gen_rect(x0,y0,a,b,theta) % angular positions of vertices
      rect = [x0-a/2, y0+b/2;  # top-left
              x0+a/2, y0+b/2;  # top-right
              x0+a/2, y0-b/2;  # bot-right
              x0-a/2, y0-b/2]; # bot-left
      # TODO: fix rotation (wrt x0 y0)
      T = [cos(theta) -sin(theta);
           sin(theta)  cos(theta)];
      for i = 1:size(rect,1)
        rotated_rect(i, :) = T*rect(i,:)';
      end
      pol = wall_generator.convpoly2cw(rotated_rect);
      pol = vertcat(pol, pol(1, :));
    end

    %Generator
    function polygon = random_world(ell_sigma)
      npoly = round(rand*10)+4;
      polygon = [];
      for n = 1:npoly
        type = rand;
        op = merge(0.8 < rand, 'ab', 'or');

        this_pol = [];
        x0_r = rand*5+1;
        y0_r = rand*5+1;
        a_r = rand*3+1;
        b_r = rand*3+1;
        theta_r = 0; #rand*(pi/2);

        if type > ell_sigma %rect
          this_pol = wall_generator.gen_rect(x0_r,y0_r,a_r,b_r,theta_r);
        else %ellipse
          this_pol = wall_generator.draw_ellipse(x0_r,y0_r,a_r/2,b_r/2,theta_r);
        end

        if ~isempty(polygon)
          [poly_x, poly_y] = oc_polybool(polygon, this_pol, 'or');
          polygon = [poly_x poly_y];
        else
          polygon = this_pol;
        end
      end
      polygon = polygon';
    end

  end # static methods
end # classdef
