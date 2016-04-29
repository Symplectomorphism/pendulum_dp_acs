% Written by Aykut
classdef PendulumControl < DrakeSystem
  
  properties
    p  % plant
  end
  
  methods
    function obj = PendulumControl(plant)
      obj = obj@DrakeSystem(0,0,2,1,true,true);
      
      obj.p = plant;
      obj = obj.setInputFrame(plant.getStateFrame);
      %         obj = obj.setInputFrame(plant.getOutputFrame);
      obj = obj.setOutputFrame(plant.getInputFrame);
      
    end
    
    function u = output(obj,t,~,x)
    alpha = [50.953        6.124      -8.7161       10.636   4.4325e-14   1.5004e-14  -4.7721e-15  -2.0002e-15  -1.5116e-15   4.8861e-16     -0.82594   -0.0022151     0.023117      0.67719]';
        a0 = alpha(1); a1 = alpha(2); a2 = alpha(3); a3 = alpha(4);
        b1 = alpha(5); b2 = alpha(6); b3 = alpha(7);
        c1 = alpha(8); c2 = alpha(9); c3 = alpha(10);
        d1 = alpha(11); d2 = alpha(12); d3 = alpha(13);
        e0 = alpha(14);
        x1 = x(1); x2 = x(2);
        u = (-1).*e0.*x2+(-1).*c1.*cos(x1)+(-1).*c2.*cos(2.*x1)+(-1).*c3.*cos(3.*x1)+(-1).*d1.*sin(x1)+(-1).*d2.*sin(2.*x1)+(-1).*d3.*sin(3.*x1);
%         u = -10*x1-5*x2;
        fprintf(['time = ', num2str(t), '\n']);
    end
    
  end
  
  methods (Static)
    function run(animFlag)
      p = PendulumPlant();
      c = PendulumControl(p);
%       v = CartPoleVisualizer(cp);
      sys = feedback(p,c);
      
      x0 = [pi-0.1; 0];
      traj = simulate(sys,[0 10], x0);
      t = linspace(0,10,1001);
      x = eval(traj,t);
      figure(1), clf
      plot(t, x);
      if nargin && animFlag
          p.animate(traj)
      end
    end
  end
end
