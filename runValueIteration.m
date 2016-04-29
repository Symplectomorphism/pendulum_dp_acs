function [xbins, ubins, ind, J] = runValueIteration

plant = PendulumPlant;
options.dt = 1e-2;
options.gamma = .999;
options.wrap_flag = [true;false];
%cost = @mintime; ulimit = 1;
cost = @lqrcost; ulimit = 2*10; % = 5 converges faster (but pumps less)
xbins = {linspace(0,2*pi,51),linspace(-10,10,51)};
ubins = linspace(-ulimit,ulimit,15);
mdp = MarkovDecisionProcess.discretizeSystem(plant,cost,xbins,ubins,options);

function drawfun(J,PI)
  fig = sfigure(2); 
  % figure size is hard-coded simply so that it is aesthetically pleasing.
  % parameters for position set through trial-and-error.
  set(fig, 'units', 'normalized', 'position', [.4 .1 .2 .75]);
  clf;
  n1=length(xbins{1});
  n2=length(xbins{2});
  subplot(2,1,1); % imagesc(xbins{1},xbins{2},reshape(ubins(PI),n1,n2)');
  mesh(xbins{1},xbins{2},reshape(ubins(PI),n1,n2)');
  axis xy;
  xlabel('$\theta$', 'Interpreter', 'LaTeX', 'FontSize', 15);
  ylabel('$\dot{\theta}$', 'Interpreter', 'LaTeX', 'FontSize', 15);
  title('$u(x)$', 'Interpreter', 'LaTeX', 'FontSize', 15);
  subplot(2,1,2); % imagesc(xbins{1},xbins{2},reshape(J,n1,n2)');
  mesh(xbins{1},xbins{2},reshape(J,n1,n2)');
  axis xy;
  xlabel('$\theta$', 'Interpreter', 'LaTeX', 'FontSize', 15);
  ylabel('$\dot{\theta}$', 'Interpreter', 'LaTeX', 'FontSize', 15);
  title('$\mathcal{J}(x)$', 'Interpreter', 'LaTeX', 'FontSize', 15);
  drawnow;
end

[J,PI] = valueIteration(mdp,0.001,@drawfun);
ind = PI.PI;

sys = feedback(plant,PI);
v = PendulumVisualizer();
for i=1:5
  xtraj = simulate(sys,[0,10],0.2*randn(2,1));
  v.playback(xtraj);
end

figure(50), clf
[X,Y,Z] = cylinder(1,50);
h = surf(X,Y,20*(Z-1/2));
set(h, 'EdgeColor', 'none', 'FaceColor', [0.7, 0.7, 0.7], 'FaceAlpha', 0.7);
t = linspace(xtraj.tspan(1), xtraj.tspan(2), 1001);
x = eval(xtraj,t);
hl = line(cos(x(1,:))', sin(x(1,:))', x(2,:)', 'Color', 'k');
hl.LineWidth = 2;

figure(2), subplot(2,1,2)
hl2 = line(x(1,:)', x(2,:)');
hl2.Color = 'k';
hl2.LineWidth = 2;

end

function g = lqrcost(sys,x,u)
  xd = [pi;0];
  g = (x-xd)'*(x-xd) + u^2;
end

function g = mintime(sys,x,u)
  xd = [pi;0];
  if (x-xd)'*(x-xd) < .05;
    g = 0;
  else
    g = 1;
  end
end
