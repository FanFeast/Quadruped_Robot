robo = importrobot('quadf.urdf');
robo.DataFormat = 'column';
taskSpaceLimits = [0.25 0.5; -0.125 0.125; -0.15 0.1];
numJoints = 3; % Number of joints in robot

numSamples = 7;
[pathSegments, surface] = generateMembranePaths(numSamples, taskSpaceLimits);

% Visualize the output
figure
surf(surface{:},'FaceAlpha',0.3,'EdgeColor','none');
hold all
for i=1:numel(pathSegments)
    segment = pathSegments{i};
    plot3(segment(:,1),segment(:,2),segment(:,3),'x-','LineWidth', 2);
end
hold off