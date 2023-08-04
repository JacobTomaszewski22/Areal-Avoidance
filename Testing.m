trajectory = generate_trajectory();
figure
boxSize = 200; % mm
axis([-boxSize boxSize -boxSize boxSize]);
hold on
grid on
for i = 1:length(trajectory)
    plot(trajectory(i,1),trajectory(i,2),'.b')
    pause(0.001);
end