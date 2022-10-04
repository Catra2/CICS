figure
N=30
for i = 1:N
    theta(i) = reference{i}(2);
end
polarhistogram(theta,5,'FaceColor','k')
ax = gca;
ax.ThetaZeroLocation = 'top';
ax.RTick = [];
title("Reference for all trials")
grid off
rlim([0 1])

figure
for i = 1:N
    theta(i) = response{i}(1,1);
end
polarhistogram(theta,1,'FaceColor','k')
ax = gca;
ax.ThetaZeroLocation = 'top';
ax.RTick = [];
title("Arm position x_0 for all trials")
grid off
rlim([0 1])

figure
for i = 1:N
    theta(i) = response{i}(3,1);
end
polarhistogram(theta,1,'FaceColor','k')
ax = gca;
ax.ThetaZeroLocation = 'top';
ax.RTick = [];
title("Pendulum position x_0 for all trials")
grid off
rlim([0 1])


x = 1:N;
y = 1:N;
figure
for i = 1:N
    y(i) = response{i}(2,1);
end
scatter(x,y,'filled')
title("Arm velocity x_0 for all trials")
xlabel("Trial (n)")
ylabel("Angular Velocity (rad/s)")


figure
for i = 1:N
    y(i) = response{i}(4,1);
end
scatter(x,y,'filled')
title("Pendulum velocity x_0 for all trials")
xlabel("Trial (n)")
ylabel("Angular Velocity (rad/s)")