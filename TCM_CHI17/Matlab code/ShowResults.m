clear all;
path = 'result.dat';
[time, jointTorq, torqs, MF, MR, MA] = LoadData(path);

%% Show biomechanical analysis results
figure;
plot(time.*0.001, jointTorq);
xlabel('Time(sec)');
ylabel('Torque (N \cdot m)');

figure; plot(time.*0.001, MF, '-r'); hold on;
plot(time.*0.001, MA, '-b'); hold on;
plot(time.*0.001, MR, '-G');
xlabel('Time(sec)');
ylabel('%');
legend('Fatigued', 'Activated', 'Rest');