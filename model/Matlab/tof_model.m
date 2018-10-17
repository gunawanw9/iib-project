% Source Loction
source = [500;3000;15000];

% Reference Points - 1 to N
N = 4;
p1 = [-2000;-2000;0];
p2 = [-2000;2000;0];
p3 = [2000;-2000;0];
p4 = [2000;2000;0];
p = [p1,p2,p3,p4];

% Generate Time of Flight
t1 = norm(p1-source)/3e8;
t2 = norm(p2-source)/3e8;
t3 = norm(p3-source)/3e8;
t4 = norm(p4-source)/3e8;
t = [t1,t2,t3,t4];

% Lets model the timing error. There are three sources of error:
% 1) Extracting the TOF by cross-correlation. Our model suggests this
%    can be done to nearest sample.
% 2) Stream misalignment due to PPS skew. Testing showed that the skew
%    was normally distributed ~ N(0, 11.17e-9).
% 3) VCTCXO Drift. Initially lets limit the analysis to static measurments.

% So lets add some noise ~ N(0, 11.17e-9) and then round to the 
% nearest multiple of T = 32.55ns to simulate the above.

T = 1/(30.72e6);
drift = 0e-9;
t1_n = T*round((t1 + drift + normrnd(0, 11.17e-9))/T);
t2_n = T*round((t2 + drift + normrnd(0, 11.17e-9))/T);
t3_n = T*round((t3 + drift + normrnd(0, 11.17e-9))/T);
t4_n = T*round((t4 + drift + normrnd(0, 11.17e-9))/T);
t_n = [t1_n, t2_n, t3_n, t4_n];

% Print Times
disp(sprintf('TIMES:'));
disp(sprintf('Actual Times: (%.2f, %.2f, %.2f, %.2f)', t*1e9));
disp(sprintf('Actual Samples: (%.2f, %.2f, %.2f, %.2f)', t/T));
disp(sprintf('Noisey Times: (%.2f, %.2f, %.2f, %.2f)', t_n*1e9));
disp(sprintf('Number of Samples: (%g, %g, %g, %g)', t_n/T));
disp(sprintf(' '));


% Generate Distance Vector
r = 3e8.*t_n;

% Estimate Position
p0 = trilat_3d(p,r,N);
abs_err = abs(p0-source);
err_mag = norm(abs_err);

% Print Solution
disp(sprintf('RESULTS:'));
disp(sprintf('Actual Position: (%g, %g, %g)', source));
disp(sprintf('Position Estimate: (%g, %g, %g)', p0));
disp(sprintf('Estimate Error: (%g, %g, %g) [%g m]', abs_err, err_mag));
disp(sprintf('Z Error: %g m', abs_err(3)));

figure;
hold on
grid on
xlabel('X (m)');
ylabel('Y (m)');
title('Time of Flight Position Estimate');

% Plot the Reference Points
plot([p1(1),p2(1),p3(1),p4(1)],[p1(2),p2(2),p3(2),p4(2)],'MarkerSize',25,'Marker','.','LineStyle','none');

% Plot the True & Estimated Source Positions
plot(source(1), source(2),'MarkerSize',25,'Marker','.','LineStyle','none');
plot(p0(1), p0(2),'MarkerSize',25,'Marker','.','LineStyle','none');
