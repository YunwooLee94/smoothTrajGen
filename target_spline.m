close all;
clear all;
clc;
%% Data Read and Parmaeter Setting
filename_read='target_route.csv';
path_from_unreal = csvread(filename_read);

%waypoint manipulation
waypoints_x = path_from_unreal(:,2); % x and y is changed in UNREAL
waypoints_y = path_from_unreal(:,1);
% centimeter to meter
waypoints_x = 0.01*waypoints_x;
waypoints_y = 0.01*waypoints_y;
%offset compensation
waypoints_x  = waypoints_x- waypoints_x(1);
waypoints_y  = waypoints_y- waypoints_y(1);

%time interval setting
seg_dt = 4.0;
time_knots = linspace(0,seg_dt*(size(path_from_unreal,1)-1),size(path_from_unreal,1));
% init velocity
% init acceleration
vx0 = 0.0; 
vy0 = 0.0; 
ax0 = 0.0;
ay0 = 0.0;
% polynomial order 
poly_order = 6; % do not change

%% QP Generation
qp_x = qp_gen(time_knots, waypoints_x, vx0, ax0);
qp_y = qp_gen(time_knots, waypoints_y, vy0, ay0);
% solve qp
opt_px = qp_solver(qp_x);
opt_py = qp_solver(qp_y);
%% Prepare for Plot
% slicing t for visualization
num_slicing = 21;
t_seg = linspace(0,seg_dt,num_slicing);
t_seg = t_seg(1:end-1);
dt_slicing = t_seg(1,2)-t_seg(1,1);
%waypoints stack for visualization
wpts_x_vis = [];
wpts_y_vis = [];

for i=1:length(time_knots)-1
    for j = 1:length(t_seg)
        wpts_x_vis = [wpts_x_vis; opt_px((poly_order+1)*(i-1)+1:(poly_order+1)*i,1).' *t_vec(poly_order,t_seg(j),0)];
        wpts_y_vis = [wpts_y_vis; opt_py((poly_order+1)*(i-1)+1:(poly_order+1)*i,1).' *t_vec(poly_order,t_seg(j),0)];
    end
end

wpts_z_vis = ones(length(wpts_x_vis),1);
wpts_z_vis = 0.1*wpts_z_vis;
%% csv_write

wpts_x_unreal = 100*wpts_x_vis;
wpts_y_unreal = 100*wpts_y_vis;
wpts_z_unreal = 100*wpts_z_vis;
vertex_string_list = [];

for i = 1:length(wpts_x_vis)
    vertex_string_list= [vertex_string_list; 'vert'+string(i)];
end

filename_write = "target_path.csv";
% wpts_before_csv = [vertex_string_list, wpts_x_unreal, wpts_y_unreal,wpts_z_unreal];
% csvwrite(filename_write,wpts_before_csv)
T=table(vertex_string_list, wpts_y_unreal, wpts_x_unreal,wpts_z_unreal); % x y changed for UNREAL
writetable(T,filename_write)
%% Plot
figure(1)
plot(wpts_x_vis,wpts_y_vis);
grid on;
hold on;
xlabel('x[m]')
ylabel('y[m]')
%% Animation
% for i =1:length(wpts_x_vis)
%     tic
%     plot(wpts_x_vis(i),wpts_y_vis(i),'s','MarkerFaceColor','red');
%     hold on;
%     pause(dt_slicing-0.0018);
%     toc
% end







