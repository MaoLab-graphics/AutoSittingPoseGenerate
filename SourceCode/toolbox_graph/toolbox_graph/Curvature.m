%function [sta] = Curvature(filename)

[vertex,face] = read_mesh('zty04.ply');

tau = 1.2;

options.curvature_smoothing = 10;
[Umin,Umax,Cmin,Cmax,Cmean,Cgauss,Normal] = compute_curvature(vertex,face,options);

options.normal = [];
clf;
options.face_vertex_color = perform_saturation(Cmax,tau);
plot_mesh(vertex,face, options);
shading interp; camlight; colormap jet(256);
%saveas(gcf, 'cmax.png', 'png');

fid = fopen('D:\curvature.txt', 'w');
fwrite(fid, Cmin, 'double');
fwrite(fid, Cmax, 'double');
fwrite(fid, Cmean, 'double');
fwrite(fid, Cgauss, 'double');
sta = fclose(fid);