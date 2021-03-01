function [c, t_interp,x_interp ] = setupPathTraj( path1,rectVer,arm,tscale )
%read in the four path segments and BS trajectory for each segment
c = cell(1,5);
for ii = 1:4
    c{ii} = load([path1,rectVer,'/Segment',num2str(ii),'.mat']);
    if strcmp(arm,'right')
        c{ii}.q_lambda = [-1;1;-1;1;-1;1;-1].*c{ii}.q_lambda;
    end
    for jj = 1:7
        c{ii}.qs(jj,:) = interp1(c{ii}.lambda,c{ii}.q_lambda(jj,:),0:0.001:1);
    end
end

c{5} = load([path1,rectVer,'/LoopTransition.mat']);
if strcmp(arm,'right')
    c{5}.q_lambda = [-1;1;-1;1;-1;1;-1].*c{5}.q_lambda;
end
for jj = 1:7
    c{5}.qs(jj,:) = interp1(c{5}.lambda,c{5}.q_lambda(jj,:),0:0.001:1);
end


%%
b = load([path1,rectVer,'/traj.mat']);

%% add some buffer for interpolation
[~,idx,~] = unique(b.x);
idx = sort(idx);
t_interp = b.t(idx);
t_interp = [t_interp;t_interp(end)+(0.001:0.001:0.5)'];
t_interp = t_interp*tscale;
x_interp = b.x(idx);
x_interp = [x_interp;ones(length(0.001:0.001:0.5),1)];

end

