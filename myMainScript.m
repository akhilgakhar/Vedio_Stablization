%% Read Video & Setup Environment
clear
clc
close all hidden
[FileName,PathName] = uigetfile({'*.avi'; '*.mp4'},'Select shaky video file');

cd mmread
file = strcat(PathName,FileName);
[path,name,ext] = fileparts(file);
vid=mmread(file,[]);
cd ..
s=vid.frames;
% Make a new object for stabilized vedio 
new_vedio = s;

%% Your code here

% Defining the first frame as frame_prev
frame_prev = s(1).cdata;
% and converting it into grayscale
frame_prev=rgb2gray(frame_prev);

% number of points
fileID = fopen(string(name)+'.txt','w');
fprintf(fileID,'frame      theta            t_x           t_y          scale\n');
fprintf(fileID,'-------------------------------------------------------------\n\n');
%% Parameters for Kalman filter

pred_x = 0;
kalman_x =0;

pred_y = 0;
kalman_y =0;

pred_theta = 0;
kalman_theta =0;

kGain = 0.25;

%% Parameters for Box_Car Averaging Filter
len_filter = 6;
filter_theta = zeros(1,len_filter);
filter_x = filter_scale;
filter_y = filter_scale;

%% Global variables for display purposes

orig_theta=zeros(1,vid.nrFramesTotal);
orig_x=zeros(1,vid.nrFramesTotal);
orig_y=zeros(1,vid.nrFramesTotal);


avg_theta=zeros(1,vid.nrFramesTotal);
avg_x=zeros(1,vid.nrFramesTotal);
avg_y=zeros(1,vid.nrFramesTotal);

k_theta=zeros(1,vid.nrFramesTotal);
k_x=zeros(1,vid.nrFramesTotal);
k_y=zeros(1,vid.nrFramesTotal);

%% Buffer to store running sequence of Transformation matrix

% To check variation of (number of past transformation matrices) on output 
percent = 1;
len_filter_H_mem = floor(vid.nrFramesTotal*percent);
H_mem = repmat(eye(3),1,len_filter_H_mem);
H_mem = reshape(H_mem,[3,3,len_filter_H_mem]);

%% Main Body of the code
count = 1;
for i=2:vid.nrFramesTotal

    % collecting New Frame
    frame_new = s(i).cdata;
    frame_new=rgb2gray(frame_new);

    [theta,translation,scale]= temp_func(frame_prev,frame_new);

%%  Collecting un-processed variables for comparison

    orig_x(i) = translation(1);
    orig_y(i) = translation(2);
    orig_theta(i) = theta;
    
    th = theta;
    tr = translation;
    sc = scale;
%%%%%%%%%%%%%%%%%%%%%%%%%% Box_Car_Averaging %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % initialize the entire Box_Car_Buffer
    if i==2
        filter_theta(:)=0;
        filter_x(:)=0;
        filter_y(:)=0;
    end
    
    % Copy the latest parameters into first location
    filter_theta(1)=theta;
    filter_x(1)=translation(1);
    filter_y(1)=translation(2);
    
    % Average the values of Buffer
    box_avg_theta=sum(filter_theta)/min(i-1,len_filter);
    box_avg_x=sum(filter_x)/min(i-1,len_filter);
    box_avg_y=sum(filter_y)/min(i-1,len_filter);

    % Collecting parameters for Comparison
    avg_x(i) = box_avg_x;
    avg_y(i) = box_avg_y;
    avg_theta(i) = box_avg_theta;

    % Right shift all Buffers by one unit
    filter_theta(2:len_filter) = filter_theta(1:len_filter-1);
    filter_x(2:len_filter) = filter_x(1:len_filter-1);
    filter_y(2:len_filter) = filter_y(1:len_filter-1);

    t_x = box_avg_x;
    t_y = box_avg_y;
    box_avg_translation = [t_x t_y];

    % Taking Difference of smoothened and Original
    method = strcat('Box_Car_filter_len',num2str(len_filter));
    th = th-box_avg_theta;
    tr = tr-box_avg_translation;
    meth = 1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% kalman filtering %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
    % Uncomment this for kalman 
    pred_x = translation(1);
    pred_y = translation(2);

    % applying kalman Equations to compute all the parameters
    kalman_theta = (1-kGain)*kalman_theta + kGain*theta;
    kalman_x = (1-kGain)*kalman_x + kGain*pred_x;
    kalman_y = (1-kGain)*kalman_y + kGain*pred_y;

    t_x = kalman_x;
    t_y = kalman_y;
    kalman_translation = [t_x t_y];
    
    % For Comparison
    % Adding kalman output with Box_car_averaged_output
    k_theta(i) = kalman_theta;
    k_x(i) = kalman_x;
    k_y(i) = kalman_y;
    
    % Taking Difference of smoothened and Original
%     method = strcat('Kalman_GainVal',num2str(kGain));
%     th = th-kalman_theta;
%     tr = tr-kalman_translation;
%     meth = 2;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Writing to a file
    fprintf(fileID,'  %d     %d  %d  %d\n',i,theta,translation,scale);

%% Recompute new transform after filtering:
    % forcing Scale to be unity
    HsRt = [[[1*[cos(th) -sin(th);
                  sin(th) cos(th)]; ...
                  tr]']; [0 0 1]]';
    
    % collecting all computed transform matrices in memory
    H_mem(:,:,1)=HsRt;
    % Find Affine Equivalent
    tformsRT = affine2d(mult(H_mem));
    % right shift the buffer by unit
    H_mem(:,:,2:len_filter_H_mem) = H_mem(:,:,1:len_filter_H_mem-1);

%% Warping the next Image to match previous Image in the vedio
    new_image = imwarp(frame_new, tformsRT,'OutputView',imref2d(size(frame_prev)));
    new_vedio(i).cdata = new_image;

    % end the condetion
    frame_prev = frame_new;
    count = count+1;

end
    fclose(fileID);

%% Print the response

x=1:vid.nrFramesTotal;

if(meth ==1)
    figure1 = figure(1);
    plot(x,orig_theta,'k');    hold on;
    plot(x,avg_theta,'r');    hold off;
    title("Theta");
    %
    figure2 = figure(2);
    plot(x,orig_x,'k');    hold on;
    plot(x,avg_x,'r');    hold off;
    title("t_x");
    %
    figure3 = figure(3);
    plot(x,orig_y,'k');    hold on;
    plot(x,avg_y,'r');    hold off;
    title("t_y");
else
    figure1 = figure(1);
    plot(x,orig_theta,'k');    hold on;
    plot(x,k_theta,'b');    hold off;
    title("Theta");
    %
    figure2 = figure(2);
    plot(x,orig_x,'k');    hold on;
    plot(x,k_x,'b');    hold off;
    title("t_x");
    %
    figure3 = figure(3);
    plot(x,orig_y,'k');    hold on;
    plot(x,k_y,'b');    hold off;
    title("t_y");
end
%
%% Write Video
PathName = '../output/';
saveas(figure1,strcat(PathName,'plots/',method,'_','theta.png'));  % here you save the figure
saveas(figure2,strcat(PathName,'plots/',method,'_','t_x.png'));  % here you save the figure
saveas(figure3,strcat(PathName,'plots/',method,'_','t_y.png'));  % here you save the figure


vfile=strcat(PathName,method,'_',FileName);
ff = VideoWriter(vfile);
ff.FrameRate = 30;
open(ff)

for i=1:vid.nrFramesTotal
    f1 = s(i).cdata;
    if i==1
        f2 = new_vedio(i).cdata;
    else
        f2 = [new_vedio(i).cdata new_vedio(i).cdata new_vedio(i).cdata];
        f2 = reshape(f2,[size(new_vedio(i).cdata),3]);
    end

    vframe=horzcat(f1,f2);
    writeVideo(ff, vframe);
end
close(ff)

%% Display Video
% figure
% msgbox(strcat('Combined Video Written In ', vfile), 'Completed')
% displayvideo(new_vedio,0.01)
