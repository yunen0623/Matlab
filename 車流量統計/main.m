clc; clear all; close all;
%% 创建视频图像处理对象
videofile = 'Test1.avi';
% 获取视频帧信息
info = mmfileinfo(videofile);
cols =info.Video.Width;
rows = info.Video.Height;
% 创建视频系统对象，读取视频文件
hReader = vision.VideoFileReader(videofile,...
    'ImageColorSpace', 'RGB',...       % GRB彩色空间
    'VideoOutputDataType', 'single');   % 视频输出类型
% 创建光流对象用于检测运动方向和速度
% 选择Horn-Schunck或 Lucas-Kanade
hFlow = opticalFlowHS; 
% 创建两个均值对象，用于分析光流矢量
% 等效mean(x(:))
hMean1 = vision.Mean;  
% 累计平均值，每次输入一个数据，计算所有历史输入的均值
hMean2 = vision.Mean('RunningMean', true); 
% 创建均值滤波对象，用来移除图像分割产生的噪声
hFilter = fspecial('average', [3 3]);
% 创建形态学关闭对象，填充分割以后的汽车孔洞
hClose = strel('line',5,45);
% 创建BLOB分析对象，用于从视频中分割汽车
hBlob = vision.BlobAnalysis(...
    'CentroidOutputPort', false,...
    'AreaOutputPort', true, ...
    'BoundingBoxOutputPort', true,...
    'OutputDataType', 'double', ...
    'MinimumBlobArea', 250,...
    'MaximumBlobArea', 3600,...
    'MaximumCount', 80);
% 创建形态学腐蚀对象，移除不需要的对象
hErode = strel('square',5);
% 创建形状嵌入对象，在视频中添加形状，框出汽车边界
hShape1 = vision.ShapeInserter(...
    'BorderColor', 'Custom', ...
    'CustomBorderColor', [0 1 0]);  % 绿色边框
hShape2 = vision.ShapeInserter(...
    'Shape','Lines', ...
    'BorderColor', 'Custom', ...
    'CustomBorderColor', [255 255 0]); %
% 创建视频播放对象，用来显示原始视频、运动矢量视频、汽车分割和最终处理结果
sz = get(0,'ScreenSize');  % 获取屏幕像素大小
pos = [(sz(3)-4*(cols+75))/2, (sz(4)-rows)/2 cols+60 rows+80];  % 视频播放器位置
hVideo1 = vision.VideoPlayer('Name','Original Video','Position',pos);
pos(1) = pos(1)+cols+75; % 将第2个播放器相对第1个播放器向右移动指定像素
hVideo2 = vision.VideoPlayer('Name','Motion Vector','Position',pos);
pos(1) = pos(1)+cols+75; % 将第3个播放器相对第2个播放器向右移动指定像素
hVideo3 = vision.VideoPlayer('Name','Thresholded Video','Position',pos);
pos(1) = pos(1)+cols+75;  % 将第4个播放器相对第3个播放器向右移动指定像素
hVideo4 = vision.VideoPlayer('Name','Results Video','Position',pos);

%% 从视频中检测追踪汽车
% 显示光流矢量的像素点
[xpos,ypos]=meshgrid(1:5:cols,1:5:rows);
xpos=xpos(:);
ypos=ypos(:);
locs=sub2ind([rows,cols],ypos,xpos);

% 循环处理视频每一帧，直到文件结束
while ~isDone(hReader)
    % 暂停0.3s，方便观看
    pause(0.3);
    % 从视频文件中读取视频帧
    frame  = step(hReader);
    % 将图像转换为灰度图
    gray = rgb2gray(frame);
    
    %1 计算光流场矢量，返回一个复数矩阵，分别代表每个像素点u和v
    flow = estimateFlow(hFlow,gray);
    % 每隔5行5列选择一个像素点，绘制它的光流图，20表示将光流幅值放大20倍
    % lines每行对应一条曲线，分别是第1和2个点的x,y坐标
    lines = [xpos, ypos, xpos+20*real(flow.Vx(locs)), ypos+20*imag(flow.Vy(locs))];
    % 将光流矢量添加到视频帧上
    vector = step(hShape2, frame, lines);
    
    %2 光流矢量幅值
    magnitude = flow.Magnitude;
    % 计算光流幅值平均值，表征速度阀值
    threshold = 0.5 * step(hMean2, step(hMean1, magnitude));
    % 使用阀值分割提取运动对象，然后滤波去噪
    carobj = magnitude >= threshold;
    carobj = imfilter(carobj, hFilter, 'replicate');
    % 通过形态学腐蚀去掉道路，然后形态学关闭填补BLOB（汽车）孔洞
    carobj = imerode(carobj, hErode);
    carobj = imclose(carobj, hClose);
    
    %3 统计估计BLOB（汽车）对象的面积area和边框bbox=[left,bottom,width,height]
    [area, bbox] = step(hBlob, carobj);
    % 只是统计过了杆的汽车，杆位置大约在帧图像的第22行
    grow=22;
    idx = bbox(:,1) > grow;
    % 计算汽车面积和边框面积的百分比，bbox(k,3)*bbox(k,4)是第k个边框面积
    ratio = zeros(length(idx), 1);
    ratio(idx) = single(area(idx,1))./single(bbox(idx,3).*bbox(idx,4));
    % 当面积百分比大于40%时认为是汽车，flag中为1的表示汽车
    flag = ratio > 0.4;
    % 统计视频帧中的汽车数量
    count = int32(sum(flag));
    bbox(~flag, :) = int32(-1);
    % 添加汽车边框，用于显示被追踪到的汽车
    result = step(hShape1, frame, bbox);
    % 在处理结果视频帧上添加白色横杆
    result(grow:grow+1,:,:) = 1;
    % 显示汽车数量位置的背景设为黑色
    result(1:15,1:30,:) = 0;
    % 在视频帧添加文本显示汽车数量
    result = insertText(result,[1 1],sprintf('%d',count));
    
    %4 显示最后处理结果
    step(hVideo1, frame);         % 原始的视频帧
    step(hVideo2, vector);        % 绘制光流矢量
    step(hVideo3, carobj);        % 阀值分割结果
    step(hVideo4, result);        % 带框标示汽车
end
%% 释放视频对象
release(hReader);