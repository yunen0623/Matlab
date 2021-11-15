clc; clear all; close all;
%% 创建视频图像处理对象
videofile = 'viptraffic.avi';
% 获取视频信息
info = mmfileinfo(videofile);
cols =info.Video.Width;
rows = info.Video.Height;
% 创建视频系统对象，读取视频文件
hsrc = vision.VideoFileReader(videofile,'ImageColorSpace','Intensity','VideoOutputDataType','uint8');
% 创建前景检测对象
hfg = vision.ForegroundDetector(...
    'NumTrainingFrames', 5, ... % 训练帧数
    'InitialVariance', 30*30); % 初始标准方差
% 创建BLOB分析对象，用于从视频中分割汽车
hblob = vision.BlobAnalysis(...
    'CentroidOutputPort', false, 'AreaOutputPort', false, ...
    'BoundingBoxOutputPort', true, ...
    'MinimumBlobAreaSource', 'Property', 'MinimumBlobArea', 250);
% 插入检测框
hsi = vision.ShapeInserter('BorderColor','White');
% 显示窗口
hsnk = vision.VideoPlayer();
%% 从视频中检测追踪汽车
% 循环处理视频每一帧，直到文件结束
while ~isDone(hsrc)
    % 暂停0.3s，方便观看
    pause(0.3);
    frame = step(hsrc); % 读取视频帧
    fgMask = step(hfg, frame); % 获取前景（汽车）
    bbox = step(hblob, fgMask); % 提取对象
    out = step(hsi, frame, bbox); % 添加方框
    step(hsnk, out); % 查看结果
end
%% 释放视频对象
release(hsnk);
release(hsrc);