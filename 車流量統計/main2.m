clc; clear all; close all;
%% ������Ƶͼ�������
videofile = 'viptraffic.avi';
% ��ȡ��Ƶ��Ϣ
info = mmfileinfo(videofile);
cols =info.Video.Width;
rows = info.Video.Height;
% ������Ƶϵͳ���󣬶�ȡ��Ƶ�ļ�
hsrc = vision.VideoFileReader(videofile,'ImageColorSpace','Intensity','VideoOutputDataType','uint8');
% ����ǰ��������
hfg = vision.ForegroundDetector(...
    'NumTrainingFrames', 5, ... % ѵ��֡��
    'InitialVariance', 30*30); % ��ʼ��׼����
% ����BLOB�����������ڴ���Ƶ�зָ�����
hblob = vision.BlobAnalysis(...
    'CentroidOutputPort', false, 'AreaOutputPort', false, ...
    'BoundingBoxOutputPort', true, ...
    'MinimumBlobAreaSource', 'Property', 'MinimumBlobArea', 250);
% �������
hsi = vision.ShapeInserter('BorderColor','White');
% ��ʾ����
hsnk = vision.VideoPlayer();
%% ����Ƶ�м��׷������
% ѭ��������Ƶÿһ֡��ֱ���ļ�����
while ~isDone(hsrc)
    % ��ͣ0.3s������ۿ�
    pause(0.3);
    frame = step(hsrc); % ��ȡ��Ƶ֡
    fgMask = step(hfg, frame); % ��ȡǰ����������
    bbox = step(hblob, fgMask); % ��ȡ����
    out = step(hsi, frame, bbox); % ��ӷ���
    step(hsnk, out); % �鿴���
end
%% �ͷ���Ƶ����
release(hsnk);
release(hsrc);