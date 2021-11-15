clc; clear all; close all;
%% ������Ƶͼ�������
videofile = 'Test1.avi';
% ��ȡ��Ƶ֡��Ϣ
info = mmfileinfo(videofile);
cols =info.Video.Width;
rows = info.Video.Height;
% ������Ƶϵͳ���󣬶�ȡ��Ƶ�ļ�
hReader = vision.VideoFileReader(videofile,...
    'ImageColorSpace', 'RGB',...       % GRB��ɫ�ռ�
    'VideoOutputDataType', 'single');   % ��Ƶ�������
% ���������������ڼ���˶�������ٶ�
% ѡ��Horn-Schunck�� Lucas-Kanade
hFlow = opticalFlowHS; 
% ����������ֵ�������ڷ�������ʸ��
% ��Чmean(x(:))
hMean1 = vision.Mean;  
% �ۼ�ƽ��ֵ��ÿ������һ�����ݣ�����������ʷ����ľ�ֵ
hMean2 = vision.Mean('RunningMean', true); 
% ������ֵ�˲����������Ƴ�ͼ��ָ����������
hFilter = fspecial('average', [3 3]);
% ������̬ѧ�رն������ָ��Ժ�������׶�
hClose = strel('line',5,45);
% ����BLOB�����������ڴ���Ƶ�зָ�����
hBlob = vision.BlobAnalysis(...
    'CentroidOutputPort', false,...
    'AreaOutputPort', true, ...
    'BoundingBoxOutputPort', true,...
    'OutputDataType', 'double', ...
    'MinimumBlobArea', 250,...
    'MaximumBlobArea', 3600,...
    'MaximumCount', 80);
% ������̬ѧ��ʴ�����Ƴ�����Ҫ�Ķ���
hErode = strel('square',5);
% ������״Ƕ���������Ƶ�������״����������߽�
hShape1 = vision.ShapeInserter(...
    'BorderColor', 'Custom', ...
    'CustomBorderColor', [0 1 0]);  % ��ɫ�߿�
hShape2 = vision.ShapeInserter(...
    'Shape','Lines', ...
    'BorderColor', 'Custom', ...
    'CustomBorderColor', [255 255 0]); %
% ������Ƶ���Ŷ���������ʾԭʼ��Ƶ���˶�ʸ����Ƶ�������ָ�����մ�����
sz = get(0,'ScreenSize');  % ��ȡ��Ļ���ش�С
pos = [(sz(3)-4*(cols+75))/2, (sz(4)-rows)/2 cols+60 rows+80];  % ��Ƶ������λ��
hVideo1 = vision.VideoPlayer('Name','Original Video','Position',pos);
pos(1) = pos(1)+cols+75; % ����2����������Ե�1�������������ƶ�ָ������
hVideo2 = vision.VideoPlayer('Name','Motion Vector','Position',pos);
pos(1) = pos(1)+cols+75; % ����3����������Ե�2�������������ƶ�ָ������
hVideo3 = vision.VideoPlayer('Name','Thresholded Video','Position',pos);
pos(1) = pos(1)+cols+75;  % ����4����������Ե�3�������������ƶ�ָ������
hVideo4 = vision.VideoPlayer('Name','Results Video','Position',pos);

%% ����Ƶ�м��׷������
% ��ʾ����ʸ�������ص�
[xpos,ypos]=meshgrid(1:5:cols,1:5:rows);
xpos=xpos(:);
ypos=ypos(:);
locs=sub2ind([rows,cols],ypos,xpos);

% ѭ��������Ƶÿһ֡��ֱ���ļ�����
while ~isDone(hReader)
    % ��ͣ0.3s������ۿ�
    pause(0.3);
    % ����Ƶ�ļ��ж�ȡ��Ƶ֡
    frame  = step(hReader);
    % ��ͼ��ת��Ϊ�Ҷ�ͼ
    gray = rgb2gray(frame);
    
    %1 ���������ʸ��������һ���������󣬷ֱ����ÿ�����ص�u��v
    flow = estimateFlow(hFlow,gray);
    % ÿ��5��5��ѡ��һ�����ص㣬�������Ĺ���ͼ��20��ʾ��������ֵ�Ŵ�20��
    % linesÿ�ж�Ӧһ�����ߣ��ֱ��ǵ�1��2�����x,y����
    lines = [xpos, ypos, xpos+20*real(flow.Vx(locs)), ypos+20*imag(flow.Vy(locs))];
    % ������ʸ����ӵ���Ƶ֡��
    vector = step(hShape2, frame, lines);
    
    %2 ����ʸ����ֵ
    magnitude = flow.Magnitude;
    % ���������ֵƽ��ֵ�������ٶȷ�ֵ
    threshold = 0.5 * step(hMean2, step(hMean1, magnitude));
    % ʹ�÷�ֵ�ָ���ȡ�˶�����Ȼ���˲�ȥ��
    carobj = magnitude >= threshold;
    carobj = imfilter(carobj, hFilter, 'replicate');
    % ͨ����̬ѧ��ʴȥ����·��Ȼ����̬ѧ�ر��BLOB���������׶�
    carobj = imerode(carobj, hErode);
    carobj = imclose(carobj, hClose);
    
    %3 ͳ�ƹ���BLOB����������������area�ͱ߿�bbox=[left,bottom,width,height]
    [area, bbox] = step(hBlob, carobj);
    % ֻ��ͳ�ƹ��˸˵���������λ�ô�Լ��֡ͼ��ĵ�22��
    grow=22;
    idx = bbox(:,1) > grow;
    % ������������ͱ߿�����İٷֱȣ�bbox(k,3)*bbox(k,4)�ǵ�k���߿����
    ratio = zeros(length(idx), 1);
    ratio(idx) = single(area(idx,1))./single(bbox(idx,3).*bbox(idx,4));
    % ������ٷֱȴ���40%ʱ��Ϊ��������flag��Ϊ1�ı�ʾ����
    flag = ratio > 0.4;
    % ͳ����Ƶ֡�е���������
    count = int32(sum(flag));
    bbox(~flag, :) = int32(-1);
    % ��������߿�������ʾ��׷�ٵ�������
    result = step(hShape1, frame, bbox);
    % �ڴ�������Ƶ֡����Ӱ�ɫ���
    result(grow:grow+1,:,:) = 1;
    % ��ʾ��������λ�õı�����Ϊ��ɫ
    result(1:15,1:30,:) = 0;
    % ����Ƶ֡����ı���ʾ��������
    result = insertText(result,[1 1],sprintf('%d',count));
    
    %4 ��ʾ�������
    step(hVideo1, frame);         % ԭʼ����Ƶ֡
    step(hVideo2, vector);        % ���ƹ���ʸ��
    step(hVideo3, carobj);        % ��ֵ�ָ���
    step(hVideo4, result);        % �����ʾ����
end
%% �ͷ���Ƶ����
release(hReader);