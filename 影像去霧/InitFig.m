function InitFig(hObject,handles)
clc;
axes(handles.axes1); cla reset; axis on; box on;
set(gca, 'XTickLabel', '', 'YTickLabel', '', 'Color', [0.8039 0.8784 0.9686]);
axes(handles.axes2); cla reset; axis on; box on;
set(gca, 'XTickLabel', '', 'YTickLabel', '', 'Color', [0.8039 0.8784 0.9686]);
set(handles.textInfo, 'String', ...
    'ͼ��ȥ��ϵͳ����������ͼ����ʾ��Ȼ��ѡ��ȥ���㷨�������Թ۲�ֱ��ͼ�Ա�Ч����');