function gab = calcG(twists,angles,init)
%CALCG �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
gab = init;
for ii = size(twists,2):-1:1
    gab = twistExp(twists(:,ii), angles(ii))*gab;
end
end

