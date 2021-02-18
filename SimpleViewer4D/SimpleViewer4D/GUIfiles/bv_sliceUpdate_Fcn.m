function  myhandles =bv_sliceUpdate_Fcn(x,view_to_update,hObject )
%UNTITLED2 Summary of this function goes here
%  This function is called when the line is rotated or when the point is
%  moved around
myhandles = guidata(hObject);
figure(myhandles.figure1);
M2_3D = eye(4);
M2_3D(1:3,1:3)= myhandles.CurrentAxisMatrix/myhandles.Rot{view_to_update} * myhandles.Mslice{view_to_update};
M2_3D(1:3,4)=  myhandles.image_centre{view_to_update};
pos = myhandles.image_centre{view_to_update};

set(myhandles.text_x,'String', pos(1));
set(myhandles.text_y,'String', pos(2));
set(myhandles.text_z,'String', pos(3));


if numel(myhandles.im{1}.size)==4
    %4d image
    im3D = myhandles.im{1}.extractFrame(myhandles.currentFrame);
    im3D.paddingValue=0;
else
    % 3d image
    im3D = PatchType(myhandles.im{1});
    im3D.data = myhandles.im{1}.data;
end
[sl3,slice] = resliceImage(im3D,'mat',M2_3D);
if sl3~=sl3
    return;
end
rgb1 =  matrixToRGB(slice.data', myhandles.colormap{1},myhandles.windowLimits{1});

if myhandles.n_images(2)
    % add the overlay image
    if numel(myhandles.im{2}.size)==4
        %4d image
        im3D2 = myhandles.im{2}.extractFrame(myhandles.currentFrame);
        im3D2.paddingValue=0;
    else
        % 3d image
        im3D2 = ImageType(myhandles.im{2});
        im3D2.data = myhandles.im{2}.data;
    end
    
    isreg = sum(sum((myhandles.regmatrix-eye(4)).^2));
    if isreg
        % There is a registrationmatrix to apply
        %orientation = im3D2.orientation;
        %origin = im3D2.origin;

        % replace the orientation and origin taking into account the
        % regmatrix
        c0 = im3D.GetPosition((im3D.size-1)/2);
        im3D2 = transform_rigid(im3D2,myhandles.regmatrix,'interpolation','linear','centreOfRotation',c0,'matrix');
        
        %im3D2 = resampleImage(setOrientation(im3D2,myhandles.regmatrix),im3D,'interpolation','linear');
        % calculate  SSD
        % values_source =  im3D2.data(:)';
        % values_target = im3D.data(:)';
        
        
        %values_source(values_source<=im3D2.paddingValue)=NaN;
        %values_target(values_target<=im3D.paddingValue)=NaN;
        %val = (values_source - values_target).^2;
        %val = nanmean(val);
        %disp(['SSD: ' num2str(val) ])
        %M2_3D = myhandles.regmatrix*M2_3D;
        
    end
    
    
    [~,slice2] = resliceImage(im3D2,'mat',M2_3D);
    rgb2 =  matrixToRGB(slice2.data' , myhandles.colormap{2},myhandles.windowLimits{2});
    alpha2 = (abs(slice2.data')>myhandles.overly_th)*myhandles.opacity;
end


bds = slice.GetBounds();
for i=1:numel(bds)/2
    if bds((i-1)*2+2)<=bds((i-1)*2+1)
        bds((i-1)*2+2) = bds((i-1)*2+2)+  slice.spacing(i) ;
    end
end

delete(findobj(myhandles.axis_h(view_to_update),'type','image'));% if there is image data, remove it (all layers!)
delete(findobj(myhandles.axis_h(view_to_update),'type','rectangle'));% if there is image data, remove it

% Maybe change the order in which items are added to axes???--------
% underlying image. I have to set back the handle for the imline
% Add objects to plot
set(gcf,'CurrentAxes',myhandles.axis_h(view_to_update));
%hold on;
hold on;
imagesc(rgb1,'XData',[bds(1) bds(2)],'YData',[bds(3) bds(4)]); % the data is transposed because matlab has rows along y
%imagesc(rgb1,'Parent', myhandles.axis_h(view_to_update),'XData',[bds(1) bds(2)],'YData',[bds(3) bds(4)]); % the data is transposed because matlab has rows along y
%hold off;

% exterior rectangle

rectangle('Position',[bds(1) bds(3) bds(2)-bds(1) bds(4)-bds(3) ],'EdgeColor',myhandles.colors(view_to_update,:),'LineWidth',3);
%hold off;
% overly image
if myhandles.n_images(2)
    % hold on;
    imagesc(rgb2,'XData',[bds(1) bds(2)],'YData',[bds(3) bds(4)],'AlphaData', alpha2); % the data is transposed because matlab has rows along y
    %imagesc(rgb2,'Parent', myhandles.axis_h(view_to_update),'XData',[bds(1) bds(2)],'YData',[bds(3) bds(4)],'AlphaData', alpha2); % the data is transposed because matlab has rows along y
    %hold off;
end

axis(bds(:)');
axis on;
all_objects = get(myhandles.axis_h(view_to_update),'Children');

if myhandles.n_images(2)
    %set(myhandles.axis_h(view_to_update),'Children',all_objects([3:end 1 2]));
    set(myhandles.axis_h(view_to_update),'Children',all_objects([2 4:end 1 3]));
else
    %set(myhandles.axis_h(view_to_update),'Children',all_objects([2:end 1]));
    set(myhandles.axis_h(view_to_update),'Children',all_objects([1 3:end 2]));
end

%get(get(myhandles.axis_h(view_to_update),'Children'),'Type')
if myhandles.enable3D
    set(gcf,'CurrentAxes',myhandles.axis_h(4));
    if myhandles.handle_3D_slice(view_to_update);
        delete(myhandles.handle_3D_slice(view_to_update));
    end
    hold on;
    myhandles.handle_3D_slice(view_to_update) = sl3.show(); colormap(gray)
    hold off;
    axis(im3D.GetBounds()')
    axis equal
    xlabel('x')
    ylabel('y')
    zlabel('z')
else
    set(gcf,'CurrentAxes',myhandles.axis_h(4));
    axis off;
end

guidata(gcf,myhandles);



end

function out = matrixToRGB(input, cmap,limits)

ymin = 1;
ymax = size(cmap,1);

% if I do not specify otherwise...
xmin = limits(1);
xmax = limits(2);
s = size(input);

input_ = input(:);
indices  = min( [ max( [ round( (input_  -xmin)*(ymax-ymin)/(xmax-xmin) +ymin) ones(size(input_)) ],[],2) ymax*ones(size(input_))],[],2);

out_r = cmap(indices,1);
out_g = cmap(indices,2);
out_b = cmap(indices,3);

out = cat(numel(s)+1,reshape(out_r,s) , reshape(out_g,s), reshape(out_b,s));

end

