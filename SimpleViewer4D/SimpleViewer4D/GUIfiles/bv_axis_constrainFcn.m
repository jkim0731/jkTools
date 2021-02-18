function constrained_position = bv_axis_constrainFcn( new_position, view_number )
%UNTITLED Summary of this function goes here
%   This function is called when the crosshair is rotated or panned.

SMALL_NUMBER=1E-10;

% The axis must keep the radius and the centroid
myhandles = guidata(gcbo);

% see if we are dragging or rotating the planes
old_pos = myhandles.sp_l{view_number}.getPosition();
oldvector = old_pos(2,:)-old_pos(1,:);
oldvector = oldvector/norm(oldvector);
new_vector=new_position(2,:)-new_position(1,:);
new_vector=new_vector/norm(new_vector);
isrot=true;
% There is a slight problem here ...
% if displacement is too small treat it as a rotation
ds = (new_position-old_pos);
ds2 = sum(ds(1,:)-ds(2,:));
if (abs(oldvector*new_vector'-1) < SMALL_NUMBER) && (abs(ds2)<SMALL_NUMBER)
    isrot=false;
    %disp([ 'is drag' ])
else
    % disp('is rotate')
end

ortho_vector = [-new_vector(2) new_vector(1)];

if isrot
    % handle rotations
    centroid = old_pos(1,:)+oldvector*myhandles.axis_radius*myhandles.factor;
    constrained_position(1,:)=centroid - new_vector*myhandles.axis_radius*myhandles.factor;
    constrained_position(2,:)=centroid + new_vector*myhandles.axis_radius;
    rot2D = [new_vector' ortho_vector'] ;
    newrotation = eye(3);
    otherviews = setdiff(1:3,4-view_number);
    newrotation(otherviews,otherviews)=rot2D;
    
    % create the rotation matrix associated with these axes
    
    if view_number ==1
        newrotation = newrotation'; % I invert the rotation because we are seing the slice from below
    end
    myhandles.CurrentAxisMatrix =      myhandles.CurrentAxisMatrix * newrotation/myhandles.Rot{view_number};
    myhandles.Rot{view_number} = newrotation;
else
    % handle translations
    centroid = new_position(1,:)+new_vector*myhandles.axis_radius*myhandles.factor;
    constrained_position(1,:)=centroid - new_vector*myhandles.axis_radius*myhandles.factor;
    constrained_position(2,:)=centroid + new_vector*myhandles.axis_radius;
    % update centroid
    displacement_2D =  (new_position(1,:)-old_pos(1,:))';
    displacement_3D =  myhandles.CurrentAxisMatrix/myhandles.Rot{view_number} *  myhandles.Mslice{view_number} *[displacement_2D; 0];
    myhandles.centroid = myhandles.centroid +displacement_3D;
    otherviews = setdiff(1:3,view_number);
    for i=1:2
        myhandles.image_centre{otherviews(i)} = (myhandles.image_centre{otherviews(i)}+displacement_3D);
        guidata(gcbo,myhandles);
    end
end

% We must also rotate ortho_line to be orthogonal to this line
ortho_position(1,:)=centroid - ortho_vector*myhandles.axis_radius*myhandles.factor;
ortho_position(2,:)=centroid + [-new_vector(2) new_vector(1)]*myhandles.axis_radius;


% update the non draggable line
set(myhandles.sp_line{view_number},'XData',ortho_position(:,1));
set(myhandles.sp_line{view_number},'YData',ortho_position(:,2));




% I somehow have to update the axis so that the crosshair is always at 0,0
guidata(gcbo,myhandles);

end

