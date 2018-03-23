%%
%Created and Written by Minsung Chris Hong, Mona Gao, Ahmed Alshareef,
%Maria Schrum.

clear
close all
clc
closeSerial();

b = Bluetooth('HC-05', 1);  % Create a bluetooth device
fopen(b);                   % open a bluetooth socket
%v = VideoWriter('test13.avi','Uncompressed AVI');
%v.FrameRate = 20;
%open(v)/
%command = ['0','1','2','3','4','5','6','7','8'];
teensyPort = 'COM18';
arduinoPort = 'COM15';

B = 115200;         % Bude rate
iteration = 50;     % One frame each seconed
END_CODE = 101010;
header = 0;    
catDetected=0;
mouseDetected=0;

% Open serial port to communicate to Teensy
COM1=serial(teensyPort,'BaudRate',115200,'OutputBufferSize',4800,'InputBufferSize',4800);
fopen(COM1);

for j=1:50
    counter = 0;
    image = zeros(0);
    searching = 1;
    
    % Wait until we can get a full image
    while searching
        header = fscanf(COM1,'%f ');
        if header(1) == END_CODE
            searching = 0;
        end
    end
    
    % Gather each frame of the image
    while counter ~= 60
        d=fscanf(COM1,'%f ');
        sz = size(d);
        if sz(1) == 80
            image=[image,d];
        end
        counter = counter+1;
    end

    image = (image-min(min(image)))/(max(max(image))-min(min(image)));
 
    image = single(image');
    %imagesc(image);
    %pause(0.02);

    %pause(1);
    objectFrame = image;
    objectFrame = imresize(objectFrame,10);
    %change to gray scale
    figure(1)
    %imshow(objectFrame)
    
    hold on
    %threshold for hot - lower the number, the less objects visible
    %IMPORTANT PARAMETER TO CHANGE
    frameHot=imcomplement(objectFrame);
    frameHot=im2bw(frameHot,.25);

%threshold for cold  - lower the number, the less objects visible
%IMPORTANT PARAMETER TO CHANGE
	frameCold=im2bw(objectFrame,.1); 
	frameCold=imcomplement(frameCold);

%reverse binary to find connected components
    frameHotReversed=imcomplement(frameHot);
    ccCat=bwconncomp(frameHotReversed,4);

%find hot part of cat
    statsCatHot=regionprops(ccCat,frameHotReversed,'Centroid','Area','MajorAxisLength','PixelList','Eccentricity','BoundingBox','Orientation');

%for testing purposes
    statsCatHot2=statsCatHot;

%set threshold for hot part of cat characteristics. THESE ARE IMPORTANT
%PARAMETERS - CHANGE IF NECESSARY
    catMin= 30;
    catMax= 650;
    catMajorAxisMin= 3;
    catMajorAxisMax= 30;
    catEccentricityMin=.2;
    catEccentricityMax=.85;
    
%get rid of blobs that do not fit criteria
    for i =1:size(ccCat.PixelIdxList,2)
        %check area
        if size(ccCat.PixelIdxList{i},1)<catMin || size(ccCat.PixelIdxList{i},1)>catMax
             ccCat.PixelIdxList{i}=[];
             statsCatHot(i).PixelList=[];
        %check axis length
        elseif statsCatHot(i).MajorAxisLength<catMajorAxisMin || statsCatHot(i).MajorAxisLength>catMajorAxisMax
             ccCat.PixelIdxList{i}=[];
             statsCatHot(i).PixelList=[];
        %check eccentricity
        elseif statsCatHot(i).Eccentricity<catEccentricityMin || statsCatHot(i).Eccentricity>catEccentricityMax
            ccCat.PixelIdxList{i}=[];
             statsCatHot(i).PixelList=[];

         end
    end

    %Delete empty cells
    emptyCells = cellfun('isempty', ccCat.PixelIdxList); 
    ccCat.PixelIdxList(emptyCells) = [];

    emptyCells2=arrayfun(@(s) isempty(s.PixelList),statsCatHot);
    statsCatHot(emptyCells2)=[];


%find cold part of cat
    hold on
    %check that only one 'cat' has been detected
    %if 0 or more than one detected, skip frame
    if size(statsCatHot,1)==1 && size(statsCatHot,2)==1
        catDetected=1;
                hold on
            plot(statsCatHot(1).Centroid(1),statsCatHot(1).Centroid(2),'r.','MarkerSize',10)
            hlen = statsCatHot(1).MajorAxisLength/2;
            xCentre = statsCatHot(1).Centroid(1);
            yCentre = statsCatHot(1).Centroid(2);
            cosOrient = cosd(statsCatHot(1).Orientation);
            sinOrient = sind(statsCatHot(1).Orientation);
            xcoords = xCentre + hlen * [cosOrient -cosOrient];
            ycoords = yCentre + hlen * [-sinOrient sinOrient];
            line(xcoords, ycoords,'LineWidth',1);
            rectangle('Position', statsCatHot(1).BoundingBox,...
        'EdgeColor','g', 'LineWidth', 1)
    
            hold on
            
            %once hot part is found, determine approximate area for entire
            %cat
            %Once this is found search for cold part in this area
            %IMPORTANT PARAMETER -CHANGE IF NECESSARY
            CATboundingBox=[statsCatHot(1).Centroid(1)-60 statsCatHot(1).Centroid(2)-60 120  120];
            rectangle('Position', CATboundingBox,...
	'EdgeColor','g', 'LineWidth', 1);


%mask cat
    %Crop the image to only leave cat, then search for cold part of cat
    cropped=imcrop(frameCold,CATboundingBox);

    %find connected components
    ccCatCold=bwconncomp(cropped,4);
    statsCatCold=regionprops(ccCatCold,cropped,'Centroid','Area','MajorAxisLength','PixelList','Eccentricity','BoundingBox','Orientation');

    %check that only one 'cat' has been detected
    %if 0 or more than one detected, skip frame
    if size(statsCatCold,1)==1 && size(statsCatCold,2)==1
        %plot rectangle around cat
        rectangle('Position', [statsCatCold(1).BoundingBox(1)+CATboundingBox(1) statsCatCold(1).BoundingBox(2)+CATboundingBox(2) statsCatCold(1).BoundingBox(3) statsCatCold(1).BoundingBox(4)],...
            'EdgeColor','g', 'LineWidth', 1);
        %get center of cold part of cat
        CatColdCenter=[statsCatCold(1).Centroid(1)+CATboundingBox(1) statsCatCold(1).Centroid(2)+CATboundingBox(2)];
        %get center of hot part of cat
        CatHotCenter=statsCatHot.Centroid;
        hold on
        line([CatColdCenter(1) CatHotCenter(1)],[CatColdCenter(2) CatHotCenter(2)],'LineWidth',1,'Color','r');
        
        %get cat vector
        vectorCat=([(CatColdCenter(1) - CatHotCenter(1)),(CatColdCenter(2) - CatHotCenter(2))]);
         % vectorCat=([CatHotCenter(1), CatHotCenter(2)]);


        
        %get midpoint of vector
        catPos =[(CatColdCenter(1) + CatHotCenter(1))/2 (CatColdCenter(2) + CatHotCenter(2))/2];
        
        %apply mask to cat so it will not be confused with mouse
        frameCold(CATboundingBox(2):CATboundingBox(2)+CATboundingBox(4),CATboundingBox(1):CATboundingBox(1)+CATboundingBox(3))=0;
        hold on
    end
    end
    hold on

    %determine location of mouse
    ccMouse=bwconncomp(frameCold, 4);
    statsMouse = regionprops(ccMouse,frameCold,'Centroid','Area','MajorAxisLength','PixelList','Eccentricity','BoundingBox','Orientation');
    statsMouse2=statsMouse;

    %set threshold for mouse characteristics. THESE ARE IMPORTANT
    %PARAMETERS - CHANGE IF NECESSARY
    mouseMin=80;
    mouseMax=800;
    mouseMajorAxisMin=10;
    mouseMajorAxisMax=50;
    mouseEccentricityMin=.3;
    mouseEccentricityMax=.77;

    for i =1:size(ccMouse.PixelIdxList,2)
        if statsMouse(i).Area<mouseMin || statsMouse(i).Area>mouseMax

             ccMouse.PixelIdxList{i}=[];
             statsMouse(i).PixelList=[];
            %plot major axis
        elseif statsMouse(i).MajorAxisLength<mouseMajorAxisMin || statsMouse(i).MajorAxisLength>mouseMajorAxisMax
             ccMouse.PixelIdxList{i}=[];
             statsMouse(i).PixelList=[];
        elseif statsMouse(i).Eccentricity<mouseEccentricityMin || statsMouse(i).Eccentricity>mouseEccentricityMax
            ccMouse.PixelIdxList{i}=[];
             statsMouse(i).PixelList=[];

        end
    end

    emptyCells = cellfun('isempty', ccMouse.PixelIdxList); 
    ccMouse.PixelIdxList(emptyCells) = [];

    emptyCells2=arrayfun(@(s) isempty(s.PixelList),statsMouse);
    statsMouse(emptyCells2)=[];

        if size(statsMouse,1)==1 && size(statsMouse,2)==1
            mouseDetected=1;
                    hold on
            plot(statsMouse(1).Centroid(1),statsMouse(1).Centroid(2),'r.','MarkerSize',10)
            hlen = statsMouse(1).MajorAxisLength/2;
            xCentre = statsMouse(1).Centroid(1);
            yCentre = statsMouse(1).Centroid(2);
            cosOrient = cosd(statsMouse(1).Orientation);
            sinOrient = sind(statsMouse(1).Orientation);
            xcoords = xCentre + hlen * [cosOrient -cosOrient];
            ycoords = yCentre + hlen * [-sinOrient sinOrient];
            hold on
            line(xcoords, ycoords,'LineWidth',2);
            rectangle('Position', statsMouse(1).BoundingBox,...
        'EdgeColor','r', 'LineWidth', 3)

          MouseCenter=statsMouse(1).Centroid;
           vectorMouse=[ (catPos(1) - MouseCenter(1)),(catPos(2) - MouseCenter(2))];
          catLine = line([catPos(1) MouseCenter(1)],[catPos(2) MouseCenter(2)],'LineWidth',1,'Color','r');

         %catLine = line([CatHotCenter(1) MouseCenter(1)],[CatHotCenter(2) MouseCenter(2)],'LineWidth',1,'Color','r');
         catMouseDist=[(catLine.XData(1)-catLine.XData(2)),(catLine.YData(1)-catLine.YData(2))];   
        end
        
        
        % aa = (vectorCat(1)*catMouseDist(2))-(catMouseDist(1)*vectorCat(2));
       % bb = (vectorCat(1)*vectorCat(2))+(catMouseDist(2)*catMouseDist(1));
        %vectorCat=[vectorCat'];
        %catMouseDist=[catMouseDist'];
        angle= acosd((vectorCat*catMouseDist')/(norm(vectorCat)*norm(catMouseDist)));
        Dist_between = sqrt((MouseCenter(1)-CatHotCenter(1))^2+(MouseCenter(2)-CatHotCenter(2))^2);
        %dir=cross(vectorCat, catMouseDist);
        %disp(dir(2))
        %D = atan2d(aa,bb)
        %theta=atand(norm(cross(vectorCat,catMouseDist)), dot(vectorCat,catMouseDist))
        x=[vectorCat';0];
        y=[catMouseDist';0];
        dir=cross(x,y);
        %disp(dir(3));
        %dot = x(1)*x(2) + y(1)*y(2);
        %det = x(1)*y(2) - y(1)*x(2);
        %angle=acosd((x*y')/norm(x)*norm(y))
        
        disp([angle,dir(3)]);
        
        
        if angle < 90 && dir(3) >0
            disp('');
            command = 5;
        
        elseif (40 < angle) &&  (angle< 1) && dir(3) <0
            disp('b');
            command = 1;
            
        elseif angle < 90 && dir(3) <0
            disp('turn left');
            command = 7;

        elseif angle >= 90 && dir(3) >0
            disp('d');
            command = 6;

        elseif angle >= 90 && dir(3) <0
            disp('e');
            command = 8;
        end
        
     fwrite(b, uint8(command));   
end

fwrite(b, uint8(0));
%closeSerial();