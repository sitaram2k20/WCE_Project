clc;
clear all
close all;

tic; 

input_dist = [28;32;36;40;44;48;52;56;60;64;68;72;76;80;84;88;92;96;100;106];  % Distance of car from traffic light (28 - 106)m
distance_road = length(input_dist);   % Total instances taken in our real time simulation
p= 0;          % This is a random variable to break the while loop
detect = [];   % State of traffic light
percent = [];  % The percentage of particular traffic light in that transmitter circle

while distance_road>0  % all instances
    
    if distance_road == 1 && p == 1   % for break the loop 
        break
    end
    
    file_rgb = "on" + int2str(input_dist(distance_road)) + ".jpg";       
    file_rgb2 = "off" + int2str(input_dist(distance_road)) + ".jpg";     
    
    rgb  = imread(file_rgb);  % read the image with transmitter (led on) 
    [r_rgb,c_rgb,color] = size(rgb);  % dimension of image (1800*1920*3)
    
    set(imtool(rgb), 'Name', 'Input Image', 'NumberTitle', 'Off') % plot the input image
   
    rgb2  = imread(file_rgb2); % read the image without tranmitter (with led off) 

    set(imtool(rgb2), 'Name', 'Consecutive Image', 'NumberTitle', 'Off') % plot the input image with led off 

    extracted_transmitter = rgb - rgb2;  % subtract the two consecutive images

    a = rgb2gray(extracted_transmitter); % gray image of subtracted frame
    
    set(imtool(a,[]), 'Name', 'Grayscale of extracted transmitter', 'NumberTitle', 'Off')  % plot the grayscale image subtracted frame

    % To show that there is noise in subtracted image we detect the edges by canny edge detector 
    Canny_begining  = edge(a, 'Canny'); 
    set(imtool(Canny_begining, []), 'Name', 'Noise in extracted transmitter after edge detection', 'NumberTitle', 'Off') 

    % Now we know there is a noise in image so for removing it we need to sharpen the image and pass it through gaussian filter(low pass filter
    
    % We use laplacian filter for sharpens the grayscale image 
    [r,g]=size(a);  % size of grayscale image
    a=im2double(a);
    filter=[0 1 0;1 -4 1; 0 1 0]; % laplacian filter
    result=a;
    for m=2:r-1
        for j=2:g-1
            sum=0;
            row=0;
            col=1;
            for k=m-1:m+1
                row=row+1;
                col=1;
                for l=j-1:j+1
                    sum = sum+a(k,l)*filter(row,col);  % applying the filter to image     
                    col=col+1;
                end
            end
          result(m,j)=sum;      
        end
    end

    result = (a-result)+a;
    set(imtool(uint8(result),[]), 'Name', 'laplacian extracted transmitter', 'NumberTitle', 'Off') % plot the sharp image

    % The gaussian Filter func is written below
    c = gaussianFilter(result,18); % Here we use the guaussian filter to remove the noise and take the value of sigma as 18 to extract the noise. We got by greedy approach.

    set(imtool(c,[]), 'Name', 'Remove noise', 'NumberTitle', 'Off') % plot the image with removed noise

    % Get edges of image with no noise 
    Canny_img1 = edge(c, 'Canny'); 

    fin = double(Canny_img1) + a;
    set(imtool(fin,[]), 'Name', 'Superimposed with the canny edge', 'NumberTitle', 'Off')

    % Here we want to find the radius and center of the circle by using circle equation
    cir = Canny_img1;
    xy = [];

    for row=1:r_rgb
        for col=1:c_rgb
            if (cir(row,col)>0) 
                 xy = [xy  ; [col , row]];  % we replace the x,y cordinates   
            end
        end
    end

    [mm,nn]=size(xy);
    % we take three cordinates on circle 
    p1x = xy(1,1);
    p1y = xy(1,2);

    p2x = xy(round(mm/2),1);
    p2y = xy(round(mm/2),2);

    p3x = xy(mm,1);
    p3y = xy(mm,2);

    A = [p1x p1y 1;
         p2x p2y  1;
         p3x p3y  1];
    B =[-(p1x^2 + p1y^2);
        -(p2x^2 + p2y^2);
        -(p3x^2 + p3y^2) ];

    X = (inv(A))*(B);

    % get the centre and radius of circle
    xo = -(X(1,1)/2);
    yo = -(X(2,1)/2);  % centre coordinates

    radiusCannysquare = ( (round(xo) - p1x)^2 + (round(yo) -p1y)^2 );
    Csize = round(sqrt(radiusCannysquare)); % radius
   
    if input_dist(distance_road) == 106                % there is condition on (cropped image part) for detection of traffic light with 106 m and 28m
        rad = Csize - (Csize)*input_dist(distance_road)/200;     
    elseif input_dist(distance_road) == 28
        rad = Csize + (Csize)*1/10; 
    else
        rad = Csize;
    end
    
    I2 = imcrop(rgb,[ (abs(xo-rad)) (abs(yo-rad)) 2*rad 2*rad]);  % cropped original image according to centre and radius of circle
    set(imtool(I2,[]), 'Name', 'Cropped Image', 'NumberTitle', 'Off') % plot cropped original image
   
    % We want to extract the unwanted pixels so we again pass cropped image to edge detection and got our precised circle.
    a1 = rgb2gray(I2);

    % Get edges
    Canny_img = edge(a1, 'Canny'); 
    set(imtool(Canny_img,[]), 'Name', 'Final Cropped Image', 'NumberTitle', 'Off')

    % here we find the radius and center of the final precised circle. 
    [centers,radii] = imfindcircles(Canny_img,[6 max(Csize, round(rad))]); % take the min. size of circle detected is 6m 

    cx = round(centers(1,1));
    cy = round(centers(1,2)); % circle centre coordinates
    radii = round(radii);   % radius 

    imshow(Canny_img,[])
    title("final inscribing circle")
    h = viscircles(centers,radii,'Color','b'); % plot the final detected circle 
    
    % The rgy_detection() func is given below. 
    [b_prob,d,e] = rgy_detection(xo,yo,radii,rgb,distance_road,input_dist); % Here we detected the state of traffic light 
    detect = [detect ; d];
    percent = [percent ; e];
 
    % After detecting state of traffic light of first instance. we stored the centre coordinates and radius of transmitter
    % Here we can detect the state of next instance with help of centre
    % coordinates and radius of previous detection.
    try
        while b_prob < 50 && distance_road > 1  % if the percentage of rest pixels other than the (R,G,Y)> 50 the loop breaks 
            
            distance_road = distance_road -1 ;  % take the next instance
            file_rgb1 = "on" + int2str(input_dist(distance_road)) + ".jpg"; % read the next frame
            rgb1  = imread(file_rgb1);
            
            [prob,d,e] = rgy_detection(xo,yo,radii,rgb1,distance_road,input_dist); % pass the previous coordinates and radius
            detect = [detect ; d];
            percent = [percent ; e];  % detect the traffic light 
            b_prob = prob;         
        end
    catch
        continue   % If it fails to detect given previous coordinates and radius then repeat the whole process again.
    end
       
    if distance_road == 1 && p == 1
        break
    end
    
    if distance_road == 1
        p=1;
    end
end

Distance = flip(input_dist);
distance = [];
for i=1:length(Distance)
    temp = int2str(Distance(i)) + " m";
    distance = [distance; temp];
end
Percentage_pixels = percent;
T = table(distance,detect,Percentage_pixels); 

disp(T)
plot(Distance,percent)
title('Distance vs Detection Percentage of pixels')
xlabel('Distance of Receiver from transmitter')
ylabel('Percentage of pixels of detected traffic light')
toc;
 
% here we detect the color of traffic light by calculating the pixels in our precised circle
function [blackprob,d,e] = rgy_detection(cx,cy,radii,rgb,distance_road,input_dist)

    redcount = 0;
    greencount = 0;
    blackcount = 0;
    yellowcount = 0;
    totalcount = 0;
    [row1,col1,~] = size(rgb);
    
    for i=1:row1
        for j=1:col1
            
           if (( (cx - j)^2 + (cy - i)^2 )< (radii^2))
               totalcount = totalcount +1;
               
               % Here we put rgb conditions for red , yellow, green 
               if ( rgb(i,j,1)<255 &&  rgb(i,j,1)>128 && rgb(i,j,2)<100 &&  rgb(i,j,2)>=0 && rgb(i,j,3)<100 &&  rgb(i,j,3)>=0)
                   redcount = redcount+1;

               elseif ( rgb(i,j,1)<50 &&  rgb(i,j,1)>=0 && rgb(i,j,2)<255 &&  rgb(i,j,2)>=107 && rgb(i,j,3)<150 &&  rgb(i,j,3)>=0)
                   greencount = greencount +1;

               elseif (rgb(i,j,1) > 200 && rgb(i,j,2) > 200 && rgb(i,j,3) < 210)
                   yellowcount = yellowcount +1;
                   
               else
                   blackcount = blackcount +1;
               end
           end     
        end
    end    

    % These are the probabilities of trafficlights (red,green,yellow)   
    redprob = (redcount/totalcount)*100;
    greenprob = (greencount/totalcount)*100;
    yellowprob = (yellowcount/totalcount)*100;
    blackprob = (blackcount/totalcount)*100;  % percentage of rest pixels

    if (redprob>greenprob && redprob>yellowprob && redprob > 50)
        d ="RED is detected ";
        e = redprob;

    elseif(greenprob>redprob && greenprob>yellowprob  && greenprob > 50)   
        d ="GREEN is detected ";
        e = greenprob;

    elseif(blackprob > 50)    
          file = "undetected " + int2str(input_dist(distance_road)) + "m away";

    else   
        d ="YELLOW is detected ";
        e = yellowprob;
    end 
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%% Gaussian Filter  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function fil_img = gaussianFilter(image, sig)

    ker = ceil(2.9786*sig);  %  The optimised value should be 2.9786*sigma
    x = -ker:ker; 
    
    ker_gaus = exp(- x.^2 / (2*sig^2) );  % Compute the Gaussian kernel

    ker_gaus = ker_gaus .* 1/sum(sum(ker_gaus)); % Normalise kernel
    
    fil_img = image;
    
    % Convolve the image
    for i=1:2     
        if(i==2)       % We switch the direction of the kernel for the second iteration
            ker_gaus = ker_gaus';
        end      
        fil_img = imfilter(fil_img,ker_gaus,'symmetric','same');
    end
end
