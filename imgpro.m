%/********************************************************************************
% 
% 	Authors : Arup Kumar Pal,Neeraj Toshniwal, Shrikant Nagori and Vaibhav Gupta
% 			  IIT Bombay
% 
% 	Project Title : This code is for a Mini Golf Robot implemented on firebird V.
% 					
% 	Copyright (c) 2012, ERTS Lab IIT Bombay erts@cse.iitb.ac.in               -*- c -*-
%    	All rights reserved.
% 
% 	Redistribution and use in source and binary forms, with or without
%    	modification, are permitted provided that the following conditions are met:
% 
%   	* Redistributions of source code must retain the above copyright
%      	notice, this list of conditions and the following disclaimer.
% 
%    	* Redistributions in binary form must reproduce the above copyright
%      	notice, this list of conditions and the following disclaimer in
%      	the documentation and/or other materials provided with the
%      	distribution.b
% 
%    	* Neither the name of the copyright holders nor the names of
%     	contributors may be used to endorse or promote products derived
%    	from this software without specific prior written permission.
% 
%    	* Source code can be used for academic purpose. 
% 	 
% 	For commercial use permission form the author needs to be taken.
% 
%   	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
%   	AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
%   	IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
%   	ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
%   	LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
%   	CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
%   	SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
%   	INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
%   	CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
%   	ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
%   	POSSIBILITY OF SUCH DAMAGE. 
% 
%   	Software released under Creative Commence cc by-nc-sa licence.
%   	For legal information refer to: 
%   	http://creativecommons.org/licenses/by-nc-sa/3.0/legalcode
% 
% 
% %********************************************************************************/
% % Tracking red and blue objects in a video in real time.
% %
% % This works on the difference between frames concept.
% % Every frame in the video is returnd as an rgb image on which we can do
% % image processing stuff.
% % Capture the video frames using the videoinput function
% % You have to replace the resolution & your installed adaptor name.

vid=videoinput('winvideo',3);

% Set the properties of the video object
set(vid, 'FramesPerTrigger', Inf);
set(vid, 'ReturnedColorspace', 'rgb')
vid.FrameGrabInterval = 5;
% This COM4 is port number through PC is connected to firebird. 
s = serial('COM4');	
fopen(s);
flag=1;
%start the video aquisition here
start(vid)
preview(vid)
% Set a loop that runs infinitely.
while(1)
    if (flag==1)
        % Get the snapshot of the current frame
        data = getsnapshot(vid);    
        % Now to track red objects in real time
        % we have to subtract the red component 
        % from the grayscale image to extract the red components in the image.
        diff_im = imsubtract(data(:,:,1), rgb2gray(data)); 
        % Convert the resulting grayscale image into a binary image.
        diff_im = im2bw(diff_im,0.18);
        % Remove all those pixels less than 300px
        diff_im = bwareaopen(diff_im,300);
        % Label all the connected components in the image.
        bw = bwlabel(diff_im, 8);    
        % Here we do the image blob analysis.
        % We get a set of properties for each labeled region.
        stats = regionprops(bw, 'BoundingBox');
        % Display the image
        imshow(data)
        %This is a loop to bound the red objects in a rectangular box.
        hold on
    
        for object = 1:length(stats)
            bb = stats(object).BoundingBox;
            c1=bb(1)+bb(3)/2;
            c2=bb(2)+bb(4)/2;
            rectangle('Position',bb,'EdgeColor','r','LineWidth',2)
            plot(c1,c2,'r*')
            plot(380,265,'b*')
            if(c1>=320 && c1<=360)
                disp('detected red ball');
                a = bb(4)/10
                a = a - mod(a,1)
                disp(a)
                fprintf(s,num2str(a));
                flag=0;
            end
        end
        hold off
    else
        % Get the snapshot of the current frame
        data = getsnapshot(vid);    
        % Now to track blue objects in real time
        % we have to subtract the blue component 
        % from the grayscale image to extract the blue components in the image.
        diff_im = imsubtract(data(:,:,3), rgb2gray(data)); 
        % Convert the resulting grayscale image into a binary image.
        diff_im = im2bw(diff_im,0.18);
        % Remove all those pixels less than 300px
        diff_im = bwareaopen(diff_im,300);
        % Label all the connected components in the image.
        bw = bwlabel(diff_im, 8);    
        % Here we do the image blob analysis.
        % We get a set of properties for each labeled region.
        stats = regionprops(bw, 'BoundingBox');
        % Display the image
        imshow(data)
        %This is a loop to bound the blue objects in a rectangular box.
        hold on
    
        for object = 1:length(stats)
            bb = stats(object).BoundingBox;
            c1=bb(1)+bb(3)/2;
            c2=bb(2)+bb(4)/2;
            rectangle('Position',bb,'EdgeColor','b','LineWidth',2)
            plot(c1,c2,'b*')
            plot(380,265,'r*')
            disp('searching flag');
            if(c1>=330 && c1<=430)
                a = bb(4)/25;
                a = a - mod(a,1);
                disp(a);
                fprintf(s,num2str(a));
                disp('found flag');
                flag=1;
            end
        end
        hold off
        
    end
        
end
% Both the loops end here.
% Stop the video aquisition.
stop(vid);
% Flush all the image data stored in the memory buffer.
flushdata(vid);
% Clear all variables
clear all
