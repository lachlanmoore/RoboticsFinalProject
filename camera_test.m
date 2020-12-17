clear; clc


images = imageDatastore(fullfile('/Users/lachlanmoore/Dropbox/F2020/Robotics 1/Project/Images'));

[imagePoints, boardsize] = detectCheckerboardPoints(images.Files);

squaresize = 25;
worldPoints = generateCheckerboardPoints(boardsize, squaresize);

imOrig = imread(fullfile('/Users/lachlanmoore/Dropbox/F2020/Robotics 1/Project/Images/newimage1.jpg'));
imageSize = [size(imOrig, 1),size(imOrig, 2)];

cameraParams = estimateCameraParameters(imagePoints, worldPoints, 'ImageSize', imageSize);


figure(1)
imshow(imOrig, 'InitialMagnification', 100);

imUndistorted = undistortImage(imOrig, cameraParams);

[imagePoints, boardsize] = detectCheckerboardPoints(imUndistorted);

[R,t] = extrinsics(imagePoints, worldPoints, cameraParams);

new_image_points = [200, 100];
% [R2, t2] = extrinsics()

zCoord = zeros(size(worldPoints, 1), 1);
worldPoints = [worldPoints zCoord];

[worldOrientation_camera, worldLocation_camera] = estimateWorldCameraPose(imagePoints, worldPoints, cameraParams);

figure(2)
pcshow(worldPoints);

hold on

plotCamera('Size', 10, 'Orientation', worldOrientation_camera, 'Location', worldLocation_camera);
hold off


figure(1)
projectedPoints = worldToImage(cameraParams, R, t, worldPoints);
hold on
plot(projectedPoints(:,1), projectedPoints(:,2), 'g-*');