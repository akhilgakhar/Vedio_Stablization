%% Calculate Scale,Translation,Rotation from the Transformation Matrix
function [theta,translation,scale] = compute_param(H)
    
    % Rotation Matrix is the top left 2X2 matrix in transformation matrix
    R = H(1:2,1:2);
    % Compute theta from mean of two possible inverse-tangents
    theta = 0.5*(atan(R(2)/R(1))+atan(-R(3)/R(4)));
    % Compute scale from mean of two stable mean calculations
    scale = mean(R([1 4])/cos(theta));
    % Translation is the 3rd column of Transformation Matrix
    % (note: a transpose has already been taken)
    translation = H(3, 1:2);
end