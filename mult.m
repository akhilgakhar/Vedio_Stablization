%% Function to Compute running multiplication of Transformation matrices
function out = mult(M)

nLoop = size(M,3);
% Store First element in the Series
out = M(:,:,1);
for i=2:nLoop
    out= out*M(:,:,i);
end