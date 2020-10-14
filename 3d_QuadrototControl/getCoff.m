function [coff, A, b] = getCoff(waypoints)
n = size(waypoints,2)-1; % number of segments P1..n
A = zeros(8*n, 8*n);
b = zeros(1,8*n);

% --------------------------------------------

% YOUR CODE HERE 
% Fill A and b matices with values using loops

% for first n equations -- Total = n equations
for i=1:n
    b(1,i) = waypoints(i);
end
row = 1;
for i=1:n
    A(row,(8*(i-1)+1):8*i) = polyT(8,0,0); 
    row = row + 1;
end
% for second n equations -- Total = 2n equations
for i=1:n
    b(1,i+n) = waypoints(i+1);
end
row = n+1;
for i = 1:n
    A(row,8*(i-1) +1:8*i) = polyT(8,0,1);
    row = row + 1;
end
%for third 3 equations  -- Total  = 2n+3 equations
for i=1:3
    b(1,i+2*n) = 0;
end
row = 2*n+1;
A(row, 1:8) =  polyT(8,1,0);
A(row+1, 1:8) = polyT(8,2,0);
A(row+2,1:8) = polyT(8,3,0);
%for fourth 3 equations  -- Total  = 2n+6 equations
for i=1:3
    b(1,i+2*n+3) = 0;
end
row = 2*n+4;
A(row, n*8-7:n*8) =  polyT(8,1,1);
A(row+1, n*8-7:n*8) = polyT(8,2,1);
A(row+2,n*8-7:n*8) = polyT(8,3,1);
%for last 6*(n-1) equations  -- Total  = 2n+6n+6-6 = 8*n equations
for k=1:6*(n-1)
    b(1,2*n+6+k) = 0;
end
row = 2*n+7;
for i=1:6
    for j =1:n-1
        A(row,8*j-7:8*j+8) = [polyT(8,i,1) -polyT(8,i,0)];
        row = row+1;
    end
end
% - -------------------------------------------

coff = A\b';
end