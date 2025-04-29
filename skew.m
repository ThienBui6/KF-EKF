function S = skew(v)
    % Check if input vector is 3-by-1
    if ~isequal(size(v), [3, 1])
        error('Input vector must be 3-by-1');
    end

    % Extract components of input vector
    vx = v(1);
    vy = v(2);
    vz = v(3);

    % Create skew symmetric matrix
    S = [0, -vz, vy;
         vz, 0, -vx;
         -vy, vx, 0];
end
