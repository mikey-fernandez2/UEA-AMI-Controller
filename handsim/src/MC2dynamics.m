function a = MC2dynamics(u, Ts, fn)

Wn = 2*pi*fn;
b = 2*Wn;
k = Wn^2;
a = zeros(size(u));
k1 = (1/Ts^2 + b/Ts + k);
k2 = -(2/Ts^2 + b/Ts);
k3 = 1/Ts^2;
for j = 1:size(u,2)
for ii = 1:size(u,1)
    if ii == 1 || ii == 2
        a(ii,j) =  0;
    else
        a(ii,j) = k*Ts^2 * u(ii, j) - k2/k1*a(ii-1,j) - k3/k1*a(ii-2,j);
    end
end

end