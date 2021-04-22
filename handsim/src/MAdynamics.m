function a = MAdynamics(u, Ts)

%'u' = normalized sEMG = (EMG(t)-minEMG)/maxEMG
%'Ts' = sampling time (e.g. 1kHz =>0.001s)

a = zeros(size(u)); % simple declaration of output matrix (no need for c++)
tauA = 0.01; % activation time constant = 10ms
tauD = 0.05; % deactivation time constant = 50ms
b = tauA/tauD; % ratio of two time constants

for j = 1:size(u,2)
for ii = 1:size(u,1)
    if ii == 1
        a(1,j) =  u(1,j); % initialization
    else
        if u(ii,j) < 0.00 % checking wheather input are lower than 0 (due to some changes in electrodes)
            u(ii,j) = 0.00;
        end
        if u(ii,j) > 1   % checking wheather input are higher than 1
            u(ii,j) = 1;
        end     
        
        
        % actuall low-pass dynamics: since this fuction is set for multiple channel it has 'j' dimension
        % However, ignore 'j' in implementing in c++
        % Test this code by a = MAdynamics(sEMG, TS)
        % sEMG will be column vector of sEMG and TS will be something like 0.001 
        a (ii,j) = (u(ii,j)/tauA + a(ii-1,j)/Ts)/(1/Ts + (b + (1-b)*u(ii,j))/tauA);  
    end
end

end