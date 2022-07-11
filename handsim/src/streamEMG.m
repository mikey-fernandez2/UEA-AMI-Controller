% Mikey Fernandez
% 05/31/2022
%
% stream processed iEMG from Python for testing the output of controllers

%%
clc; clear;

%% setup socket
% socket = tcpclient("127.0.0.1", 1235);
socket = tcpclient("0.0.0.0", 1235);
configureTerminator(socket, 213);
configureCallback(socket, "terminator", @(varargin)printEMG(socket))
% socket.BytesAvailableFcnMode = 'terminator';
% socket.BytesAvailableFcn = @printEMG;

% loop to read data
while(true)
    rawData = 0xd5;
    write(socket, rawData, "uint8");
%     rawData = read(socket, 1, "uint64")
%     disp([count ': ' rawData]);
    
%     if isempty(readline(socket))
%     data = typecast(readline(socket), 'uint8')
%     readline(socket)
end

function printEMG(socket)
% printEMG:
%  print the received EMG data
    data = read(socket, socket.NumBytesAvailable, 'uint8');
%    data = read(socket, 92);

%    data = dec2hex(data);
%    cellstr(data)'
    catData = compose('%02X', data)
%     cat(2, catData{1:4:end}, catData{2:4:end}, catData{3:4:end}, catData{4:4:end})
%    data = typecast(data, 'uint32')
%    disp(data)
end