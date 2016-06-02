clear;
close all;
tic
errorfile = fopen('./logErrorskdxy.txt','r');
timefile = fopen('./logTimesdkdxy.txt','r');
actualErrors = [0, 0];
errors = textscan(errorfile,'%s','Delimiter',']/n[');
times  = textscan(timefile,'%f','Delimiter','/n');
for i = 2:4:(length(errors{1,1})-1)
    c = strsplit(errors{1,1}{i,1}, ' '); %this is one row of errors
    d = strsplit(c{1,1}, '.');
    d1 = strsplit(c{1,2}, '.');
    actualErrors = [actualErrors; [str2double(d{1,1}), str2double(d1{1,1})]];
end
timeVector = 0;
for j = 1:2:length(times{1,1})
    timeVector = [timeVector; times{1,1}(j)];
end
toc
pltlength = floor((j/2)) - 1000;
tic
fclose('all');
figure(1) 
plot(1e-10.*(timeVector(2:pltlength)), actualErrors(2:pltlength,1),'og-');
hold on;
plot(1e-10.*(timeVector(2:pltlength)), actualErrors(2:pltlength,2),'or-');
legend('errors in X','errors in Y');
toc

tic
errorvector = sqrt(actualErrors(:,1).^2 + actualErrors(:,2).^2);
figure(2)
plot(timeVector(2:pltlength),errorvector(2:pltlength));
toc