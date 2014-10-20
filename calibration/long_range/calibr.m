files = dir;
ch1 = zeros(2);
ch2 = zeros(2);
for file = 3:length(files)
    if strcmp(files(file).name,'calibr.m') == 1
        continue
    end
    dist = strrep(files(file).name,'.txt','');
    A = importdata(files(file).name);
    L = length(A.data(:,2));
    D = str2num(dist) * ones(L,1);
    ch1 = [ch1; A.data(:,2), D];
    ch2 = [ch2; A.data(:,3), D];
end

%ch1 = ch1(ch1>0);
%ch2 = ch2(ch2>0);

count1 = 0;
count2 = 0;
sum1 = 0;
sum2 = 0;
MA1 = 0;
MA2 = 0;
X1 = 0;
X2 = 0;


for i=3:length(ch1)
    if ch1(i,2) == ch1(i-1,2)
        sum1 = sum1 + ch1(i,1);
        count1 = count1+1;
    else
        MA1 = [MA1; sum1/count1];
        X1 = [X1; ch1(i-1,2)];
        sum1 = 0;
        count1 = 0;
    end
        if ch2(i,2) == ch2(i-1,2)
        sum2 = sum2 + ch2(i,1);
        count2 = count2+1;
    else
        MA2 = [MA2; sum2/count2];
        X2 = [X2; ch2(i-1,2)];
        sum2 = 0;
        count2 = 0;
        end
end

MA1 = MA1(X1>5);
MA2 = MA2(X2>5);
X1 = X1(X1>5);
X2 = X2(X2>5);