load 'cloud1.mat'
t = 1;
xp = [];
tic
for i =200:300
        for j=200:300
            p = cloudsamp(cloud,i,j,t);
            if p==1
               % xp = [xp [i;j]];
            end
        end
end
toc