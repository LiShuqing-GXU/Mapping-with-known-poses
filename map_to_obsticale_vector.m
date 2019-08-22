%% import map 
rawData1 = importdata('or1.pgm');
[~,name] = fileparts('or1.pgm');
newData1.(genvarname(name)) = rawData1; 
vars = fieldnames(newData1);
for i = 1:length(vars)
    assignin('base', vars{i}, newData1.(vars{i}));

end
%% map parameters from yaml 
map_raw = or1;
threashold = 205;
obsticle_vector = [];
resulution = 0.05;

%% convert map to vector 
[y_size,x_size] = size(map_raw);
for ii = 1:x_size
    for jj = 1:y_size
        if map_raw(jj,ii) < threashold
            obsticle_vector = [resulution*[ii,-jj]+[-51.224998, -51.224998]+[0,resulution*y_size];obsticle_vector];
        end
    end
end

clear map_raw ii jj resulution threashold name newData1 or1 rawData1 vars x_size y_size i
