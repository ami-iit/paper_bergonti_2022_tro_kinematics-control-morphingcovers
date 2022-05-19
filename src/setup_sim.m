function setup_sim()
    start_full_path = fileparts(mfilename('fullpath'));
    addpath(fullfile(start_full_path,'controller'));
    addpath(fullfile(start_full_path,'motor-positioning'));
    addpath(fullfile(start_full_path,'..','meshes'));
end
