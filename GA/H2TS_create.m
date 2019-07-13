function pop = H2TS_create(NVARS,~,options)
%   POP = H2TS_CREATE(NVARS,FITNESSFCN,OPTIONS) creates a population
%    each with a length of NVARS.
%   The arguments to the function are
%     NVARS: Number of variables
%     FITNESSFCN: Fitness function
%     OPTIONS: Options structure used by the GA
global TaskNumMat;
totalTaskNum = sum(sum(TaskNumMat));
totalPopulationSize = sum(options.PopulationSize);
pop = cell(totalPopulationSize,1);
for pop_id = 1:totalPopulationSize
    individualCell = vec2cell((randperm(totalTaskNum))',TaskNumMat);
    individualCell = feasibleValidation(individualCell,TaskNumMat,[1 1 1]);
    pop{pop_id} = cell2vec(individualCell,TaskNumMat)';
end
