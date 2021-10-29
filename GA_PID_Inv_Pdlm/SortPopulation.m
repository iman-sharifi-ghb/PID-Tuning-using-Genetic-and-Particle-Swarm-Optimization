function [pop Costs]=SortPopulation(pop)

    Costs=[pop.Cost];
    
    [Costs SortOrder]=sort(Costs);
    
    pop=pop(SortOrder);

end