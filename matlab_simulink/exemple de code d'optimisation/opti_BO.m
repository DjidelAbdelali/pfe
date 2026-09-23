vars = [
    optimizableVariable('c1', [5, 50])
    optimizableVariable('c2', [5, 50])
    optimizableVariable('b1', [1.01, 1.99])
    optimizableVariable('b2', [0.01, 0.99])
    optimizableVariable('k', [5, 50])
    optimizableVariable('gamma', [0.01, 0.99])
];

results = bayesopt(@(x) costFunction([x.c1, x.c2, x.b1, x.b2, x.k, x.gamma]), ...
                   vars, ...
                   'MaxObjectiveEvaluations', 150, ...
                   'IsObjectiveDeterministic', true, ...
                   'AcquisitionFunctionName', 'expected-improvement-plus');

bestParams = results.XAtMinObjective;
