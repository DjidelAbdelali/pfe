% Script MATLAB pour optimiser les paramètres d'un controleur par algorithme génétique (GA)
% en corrigeant les erreurs mentionnées (suppression de d, correction saturation/rapport 12)

function J = fonction_objet(params)
    % Extraction des paramètres d'optimisation
    b1 = params(1);
    b2 = params(2);
    c1 = params(3);
    c2 = params(4);
    k = params(5);
    gamma = params(6);
    
    % Charger le modèle Simulink
    model = 'TDC_SMC_NV';
    load_system(model);
    
    % Appliquer les paramètres au modèle
    set_param([model, '/b1'], 'Value', num2str(b1));
    set_param([model, '/b2'], 'Value', num2str(b2));
    set_param([model, '/c1'], 'Value', num2str(c1));
    set_param([model, '/c2'], 'Value', num2str(c2));
    set_param([model, '/k'], 'Value', num2str(k));
    set_param([model, '/gamma'], 'Value', num2str(gamma));
    
    % Supprimer l'entrée d (perturbation) du modèle si présente
    try
        delete_block([model, '/d']);
    catch
        % Si le bloc n'existe pas, ne rien faire
    end
    
    % Correction : saturation puis rapport 12
    set_param([model, '/Saturation'], 'UpperLimit', num2str(15/12));
    set_param([model, '/Saturation'], 'LowerLimit', num2str(-15/12));
    
    % Exécuter la simulation
    simOut = sim(model, 'StopTime', '10');
    
    % Récupérer l'erreur de suivi
    erreur = simOut.yout.getElement('erreur').Values.Data;
    
    % Calcul de la fonction objectif (erreur quadratique)
    J = sum(erreur.^2);
end

% Optimisation par algorithme génétique avec contraintes spécifiques
options = optimoptions('ga', 'PopulationSize', 20, 'MaxGenerations', 50);
[opt_params, best_cost] = ga(@fonction_objet, 6, [], [], [], [], [1.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001], [1.9999, 0.9999, 0.9999, 0.9999, 0.9999, 0.9999], [], options);

disp('Paramètres optimisés :');
disp(opt_params);
