function [prob_posterior_map] = update_map(prob_prior_map,x_crdnts,y_crdnts)

    % Get the map's size
    L=size(prob_prior_map);
    
    % initialize log-odds matrices
    log_odds_prior = log(prob_prior_map./(ones(L)-prob_prior_map));
    log_odds_posterior=log_odds_prior;
    % set emprical probabilites
    p_occupied=0.75;
    p_free=1-p_occupied;
    
    % update each cell along the ray according to the Binary log-odds Bayes
    % filter and inverse sensor model, explained above.
    for i=1:length(x_crdnts)
        % if this is the last coordinate, update according to occupied cell.
        if i==length(x_crdnts)
            log_odds_posterior(y_crdnts(i),x_crdnts(i)) =  log_odds_prior(y_crdnts(i),x_crdnts(i)) + log(p_occupied/p_free);
        else
            log_odds_posterior(y_crdnts(i),x_crdnts(i)) =  log_odds_prior(y_crdnts(i),x_crdnts(i)) + log(p_free/p_occupied);
        end 
%         prob_posterior_map=ones(L) - ones(L)./(ones(L)+exp(log_odds_posterior));
%         imagesc(prob_posterior_map)
%         colormap(flipud(gray))
    end
    
    % we want to et back the probabiliries of eaqch cell and not the
    % log-odds ratios, so we an turn back each pixel according to:
    % p(cell_i)=1-1/(1+exp(l_t,i))
    prob_posterior_map=ones(L) - ones(L)./(ones(L)+exp(log_odds_posterior));

end

