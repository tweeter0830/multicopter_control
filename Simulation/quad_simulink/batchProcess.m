% W_u = linspace(0.1,10,3);
% W_p = linspace(0.1,10,3);
% W_i = linspace(0.1,10,3);
% W_d = linspace(0.1,10,3);

W_u = 7;
W_p = 0.1;
W_i = 0.1;
W_d = 5.05;

orig_state = warning;
warning('off','all');

Perf = nan(length(W_u),length(W_p),length(W_i), length(W_d));
Final= nan(length(W_u),length(W_p),length(W_i), length(W_d));
Map = cell(length(W_u),length(W_p),length(W_i), length(W_d));

hudps = dsp.UDPSender('RemoteIPAddress','127.0.0.1',...
                      'RemoteIPPort', 2040);
i_u = 0;                  
for w_u = W_u
    i_u = i_u+1;
    i_p = 0;
    for w_p = W_p
        i_i = 0;
        i_p=i_p+1;
        for w_i = W_i
            i_i= i_i+1;
            i_d = 0;
            for w_d = W_d
                i_d=i_d+1;
                indx = [i_u, i_p, i_i, i_d];
                display(indx);
                display(min(min(min(min(Perf)))));
                step(hudps,double([w_u,w_p,w_i,w_d]) )
                Map{indx(1),indx(2),indx(3),indx(4)} = [w_u,w_p,w_i,w_d];
                try
                    simOut = sim( 'systema' );
                    Perf(indx(1),indx(2),indx(3),indx(4)) = perf.Data(end,2);
                    Final(indx(1),indx(2),indx(3),indx(4)) = perf.Data(end,1);
                catch err
                    Perf(indx(1),indx(2),indx(3),indx(4)) = NaN;
                    Final(indx(1),indx(2),indx(3),indx(4)) = NaN;
                    if( strcmp(err.identifier, 'Simulink:blocks:MATLABFcnBadDataType') )
                        fprintf( 'Hit a nan, restarting\n' );
                    elseif( strcmp(err.identifier, 'Simulink:blocks:MSFB_BlockMethodFailed_NoStacktrace') )
                        fprintf( 'Timeout, Restarting\n' );
                    else
                        rethrow(err);
                    end
                end
            end
        end
    end
end
minPerf = min(min(min(min(Perf))));
minIndx = find(Perf == minPerf)
bestParams = Map{minIndx};