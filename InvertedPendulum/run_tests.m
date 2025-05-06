for init_angle=-10:0.5:10
    % With exo
    runStandingBalanceTest(true, false, true, init_angle, 0)
    % Without exo
    runStandingBalanceTest(false, false, true, init_angle, 0)
end