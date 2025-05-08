for init_angle=-10:0.5:10
    for init_speed = -100:10:100
        % With exo
        runStandingBalanceTest(true, false, true, init_angle, init_speed)
        % Without exo
        runStandingBalanceTest(false, false, true, init_angle, init_speed)
    end
end