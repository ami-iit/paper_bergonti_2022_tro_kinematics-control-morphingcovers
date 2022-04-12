function plotStatsFitValues(obj)
    figure
    hold on
    stairs(obj.stats.maxFitValues ,'linewidth',1.2)
    plot(obj.stats.meanFitValues,'linewidth',1.2)
    stairs(obj.stats.minFitValues ,'linewidth',1.2)
    hold off
    grid on
    set(gca, 'XScale', 'log')
    legend('max','mean','min')
    legend('location','best')
    xlabel('Number of generation')
    ylabel('Fitness value population')
end
