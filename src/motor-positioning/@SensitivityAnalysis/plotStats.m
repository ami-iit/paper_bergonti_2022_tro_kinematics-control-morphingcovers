function plotStats(obj)
    figure
    sgtitle('stats sensitivity analsysis')
    subplot(1,2,1)
    [X Y] = meshgrid(1:obj.stgs.numberCombinations,1:2*obj.stgs.numberMotorConf);
    ribbon(Y,sort(abs(obj.stats.dDeterminant_dAngle),2)')
    zlabel('$\frac{\partial det}{\partial \theta}$','interpreter','latex')
    ylabel('directions')
    xlabel('combination')
    subplot(1,2,2)
    hold on
    plot(obj.stats.divergenceDeterminant)
    yVal = ylim;
    plot(obj.indexBestCandidate*ones(2,1),yVal,'--')
    ylabel('$\nabla  \cdot det$','interpreter','latex')
    yyaxis right
    plot(abs(obj.stats.determinant_iniConf))
    ylabel('$det$','interpreter','latex')
end
