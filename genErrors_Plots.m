function [ errData,spatialData ] = genErrors_Plots( allData,arm,inds,idxseg1,idxseg2,idxseg3,idxseg4,idxseg5,numLoops )
%% Figures
if strcmp(arm,'left')
    armNum = 1;
    addTo = 0;
else
    armNum = 2;
    addTo = 7;
end
[robot_const, ~] = defineBaxter();


for ii = 1:length(allData.q_baxter)
    [spatialData.Rexp(:,:,ii),spatialData.Pexp(:,ii)] = fwdkin(robot_const(armNum).kin,allData.q_baxter(inds,ii));
end
for ii = 1:length(allData.q_desired)
    [spatialData.Rdes(:,:,ii),spatialData.Pdes(:,ii)] = fwdkin(robot_const(armNum).kin,allData.q_desired(1:7,ii));
end
spatialData.eulexp = rotm2eul(spatialData.Rexp);
spatialData.euldes = rotm2eul(spatialData.Rdes);
figure(1);subplot(2,3,1);hold on;plot(allData.t,spatialData.Pdes(1,:),'k',allData.t,spatialData.Pexp(1,:),'LineWidth',2);title('X')
subplot(2,3,2);hold on;plot(allData.t,spatialData.Pdes(2,:),'k',allData.t,spatialData.Pexp(2,:),'LineWidth',2);title('Y')
subplot(2,3,3);hold on;plot(allData.t,spatialData.Pdes(3,:),'k',allData.t,spatialData.Pexp(3,:),'LineWidth',2);title('Z')
subplot(2,3,4);hold on;plot(allData.t,spatialData.euldes(:,3)*180/pi,'k',allData.t,spatialData.eulexp(:,3)*180/pi,'LineWidth',2);title('Roll')
subplot(2,3,5);hold on;plot(allData.t,spatialData.euldes(:,2)*180/pi,'k',allData.t,spatialData.eulexp(:,2)*180/pi,'LineWidth',3);title('Pitch')
subplot(2,3,6);hold on;plot(allData.t,spatialData.euldes(:,1)*180/pi,'k',allData.t,spatialData.eulexp(:,1)*180/pi,'LineWidth',2);title('Yaw')

figure(2);
for ii = 1:6
    subplot(3,3,ii);hold on;plot(allData.t,allData.q_desired(ii,:)*180/pi,'k',allData.t,allData.q_baxter(ii+addTo,:)*180/pi,'LineWidth',2);title(num2str(ii))
end
subplot(3,3,8);hold on;plot(allData.t,allData.q_desired(7,:)*180/pi,'k',allData.t,allData.q_baxter(7+addTo,:)*180/pi,'LineWidth',2);title(num2str(7));


figure(3);
subplot(2,2,1);hold on;plot3(spatialData.Pdes(1,:),spatialData.Pdes(2,:),spatialData.Pdes(3,:),'k','LineWidth',2);
plot3(spatialData.Pexp(1,:),spatialData.Pexp(2,:),spatialData.Pexp(3,:),'LineWidth',2);xlabel('X');ylabel('Y');zlabel('Z');view(3)
plot3(spatialData.Pdes(1,1),spatialData.Pdes(2,1),spatialData.Pdes(3,1),'g<','LineWidth',3,'MarkerSize',20);
plot3(spatialData.Pdes(1,idxseg1(1)),spatialData.Pdes(2,idxseg1(1)),spatialData.Pdes(3,idxseg1(1)),'m^','LineWidth',3,'MarkerSize',20);
plot3(spatialData.Pdes(1,idxseg2(1)),spatialData.Pdes(2,idxseg2(1)),spatialData.Pdes(3,idxseg2(1)),'m>','LineWidth',3,'MarkerSize',20);
plot3(spatialData.Pdes(1,idxseg3(1)),spatialData.Pdes(2,idxseg3(1)),spatialData.Pdes(3,idxseg3(1)),'mV','LineWidth',3,'MarkerSize',20);

subplot(2,2,2);hold on;plot3(spatialData.Pdes(1,:),spatialData.Pdes(2,:),spatialData.Pdes(3,:),'k','LineWidth',2);
plot3(spatialData.Pexp(1,:),spatialData.Pexp(2,:),spatialData.Pexp(3,:),'LineWidth',2);xlabel('X');ylabel('Y');zlabel('Z');view([1 0 0])
plot3(spatialData.Pdes(1,1),spatialData.Pdes(2,1),spatialData.Pdes(3,1),'g>','LineWidth',3,'MarkerSize',20);
plot3(spatialData.Pdes(1,idxseg1(1)),spatialData.Pdes(2,idxseg1(1)),spatialData.Pdes(3,idxseg1(1)),'m^','LineWidth',3,'MarkerSize',20);
plot3(spatialData.Pdes(1,idxseg2(1)),spatialData.Pdes(2,idxseg2(1)),spatialData.Pdes(3,idxseg2(1)),'m<','LineWidth',3,'MarkerSize',20);
plot3(spatialData.Pdes(1,idxseg3(1)),spatialData.Pdes(2,idxseg3(1)),spatialData.Pdes(3,idxseg3(1)),'mV','LineWidth',3,'MarkerSize',20);


subplot(2,2,3);hold on;plot3(spatialData.Pdes(1,:),spatialData.Pdes(2,:),spatialData.Pdes(3,:),'k','LineWidth',2);
plot3(spatialData.Pexp(1,:),spatialData.Pexp(2,:),spatialData.Pexp(3,:),'LineWidth',2);xlabel('X');ylabel('Y');zlabel('Z');view([0 1 0])
plot3(spatialData.Pdes(1,1),spatialData.Pdes(2,1),spatialData.Pdes(3,1),'g>','LineWidth',3,'MarkerSize',20);
plot3(spatialData.Pdes(1,idxseg1(1)),spatialData.Pdes(2,idxseg1(1)),spatialData.Pdes(3,idxseg1(1)),'m^','LineWidth',3,'MarkerSize',20);
plot3(spatialData.Pdes(1,idxseg2(1)),spatialData.Pdes(2,idxseg2(1)),spatialData.Pdes(3,idxseg2(1)),'m<','LineWidth',3,'MarkerSize',20);
plot3(spatialData.Pdes(1,idxseg3(1)),spatialData.Pdes(2,idxseg3(1)),spatialData.Pdes(3,idxseg3(1)),'mV','LineWidth',3,'MarkerSize',20);


subplot(2,2,4);hold on;plot3(spatialData.Pdes(1,:),spatialData.Pdes(2,:),spatialData.Pdes(3,:),'k','LineWidth',2);
plot3(spatialData.Pexp(1,:),spatialData.Pexp(2,:),spatialData.Pexp(3,:),'LineWidth',2);xlabel('X');ylabel('Y');zlabel('Z');view([0 0 1])

%% generate the error norms and rmse
errData.qenorm = zeros(7,1);
errData.qermse = zeros(7,1);
for ii = 1:7
    errData.qenorm(ii) = norm(allData.q_error(ii,:));
    errData.qermse(ii) = errData.qenorm(ii)/sqrt(length(allData.q_error));
end

errData.transenorm = zeros(3,1);
errData.transermse = zeros(3,1);
for ii = 1:3
    errData.transenorm(ii) = norm((spatialData.Pdes(ii,:)-spatialData.Pexp(ii,:)));
    errData.transermse(ii) = errData.transenorm(ii)/sqrt(length(spatialData.Pdes));
end

errData.rotenorm = zeros(3,1);
errData.rotermse = zeros(3,1);
for ii = 1:3
    errData.rotenorm(ii) = norm(spatialData.euldes(:,ii)-spatialData.eulexp(:,ii));
    errData.rotermse(ii) = errData.rotenorm(ii)/sqrt(length(spatialData.euldes));
end

%% generate errors per loop
if numLoops>1
    errData.qenormPerLoop = zeros(7,numLoops); errData.qermsePerLoop = zeros(7,numLoops);
    errData.transenormPerLoop = zeros(3,numLoops); errData.transermsePerLoop = zeros(3,numLoops);
    errData.rotenormPerLoop = zeros(3,numLoops); errData.rotermsePerLoop = zeros(3,numLoops);
    for jj = 1:numLoops
        ind2 = idxseg4(jj);
        if ind2>length(allData.q_error);ind2 = length(allData.q_error);end
        if jj == 1
            ind1 = 1;
        else
            ind1 = idxseg5(jj-1);
        end
        for ii = 1:7
            errData.qenormPerLoop(ii,jj) = norm(allData.q_error(ii,ind1:ind2));
            errData.qermsePerLoop(ii,jj) = errData.qenormPerLoop(ii,jj)/sqrt(length(allData.q_error(ind1:ind2)));
        end     
        for ii = 1:3
            errData.transenormPerLoop(ii,jj) = norm((spatialData.Pdes(ii,ind1:ind2)-spatialData.Pexp(ii,ind1:ind2)));
            errData.transermsePerLoop(ii,jj) = errData.transenormPerLoop(ii,jj)/sqrt(length(spatialData.Pdes(ind1:ind2)));
            errData.rotenormPerLoop(ii,jj) = norm(spatialData.euldes(ind1:ind2,ii)-spatialData.eulexp(ind1:ind2,ii));
            errData.rotermsePerLoop(ii,jj) = errData.rotenormPerLoop(ii,jj)/sqrt(length(spatialData.euldes(ind1:ind2)));
        end
    end
end
end

