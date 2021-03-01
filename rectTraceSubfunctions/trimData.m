function [ allData] = trimData( allData )
%trim off extra zeros
idx = find(allData.t>0,1,'last');
allData.t = allData.t(2:idx);
allData.q_desired = allData.q_desired(:,1:idx-1);
allData.q_baxter = allData.q_baxter(:,1:idx-1);
allData.q_error = allData.q_error(:,1:idx-1);
allData.q_vel_baxter = allData.q_vel_baxter(:,1:idx-1);
allData.tor_baxter = allData.tor_baxter(:,1:idx-1);
allData.q_c = allData.q_c(:,1:idx-1);
allData.q_ce = allData.q_ce(:,1:idx-1);
allData.q_bias = allData.q_bias(:,1:idx-1);
allData.ftdata = allData.ftdata(:,1:idx-1);
allData.commTor = allData.commTor(:,1:idx-1);
allData.commQ = allData.commQ(:,1:idx-1);


end

