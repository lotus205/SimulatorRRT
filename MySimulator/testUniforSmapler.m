uniformSmapler = myUniformPoseSampler([0 50 -15 15]);
poseBuffer = uniformSmapler.PoseBuffer;
figure 
plot(poseBuffer(1, :)', poseBuffer(2, :)','o')