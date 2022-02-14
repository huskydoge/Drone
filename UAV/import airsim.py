import airsim
client=airsim.MultirotorClient()
client.confirmConnection()

client.enableApiControl(True)

client.armDisarm(True)  #阻塞

client.takeoffAsync(2.0).join()
client.moveToPositionAsync(3,3,-3,1)



