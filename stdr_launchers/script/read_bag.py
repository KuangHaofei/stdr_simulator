import rosbag

bag = rosbag.Bag('/home/kuang/software/Robotics/mcmc/stdr/stdr_ws/stdr_data.bag')
for topic, msg, t in bag.read_messages(topics=['/slam_out_pose', '/robot0/laser_0']):
    print msg

bag.close()