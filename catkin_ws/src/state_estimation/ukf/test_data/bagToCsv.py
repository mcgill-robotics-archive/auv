import rosbag

acc = open('acc.csv','w')
gyro = open('gyro.csv', 'w')

for topic, msg, t in rosbag.Bag('test1.bag').read_messages():
    a = msg.linear_acceleration
    v = msg.angular_velocity
    acc.write('{},{},{},\n'.format(a.x,a.y,a.z))
    gyro.write('{},{},{},\n'.format(v.x,v.y,v.z))