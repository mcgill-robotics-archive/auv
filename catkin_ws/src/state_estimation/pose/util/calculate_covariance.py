import rosbag
import numpy

fileName = input("rosbag file name: (eg \"name.bag\") ")
topicName = input("topic name: (eg \"/state_estimation/acc\", check rostopic list) ")
n = 0
summation = numpy.matrix([0.,0.,0.,])
summOuter = numpy.matrix([[0., 0., 0.], [0., 0., 0.],[0., 0., 0.]])
bag = rosbag.Bag(fileName)
for topic, msg, t in bag.read_messages(topics=[topicName]):
    vector= numpy.matrix([msg.x,msg.y,msg.z])
    summation += numpy.matrix([msg.x,msg.y,msg.z])
    transpose = vector.T
   # print vector
    #print transpose
   # print vector*transpose
    summOuter += transpose*vector
    n += 1
bag.close()
mean = summation/n
print summOuter/n - mean.T*mean

