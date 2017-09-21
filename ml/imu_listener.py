#!/usr/bin/env python

#directory above needed to import controller2
import sys
sys.path.append("..")

import rospy
from geometry_msgs.msg import Twist
import controller2 as c

import time
import pygame

#if subscriber is to slow (shouldn't be), increasing the batch_size here might help

class imu_listener:
    """
    Constructor of an imu listener. To make the networks output more stable, a list of the last queue_size outcomes are saved, and the most frequent outcome is used

    @param feed_fn: the feed function for the RNN (should take a list of input vectors, and return argmax of the network output)
    @param labels: list of labels where labels[i] is the label of output i of feed_fn
    @param batch_size: batch size used for buffering imu data and then feeding it as a batch
    @param queue_size: how many outcomes of the RNN are buffered for a smoothed outcome
    """
    def __init__(self, feed_fn, labels, batch_size=1, queue_size=1, record=False):
        #RNN stuff
        self.feed_fn = feed_fn
        self.labels = labels
        self.batch = []
        self.batch_size = batch_size
        self.queue_size = queue_size
        self.res_queue = []
        self.hertz = [0]*50
        self.avg_hz = 0
        self.record = record
        if record: self.history = []
    
        #pygame for display and controls
        pygame.init()
        self.screen = pygame.display.set_mode((300,300))
        pygame.font.init()
        self.font = pygame.font.SysFont(None, 30)

        #ROS subscriber
        self.sub = rospy.Subscriber('/odom', Twist, self.process_imu)
        self.start = time.time()
        c.controller()
    
        #for exit
        pygame.quit()
        self.sub.unregister()

    """
    Function called by the subscriber to feed imu data to the RNN.
    @param val: a single input vector of imu data
    """
    def process_imu(self, val):

        #measure time since last call
        end = time.time()
        hz = 1.0/(end-self.start)
        self.start = end

        #calc hertz to make sure RNN isn't too slow for imu
        self.hertz.append(hz)
        del self.hertz[0]
        self.avg_hz = sum(self.hertz)/len(self.hertz)
        
        #collect data
        data = []
        data.append(val.linear.x)
        data.append(val.linear.y)
        data.append(val.linear.z)
        data.append(val.angular.x)
        data.append(val.angular.y)
        data.append(val.angular.z)

        #feed rnn, if batch is full
        self.batch.append(data)        
        if len(self.batch) >= self.batch_size:
            queue_size = self.queue_size
            if queue_size >1:
                res = self.feed_fn(self.batch)[0]
                self.res_queue.append(res)
                if len(self.res_queue) > queue_size: self.res_queue.pop(0)
                #get most frequent result
                freqs = {}
                for r in self.res_queue:
                    if r in freqs: freqs[r] += 1
                    else: freqs[r] = 0
                max_count = 0
                for r in freqs:
                    if freqs[r] > max_count:
                        max_count = freqs[r]
                        res = r
                #for tie choose latest label
                last = self.res_queue[-1]
                if freqs[res] == freqs[last]: res = last
            else:
                res = self.feed_fn(self.batch)[0] #[0]?
            self.batch = []

            #save result with timestamp
            if self.record: self.history.append((res,end))
            
        
            #draw info (only when batch is full)
            self.screen.fill((0,0,0))
            t1 = self.font.render('Result: %s' % self.labels[res], True, (255,255,255))
            t2 = self.font.render('%s hz' % self.avg_hz, False, (255,255,255))
            self.screen.blit(t1, (10,10))
            self.screen.blit(t2, (10,30))
            pygame.display.update()

    
    """
    Save history if recorded to the given file.
    """
    def save_history(self, filename='history.dat'):
        if not self.record:
            print("History was not recorded")
            return
        f = open(filename, 'w')
        for h in self.history:
            f.write("%s %s\n" % (self.labels[h[0]], h[1]))
        f.close()














