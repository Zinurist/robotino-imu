#!/usr/bin/env python

from pymongo import MongoClient
import pickle
import json
import random

#for float comparison
E = 0.00001

#connection to the database
client = MongoClient()
db = client.robotino_database
col = db.samples


"""
Saves a samples object to a file.
If not using json, both load and save assume convert_to_input was called on the samples object (but no function further).
@param samples the samples object to save
@param filename path where to save to
@param use_json: use json format instead of pickel (recommended)
"""
def save(samples, filename='data.json', use_json=True):
    f = open(filename, 'w')

    if use_json:
        #todo: this can be optimized/generalised
        data_dict = {'labels': samples.labels,
                     'input_dim': samples.input_dim,
                     'data': samples.data,
                     'labels_data': samples.labels_data
                    }
        json.dump(data_dict, f, sort_keys=True, indent=4)
    else:
        pickle.dump(samples, f, 2)
        
    f.close()

"""
Loads a sample object from a file.
If not using json, both load and save assume convert_to_input was called on the samples object (but no function further).
@param filename path from where to load
@param use_json: use json format instead of pickel (recommended)
@return the samples object from the file
"""
def load(filename='data.json', use_json=True):
    f = open(filename, 'r')
	
    if use_json:
        data = json.load(f)
        samples = Samples(data['labels'])
        samples.data = data['data']
        samples.labels_data = data['labels_data']
        samples.input_dim = data['input_dim']
        samples.input_dim_org = data['input_dim']
        samples.converted = True
    else:
        samples = pickle.load(f)
        
    return samples
    f.close()



'''
This class is used as a wrapper for the sample data. It loads and preprocesses the sample data, to then use it for training.
It saves the sample data in two lists self.data and self.labels_data:

self.data is a dict with entries for each label, which contain the list of samples:
    self.data = { label : samples, ... }, ex.: label = 'curve_m', samples: [ sample1, sample2, ...]

"sample1" in the above example is a list of all data vectors (e.g. a 6-dimensional vector when using all accelerometer and gyroscope data):
    sample1 = [timestep1, timestep2, ...]
After calling unroll, this becomes a list of the windows (which are then the lists that contain the data vectors):
    sample1 = [window1, window2, ...], window1 = [timestep1, timestep2, ...], length(window1) = steps in the unroll method
This structure still saves what windows belong to the same sample (this was useful for splitting data to the test set).
    
After calling split_test, the list self.test and self.labels_test is filled with samples removed from self.data and self.labels_data. 
These represent the test or validation set (for the test set a seperate data set should be loaded, and 100% of the data can be used).

When calling the get_all/test method, a list with all the samples flattened is reduced.


The methods of this class should be called in the following order (in parentheses mean its optional):
1. load_samples 
2. convert_to_input 
3. (remove_label, set_labels_data, add_absolutes, add_moving_average)
4. unroll
5. (convert_to_onehot)
6. (split_test)
7. (flatten) 
8. get_all/test/batch

'''
class Samples:
    """
    Constructor that creates a samples object using the given labels.
    @param labels: the labels to use
    """
    def __init__(self, labels):			
        #dict containing all loaded samples for given label (as key)
        self.data = {}
        self.test = {}
        self.labels_data = {}
        self.labels_test = {}
		
		#making sure some methods arent called mutliple times (might not cover all cases!)
        self.converted = False
        self.unrolled = False
        self.flattened = False
        self.onehot = False
        
        self.labels = labels
        self.classes = len(labels)
        self.labels_dict = {}
        for label in labels:
            self.labels_dict[label] = len(self.labels_dict)
            self.data[label] = []
            self.test[label] = []
            self.labels_data[label] = []
            self.labels_test[label] = []

        #used for get_batch
        self.last_label = 0
        self.last_index = [0]*self.classes
        
            
    """
    Loads the samples from the mongo database. The parameters label and samples_ids are usually used to get all samples for a specific label. If the parameter is set to None (default), it is ignored.
    This function should be called first before all other methods of the samples object.
    
    @param label: the label to get, needs to be in the labels list of this samples object
    @param angular: picks all samples were the robot had <angular> set at its angular z speed (roughly)
    @param sample_ids: list of ids of the samples to load
    @param run: only load samples with this run value
    """
    def load_samples(self, label = None, angular = None, sample_ids = None, run = None):
        assert not self.converted
        if label not in self.labels_dict: raise ValueError('Unknown label \'%s\'' % label)
        annotations = []    

        if sample_ids is not None and len(sample_ids)>0: 
            data = [{'domains' : 'id'}]
            data2 = []
            for sample_id in sample_ids:
                data2.append({'value' : sample_id})
            if len(sample_ids)>1:
                data.append({'$or' : data2})
            else:
                data.extend(data2)
            match = {'$elemMatch' : {'$and' : data}}
            annotations.append({'annotations' : match})

        if run is not None: 
            data = [{'domains' : 'run'}, {'value' : run}]
            match = {'$elemMatch' : {'$and' : data}}
            annotations.append({'annotations' : match})
        
        if label is not None:
            match = {'$elemMatch' : {'domains' : label}}
            annotations.append({'annotations' : match})

        if angular is not None:
            data = [{'domains' : 'movement'}, {'angular.z' : 
                {"$gt": angular - E, "$lt": angular + E}}]
            match = {'$elemMatch' : {'$and' : data}}
            annotations.append({'annotations' : match})
            
        if annotations == []:
            res = col.find({})
        else:
            res = col.find({'$and' : annotations})
        
        samples = []
        labels_data = []
        for s in res:
            data = {}
            for m in s['media']:
                data[m['tags'][1]] = m['data']
            data['size'] = len(data['accel'])
            samples.append(data)
            labels_data.append([self.labels_dict[label]] * data['size'])
        self.data[label] = samples
        self.labels_data[label] = labels_data
        self.last_index.append(0)
   
   
    """
    Converts the data in this samples object to the format used for all other functions.
    What data to use can be set using the parameter strings, e.g. when accel is set to "xz", then the x and z component of the accelerometer data is used. All other components are discarded.
    The parameter size is used to cut the sample around the middle (e.g. when set to 50, 25 points before and after the middle are used).
    Should only be called after having loaded all data with load_samples, and before all other methods.
    
    @param size: window size (needs to be smaller than maximum sample length)
    @param accel,gyro,compass: string that contains the components (x/y/z) to use
    """
    def convert_to_input(self, size=None, accel='xyz', gyro='xyz', compass=''):
        assert not self.converted
        self.converted = True
        for key,samples in self.data.items():
            new_samples = []
            for s in samples:
                new_s = []
                if size is None: r = range(s['size'])
                else:
                    s1 = int(s['size']/2)
                    s2 = int(size/2)
                    lower = s1 - s2
                    upper = s1 + (size-s2)
                    r = range(lower,upper)
                for i in r:
                    new_d = []
                    if 'x' in accel: new_d.append(s['accel'][i][0])
                    if 'y' in accel: new_d.append(s['accel'][i][1])
                    if 'z' in accel: new_d.append(s['accel'][i][2])
                    if 'x' in gyro: new_d.append(s['gyro'][i][0])
                    if 'y' in gyro: new_d.append(s['gyro'][i][1])
                    if 'z' in gyro: new_d.append(s['gyro'][i][2])
                    if 'x' in compass: new_d.append(s['compass'][i][0])
                    if 'y' in compass: new_d.append(s['compass'][i][1])
                    if 'z' in compass: new_d.append(s['compass'][i][2])
                    self.input_dim = len(new_d)
                    self.input_dim_org = len(new_d)
                    new_s.append(new_d)
                new_samples.append(new_s)
            self.data[key] = new_samples
    
    
    """
    Adds the absolutes of all (original) values to the data. Should only be called before unroll/after convert_to_input.
    """
    def add_absolutes(self):
        assert self.converted
        assert not self.unrolled
        DIM = self.input_dim_org
        self.input_dim += DIM
        for key in self.data:
            for sample in self.data[key]:
                for s in range(len(sample)):
                    vals = sample[s][:DIM]
                    for v in vals:
                        sample[s].append(abs(v))
    
    """
    Adds a moving average with window size <steps> of the (original) values to the data. Should only be called before unroll/after convert_to_input.
    @param steps: window size for the moving average
    """                
    def add_moving_average(self, steps=10):
        assert self.converted
        assert not self.unrolled
        DIM = self.input_dim_org
        self.input_dim += DIM
        for key in self.data:
            for sample in self.data[key]:
                for s in range(0,steps):
                    vals_avg = [0]*DIM
                    for k in range(0,s):
                        vals = sample[k][:DIM]
                        for i in range(DIM): vals_avg[i] += vals[i]*1.0/s
                    sample[s].extend(vals_avg)
                    
                for s in range(steps, len(sample)):
                    vals_avg = [0]*DIM
                    for k in range(0,steps):
                        vals = sample[s-k][:DIM]
                        for i in range(DIM): vals_avg[i] += vals[i]*1.0/steps
                    sample[s].extend(vals_avg)
                    
    
    """
    Removes the label from this samples object. This function was mainly used to remove the foot label after having labelled the data for task 2.
    Should only be called before unroll/after convert_to_input.
    @param label: the label to remove
    """
    def remove_label(self, label):
        assert self.converted
        assert not self.unrolled
        old_labels = self.labels[:]
        if label in self.labels:
            self.labels.remove(label)
            del self.data[label]
            del self.labels_data[label]
            del self.test[label]
            del self.labels_test[label]
            
        #fix labels_data (class nums have changed)
        self.labels_dict = {}
        for label in self.labels:
            self.labels_dict[label] = len(self.labels_dict)
        for key in self.labels_data:
            new_sample = []
            for sample in self.labels_data[key]:
                new_s = []
                for s in sample:
                    old_label = old_labels[s]
                    new_s.append(self.labels_dict[old_label])
                new_sample.append(new_s)
            self.labels_data[key] = new_sample

    """
    Set the labels data. Used by the truncator script. Should only be called before unroll/after convert_to_input.
    @param labels_data: the new labels_data list
    """
    def set_labels_data(self, labels_data):
        assert self.converted
        assert not self.unrolled
        self.labels_data = labels_data
        

    #make sequences of length steps out of samples
    #ex.: [[1],[2],[3],[4]] steps=2 -> [[[1],[2]],[[2],[3]],[[3],[4]]]
    """
    Unroll the data. The parameter steps determines how the data is cut, e.g. when set to 100 each sample is cut into windows of size 100. When set to None the sample is not cut, but just copied (keep in mind that the data might need to be padded then).
    The data is cut starting at the end, with windows overlapping by <overlap_step>. 
    An extra window starting at the start of the data is added (to make sure all data points are used) when setting complete to True.
    
    TODO how the data representation changes:
    
    Should only be called before convert_to_onehot/after convert_to_input.
    
    @param steps: window size
    @param overlap_step: how much the windows overlap
    @param complete: if true, adds an extra window from the start to include all data points
    """
    def unroll(self, steps, overlap_step=1, complete=False):
        assert self.converted and not self.unrolled
        self.unrolled = True
        self.input_steps = steps
        dicts = [self.data, self.labels_data]
        self.data_old = self.data.copy()
        self.labels_data_old = self.labels_data.copy()
        for d in dicts:
            for key,values in d.items():
                new_values = []
                for val in values:
                    if steps is None or steps >= len(val):
                        new_values.append([val])
                    else:
                        new_val = []
                        for i in range(len(val) - steps, -1,-overlap_step):
                            new_val.append(val[i:i+steps])
                        if complete:
                            new_val.append(val[0:steps])
                        new_values.append(new_val)
                d[key] = new_values



    """
    Converts the labels_data list to a list of one-hot vectors (instead of integers for the classes). A label i becomes a vector v with v[i]=1 and 0 for all other components.
    Should only be called before flatten and split_test/after unroll.
    """
    def convert_to_onehot(self):
        assert self.unrolled and not self.onehot
        self.onehot = True
        onehots = []
        for i in range(self.classes):
            onehot = [0.0]*self.classes
            onehot[i] = 1.0
            onehots.append(onehot)
        for key in self.labels_data:
            samples = self.labels_data[key]
            for sample in samples:
                #unrolled samples
                for s in sample:
                    for i in range(len(s)):
                        s[i] = onehots[s[i]]

    """
    Smooths data by averaging over 3 values. Should only be called before flatten/after unroll.
    """
    def smooth(self):
        assert self.unrolled and not self.flattened
        for key in self.data:
            labels = self.labels_data[key]
            samples = self.data[key]
            for s in range(len(samples)):
                sample = samples[s]
                for i in range(len(sample)):
                    del labels[s][i][0]
                    del labels[s][i][-1]
                    window = sample[i]
                    new_window = []
                    for k in range(len(window)-2):
                        new_vec = [0]*len(window[k])
                        for g in range(len(new_vec)):
                            val = window[k][g]+2*window[k+1][g]+window[k+2][g]
                            val /= 4.0
                            new_vec[g] = val
                        new_window.append(new_vec)
                    sample[i] = new_window
                    
            

    """
    Split data by moving percentage*len(samples) (or at least one) of samples to the test set.
    Should only be called before flatten/after unroll.
    @param percentage: percentage of the data to be moved
    @param random: select the data at random (not recommended, as the order is important when reloading the data at a later time to continue training)
    """
    def split_test(self, percentage, random=False):
        assert self.unrolled and not self.flattened
        self.onehot = True
        for key,samples in self.data.items():
            if len(samples) <= 1 : continue
            to_move = max(int(len(samples)*percentage),1)
            labels = self.labels_data[key]

            for i in range(to_move):
                if random:
                    index = random.randint(0,len(samples)-1)
                else:
                    #not random -> consistent when reloading
                    index = len(samples)-1
                
                sample = samples[index]
                del samples[index]
                self.test[key].append(sample)

                label = labels[index]
                del labels[index]
                self.labels_test[key].append(label)

    #use after unroll for DNNs, since they need flattened data instead of a sequence
    #ex.: [[[1],[2]],[[2],[3]],[[3],[4]]] -> [[1,2],[2,3],[3,4]]
    #         ^--sequence: [1] then [2]         ^--all at once: [1,2]
    """
    Flattens the data for deep neural networks. These network take all timesteps at once, therefore a sample with dimension (timesteps, input_dim) needs to be converted to (timesteps*input_dim).
    Should only be called after unroll.
    """
    def flatten(self):
        assert self.unrolled and not self.flattened
        self.flattened = True
        dicts = [self.data, self.test]
        for d in dicts:
            for key,samples in d.items():
                new_samples = []
                for s in samples:
                    for seq in s:
                        new_seq = []
                        for i in seq:
                            for k in i:
                                new_seq.append(k)
                        new_samples.append(new_seq)        
                d[key] = new_samples
        dicts = [self.labels_data, self.labels_test]
        for d in dicts:
            for key,samples in d.items():
                new_samples = []
                for s in samples:
                    for seq in s:
                        new_seq = []
                        for i in seq:
                            new_seq.append(i)
                        new_samples.append(new_seq)        
                d[key] = new_samples

                
    """
    Returns a batch with mixed samples from all available classes. Not yet implemented, use Keras instead.
    Should only be called after unroll.
    """
    def get_minibatch(self, size, padding=True, use_labels_data=False):
        assert self.unrolled
        batch_x = []
        batch_y = []
        seqlen = []

        return batch_x,batch_y,seqlen
        
              
    """
    Returns a batch with mixed samples from all available classes. Not recommended, use Keras instead.
    Should only be called after unroll.
    """
    def get_batch(self, size, padding=True, use_labels_data=False):
        assert self.unrolled
        batch_x = []
        batch_y = []
        seqlen = []
        items = list(self.data.items())
        
        i = 0
        max_len = 0
        while i < size:
            ilabel = self.last_label
            samples = items[ilabel][1]
            key = items[ilabel][0]
            sample = samples[self.last_index[ilabel]]
            
            #if use_labels_data:
            labels = self.labels_data[key]
            label = labels[self.last_index[ilabel]]
            

            self.last_index[ilabel] = (self.last_index[ilabel]+1) % len(samples)
            self.last_label = (self.last_label+1) % len(items)
            
            #TODO c not needed
            c = self.labels_dict[key]
            i += 1
            if self.flattened:
                batch_x.append(sample)
                batch_y.append(label[-1])
            else:
                #take all samples of the sequence and add them to the batch
                #might make the batch bigger than wanted
                if use_labels_data:
                    batch_y.extend(label)
                else:
                    for lab in label:
                        batch_y.append(lab[-1])
                
                for s in sample:
                    #copy (to that padding doesnt affect sample)
                    batch_x.append(list(s))
                    sample_len = len(s)
                    if sample_len > max_len: max_len = sample_len
                    seqlen.append(sample_len)

        #padding is only not needed if ALL samples have max_len timesteps
        if padding:
            #pad with zeros
            zero = [0] * self.input_dim
            for b in batch_x: 
                if len(b) < max_len:
                    b.extend([zero] * (max_len-len(b)))
            if use_labels_data:
                for b in batch_y:
                    if len(b) < max_len:
                        b.extend([0] * (max_len-len(b)))
        return batch_x,batch_y,seqlen
    
    """
    Returns all training samples as a list with shape (#number samples in total, timesteps, input_dim) (or with timesteps*input_dim if flattened), with the labels as list with shape (#number samples in total, timesteps, classes) (or without timesteps when not setting use_labels_data).
    Should only be called after unroll. 
    @param padding: pad the data with zeros if needed (not recommended, use Keras/numpy instead)
    @param use_labels_data: if true, return a sequence for the labels instead of only the last one
    """
    def get_all(self, padding=True, use_labels_data=False):
        assert self.unrolled
        batch_x = []
        batch_y = []
        seqlen = []
        
        max_len = 0
        for key,samples in self.data.items():
            #TODO c not needed
            c = self.labels_dict[key]
            for i in range(len(samples)):
                sample = samples[i]
                if self.flattened:
                    batch_x.append(sample)
                    batch_y.append(self.labels_data[key][i][-1])
                else:
                    if use_labels_data:
                        batch_y.extend(self.labels_data[key][i])
                    else:
                        for lab in self.labels_data[key][i]:
                            batch_y.append(lab[-1])
                        
                    for s in sample:
                        batch_x.append(list(s))
                        sample_len = len(s)
                        if sample_len > max_len: max_len = sample_len
                        seqlen.append(sample_len)

        if padding:
            #pad with zeros
            zero = [0] * self.input_dim
            for b in batch_x: 
                if len(b) < max_len:
                    b.extend([zero] * (max_len-len(b)))
            if use_labels_data:
                for b in batch_y:
                    if len(b) < max_len:
                        b.extend([0] * (max_len-len(b)))
        return batch_x,batch_y,seqlen
    
    
    """
    Returns all test samples as a list with shape (#number samples in total, timesteps, input_dim) (or with timesteps*input_dim if flattened), with the labels as list with shape (#number samples in total, timesteps, classes) (or without timesteps when not setting use_labels_data).
    Should only be called after unroll.
    @param padding: pad the data with zeros if needed (not recommended, use Keras/numpy instead)
    @param use_labels_data: if true, return a sequence for the labels instead of only the last one
    """
    def get_test(self, padding=True, use_labels_data=False):
        assert self.unrolled
        batch_x = []
        batch_y = []
        seqlen = []
        
        max_len = 0
        for key,samples in self.test.items():
            #TODO c not needed
            c = self.labels_dict[key]
            for i in range(len(samples)):
                sample = samples[i]
                if self.flattened:
                    batch_x.append(sample)
                    batch_y.append(self.labels_test[key][i][-1])
                else:
                    if use_labels_data:
                        batch_y.extend(self.labels_test[key][i])
                    else:
                        for lab in self.labels_test[key][i]:
                            batch_y.append(lab[-1])
    
                    for s in sample:
                        batch_x.append(list(s))
                        sample_len = len(s)
                        if sample_len > max_len: max_len = sample_len
                        seqlen.append(sample_len)
        
        if padding:
            #pad with zeros
            zero = [0] * self.input_dim
            for b in batch_x: 
                if len(b) < max_len:
                    b.extend([zero] * (max_len-len(b)))
            if use_labels_data:
                for b in batch_y:
                    if len(b) < max_len:
                        b.extend([0] * (max_len-len(b)))
        return batch_x,batch_y,seqlen




