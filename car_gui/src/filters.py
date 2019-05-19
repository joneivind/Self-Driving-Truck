#!/usr/bin/env python
import numpy as np

def low_pass_filter_avg(input_signal, np_array, window_size):
	
	np_array

	np_array = np.append(np_array, input_signal)
	np_array = np.delete(np_array, 0)	
	avg = np.sum(np_array/window_size)

	return avg, np_array

def low_pass_filter_median(input_signal, np_array, window_size):
	
	np_array = np.append(np_array, input_signal)
	np_array = np.delete(np_array, 0)
	median = np.sum(np.sort(np_array)[window_size/2-2:window_size/2+2])/4.0

	return median, np_array
