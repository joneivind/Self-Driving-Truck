#!/usr/bin/env python

from math import *
from random import randint
import numpy as np

window_size = 20
sliding_window_avg = np.zeros((1, window_size))
sliding_window_median = np.zeros((1, window_size))

def low_pass_filter_avg(input_signal):

	global sliding_window_avg
	global window_size
	
	sliding_window_avg = np.append(sliding_window_avg, input_signal)
	sliding_window_avg = np.delete(sliding_window_avg, 0)	
	avg = np.sum(sliding_window_avg/window_size)

	return avg

def low_pass_filter_median(input_signal):

	global sliding_window_median
	global window_size
	
	sliding_window_median = np.append(sliding_window_median, input_signal)
	sliding_window_median = np.delete(sliding_window_median, 0)
	median = np.sum(np.sort(sliding_window_median)[window_size/2-2:window_size/2+2])/4.0

	return median

def main():

	for i in range(300):
		a = randint(0, 9)
		filtered_signal = low_pass_filter_avg(a)
		filtered_signal2 = low_pass_filter_median(a)

	print filtered_signal
	print filtered_signal2

if __name__ == '__main__':	
	main()