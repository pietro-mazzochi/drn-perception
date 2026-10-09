import threading
import numpy as np

thread_lock = threading.Lock()

class PointCloudBuffer():
    def __init__(self, buffer_max_size = 10):
        
        self.buffer_max_size = buffer_max_size
        self.buffer = []

    def clear(self):
        thread_lock.acquire()
        self.buffer.clear()
        thread_lock.release()

    def set_data(self, data):
        thread_lock.acquire()
        if len(self.buffer) >= self.buffer_max_size:
            self.buffer.pop(0)
        self.buffer.append(data)
        thread_lock.release()

    def get_data(self):
        thread_lock.acquire()
        if len(self.buffer) > 0:
            data = np.concatenate(self.buffer)
        else:
            data = self.buffer
        thread_lock.release()
        return data
    