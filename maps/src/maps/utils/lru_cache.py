import collections


class LRUCache(object):
    """ Elegant implementation from https://www.kunxi.org/2014/05/lru-cache-in-python/ """
    def __init__(self, capacity):
        self.capacity = capacity
        self.cache = collections.OrderedDict()

    def get(self, key):
        try:
            value = self.cache.pop(key)
            self.cache[key] = value
            return value
        except KeyError:
            return None

    def set(self, key, value):
        try:
            self.cache.pop(key)
        except KeyError:
            if len(self.cache) >= self.capacity:
                self.cache.popitem(last=False)
        self.cache[key] = value

    def contains(self, key):
        return key in self.cache

    def clear(self):
        self.cache = collections.OrderedDict()
