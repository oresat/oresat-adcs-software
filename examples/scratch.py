
import numpy as np
import copy

class example_array(np.ndarray):
    
    def __new__(cls, input_array, info=None):
        obj = np.asarray(input_array).view(cls)
        
        # add attributes
        obj.info = info
        return obj

    def __array_finalize__(self, obj):
        # see InfoArray.__array_finalize__ for comments
        if obj is None: return
        self.info = getattr(obj, 'info', None)

    def update(self):
        '''Updates alias attributes and reference attributes.
        Clock must be attached first. Setting position and velocity will not update state'''
        # alias variables
        # try copy to make an alias
        self.position = self[0]
        self.velocity = self[1]
        pass






if __name__ == "__main__":

    pos = np.array([1, 2, 3])
    vel = np.array([9, 8, 7])

    state = example_array(np.array([pos, vel], dtype=object))
    state.update()

    print(state.position, state.velocity)

    # try doing some modifications
    blah = np.block([state[0], state[1]])

    print(blah)
    blah = blah + np.array([111, 0, 0, 0, 0, 0])
    print(blah)
    state.position = blah[:3]

    print(state)