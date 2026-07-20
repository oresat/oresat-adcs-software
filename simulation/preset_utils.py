from config import GuidanceMode, PointingReference, ControlMode
import numpy as np
import json

def npdict_to_plaindict(npdict):
    """Convert numpy arrays of dictionary to list"""
    return {
        key: val.tolist() if isinstance(val, np.ndarray) else val 
        for key,val in npdict.items()
    }

def plaindict_to_npdict(plaindict):
    """Convert lists in dictionary to numpy arrays"""
    return {
        key: np.array(val) if isinstance(val, list) else val 
        for key,val in plaindict.items()
    }


def load_preset(file):
    with open(file, 'r') as fd:
        return plaindict_to_npdict(json.load(fd))
