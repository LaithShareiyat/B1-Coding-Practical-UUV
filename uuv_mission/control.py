import numpy as np

class Controller:
    def __init__(self, KP: float = 0.15, KD: float = 0.6):
        self.KP = KP
        self.KD = KD
        self.previous_error = 0.0
    
    def compute_action(self, reference: float, current_depth: float) -> float:
        error = reference - current_depth
        derivative = error - self.previous_error
        
        action = self.KP * error + self.KD * derivative
        
        self.previous_error = error
        
        return action
    
    def reset(self):
        self.previous_error = 0.0