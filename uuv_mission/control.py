import numpy as np

class Controller:
    def __init__(self, KP: float = 0.15, KD: float = 0.6):
        self.KP = KP
        self.KD = KD
        self.previous_error = 0.0
    
    def compute_action(self, reference: float, current_depth: float) -> float:
        """
        Compute control action using the below PD design:
        
        u[t] = KP * e[t] + KD * (e[t] - e[t-1])
        
        Args:
            reference: Reference signal r[t]
            current_depth: Current depth y[t]
        
        Returns:
            Control action u[t]
        """
        error = reference - current_depth
        derivative = error - self.previous_error
        
        action = self.KP * error + self.KD * derivative
        
        self.previous_error = error
        
        return action
    
    def reset(self):
        """Reset the controller state."""
        self.previous_error = 0.0