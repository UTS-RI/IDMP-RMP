"""
wrapper classes for input/output of the environment
"""

from abc import ABC, abstractmethod
import tensorflow as tf

class EnvWrapper(ABC):
    """
    wrapper class for input/output of the environment
    """
    @abstractmethod
    def obs_to_policy_input(self, obs):
        """
        params obs: environment observation
        return policy_input: input to the policy
        """
        
    
    @abstractmethod
    def obs_to_value_input(self, obs):
        """
        params obs: environment observation
        return value_input: input to the value function
        """

    @abstractmethod
    def policy_output_to_action(self, actions):
        """
        params actions:  output of the policy
        return env_actions: action to the environment
        """

class URFullRMPWrapper(EnvWrapper):
    def __init__(self, dtype='float32'):
        self.dtype = dtype

    def obs_to_policy_input(self, obs):
        obs = tf.cast(obs, self.dtype)
        batch_size, obs_dim = obs.shape
        batch_size, obs_dim = int(batch_size), int(obs_dim)

        sin_q = tf.gather(obs, range(0, 6), axis=1)
        cos_q = tf.gather(obs, range(6, 12), axis=1)
        q = tf.atan2(sin_q, cos_q)
        qd = tf.gather(obs, range(12, 18), axis=1)
        controlPts = int((obs_dim-21)/4)
        dists = tf.squeeze(tf.gather(obs, range(21, 21+controlPts), axis=1))
        grads = tf.squeeze(tf.gather(obs, range(21+controlPts, obs_dim), axis=1))

        return {'q': q, 'qd': qd, 'goal': None, 'dists': dists, 'grads': grads}

    def obs_to_value_input(self, obs):
        obs = tf.cast(obs, self.dtype)
        batch_size, obs_dim = obs.shape
        batch_size, obs_dim = int(batch_size), int(obs_dim)
        obs_to_value = tf.gather(obs, range(0, obs_dim - 9), axis=1)
        return obs_to_value

    @staticmethod
    def policy_output_to_action(actions):
        return actions