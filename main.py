import gymnasium as gym
#import sympy
#xxx=sympy.symbols('x')

class PID:
    def __init__(self, kp,ki, kd, goal):
        self.kp=kp
        self.kd=kd
        #self.ki=ki
        self.goal=goal
        self.last_error=0

    def observe(self, x):
        error=self.goal-x
        d_error=error-self.last_error
        #i_int=sympy.integrate(error, xxx)
        self.last_error=error
        
        return self.kp*error+self.kd*d_error


class Controller:
    def __init__(self):
        self.cart=PID(kp=0.5, ki=2, kd=65, goal=0)
        self.pole=PID(kp=5, ki=2, kd=65, goal=0)

    def observe(self, cart_position, pole_angle):
        u_cart=self.cart.observe(cart_position)
        u_pole=self.pole.observe(pole_angle)
        action=1 if u_cart+u_pole<0 else 0 #1-right 0-left
        return action




env=gym.make("CartPole-v1", render_mode='human')

observation, info=env.reset()
#observation[0] = cart position
#observation[2] = pole angle
controller=Controller()

for i in range(1000):
    cart_position=observation[0] 
    pole_angle=observation[2]
    action=controller.observe(cart_position, pole_angle)
    observation, reward, terminated, truncated, info=env.step(action)
    
    if terminated or truncated:
        observation, info=env.reset()

env.close()
