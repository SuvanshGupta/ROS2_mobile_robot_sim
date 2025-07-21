import random
import numpy 
import math
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
class solution:
    def __init__(self):
        self.best = None
        self.bestIndividual = None
        self.convergence = None
        self.optimizer = None
        self.objfname = None
        self.executionTime = None
        self.startTime = None
        self.endTime = None

class HHOPIDTuner(Node):
    def __init__(self):
        super().__init__('hho_pid_tuner')

        # Define search space for PID tuning
        self.lb = [0.01, 0.0001, 0.01]  # Lower bounds for Kp, Ki, Kd
        self.ub = [10, 1, 5]           # Upper bounds
        self.dim = 3                   # Number of parameters to optimize
        self.SearchAgents_no = 20       # Number of hawks
        self.Max_iter = 100             # Iterations for tuning

        # ROS2 Subscribers & Publishers
        self.create_subscription(Float32MultiArray, '/robot_pid_feedback', self.feedback_callback, 10)
        self.pid_gains_pub = self.create_publisher(Float32MultiArray, '/optimized_pid_gains', 10)

        # Store error & control effort data
        self.error_history = []
        self.control_effort_history = []

        self.get_logger().info("HHO PID Tuner Node Initialized.")

    def feedback_callback(self, msg):
        """Receive PID feedback: error, control effort, timestamp."""
        error, control_effort, _ = msg.data  # Extract error and control effort
        self.error_history.append(error)
        self.control_effort_history.append(control_effort)

        # Run optimization after collecting 50 data points
        if len(self.error_history) >= 50:
            self.optimize_pid_gains()
            self.error_history.clear()
            self.control_effort_history.clear()

    def cost_function(self, params):
        """Cost function to evaluate PID gains based on error history."""
        Kp, Ki, Kd = params  # Extract Kp, Ki, Kd from the optimizer

        # Calculate total squared error (goal: minimize this)
        total_error = sum(e**2 for e in self.error_history)

        # Control effort penalty (to prevent aggressive tuning)
        control_effort = sum(abs(u) for u in self.control_effort_history)

        # Weighted sum: minimize error while keeping control effort low
        return total_error + 0.01 * control_effort

    def optimize_pid_gains(self):
        """Optimize Kp, Ki, Kd using Harris Hawks Optimization (HHO) and publish them."""
        result = HHO(self.cost_function, self.lb, self.ub, self.dim, self.SearchAgents_no, self.Max_iter)
        
        optimized_Kp, optimized_Ki, optimized_Kd = result.bestIndividual  # Extract best gains

        # Publish optimized PID gains
        gains_msg = Float32MultiArray()
        gains_msg.data = [optimized_Kp, optimized_Ki, optimized_Kd]
        self.pid_gains_pub.publish(gains_msg)

        self.get_logger().info(f"Published Optimized Gains: Kp={optimized_Kp:.3f}, Ki={optimized_Ki:.3f}, Kd={optimized_Kd:.3f}")

   

def HHO(objf,lb,ub,dim,SearchAgents_no,Max_iter):

           
    # initialize the location and Energy of the rabbit
    Rabbit_Location = numpy.zeros(dim)
    Rabbit_Energy=float("inf")  #change this to -inf for maximization problems
    
    if not isinstance(lb, list):
        lb = [lb for _ in range(dim)]
        ub = [ub for _ in range(dim)]
    lb = numpy.asarray(lb)
    ub = numpy.asarray(ub)
         
    #Initialize the locations of Harris' hawks
    X = numpy.random.uniform(lb, ub, (SearchAgents_no, dim))

    
    #Initialize convergence
    convergence_curve=numpy.zeros(Max_iter)
    
    
    ############################
    s=solution()

    print("HHO is now tackling  \""+objf.__name__+"\"")    

    timerStart=time.time() 
    s.startTime=time.strftime("%Y-%m-%d-%H-%M-%S")
    ############################
    
    t=0  # Loop counter
    
    # Main loop
    while t<Max_iter:
        for i in range(0,SearchAgents_no):
            
            # Check boundries
                      
            X[i,:]=numpy.clip(X[i,:], lb, ub)
            
            # fitness of locations
            fitness=objf(X[i,:])
            
            # Update the location of Rabbit
            if fitness<Rabbit_Energy: # Change this to > for maximization problem
                Rabbit_Energy=fitness 
                Rabbit_Location=X[i,:].copy() 
            
        E1=2*(1-(t/Max_iter)) # factor to show the decreaing energy of rabbit    
        
        # Update the location of Harris' hawks 
        for i in range(0,SearchAgents_no):

            E0=2*random.random()-1  # -1<E0<1
            Escaping_Energy=E1*(E0)  # escaping energy of rabbit Eq. (3) in the paper

            # -------- Exploration phase Eq. (1) in paper -------------------

            if abs(Escaping_Energy)>=1:
                #Harris' hawks perch randomly based on 2 strategy:
                q = random.random()
                rand_Hawk_index = math.floor(SearchAgents_no*random.random())
                X_rand = X[rand_Hawk_index, :]
                if q<0.5:
                    # perch based on other family members
                    X[i,:]=X_rand-random.random()*abs(X_rand-2*random.random()*X[i,:])

                elif q>=0.5:
                    #perch on a random tall tree (random site inside group's home range)
                    X[i,:]=(Rabbit_Location - X.mean(0))-random.random()*((ub-lb)*random.random()+lb)

            # -------- Exploitation phase -------------------
            elif abs(Escaping_Energy)<1:
                #Attacking the rabbit using 4 strategies regarding the behavior of the rabbit

                #phase 1: ----- surprise pounce (seven kills) ----------
                #surprise pounce (seven kills): multiple, short rapid dives by different hawks

                r=random.random() # probablity of each event
                
                if r>=0.5 and abs(Escaping_Energy)<0.5: # Hard besiege Eq. (6) in paper
                    X[i,:]=(Rabbit_Location)-Escaping_Energy*abs(Rabbit_Location-X[i,:])

                if r>=0.5 and abs(Escaping_Energy)>=0.5:  # Soft besiege Eq. (4) in paper
                    Jump_strength=2*(1- random.random()) # random jump strength of the rabbit
                    X[i,:]=(Rabbit_Location-X[i,:])-Escaping_Energy*abs(Jump_strength*Rabbit_Location-X[i,:])
                
                #phase 2: --------performing team rapid dives (leapfrog movements)----------

                if r<0.5 and abs(Escaping_Energy)>=0.5: # Soft besiege Eq. (10) in paper
                    #rabbit try to escape by many zigzag deceptive motions
                    Jump_strength=2*(1-random.random())
                    X1=Rabbit_Location-Escaping_Energy*abs(Jump_strength*Rabbit_Location-X[i,:])
                    X1 = numpy.clip(X1, lb, ub)

                    if objf(X1)< fitness: # improved move?
                        X[i,:] = X1.copy()
                    else: # hawks perform levy-based short rapid dives around the rabbit
                        X2=Rabbit_Location-Escaping_Energy*abs(Jump_strength*Rabbit_Location-X[i,:])+numpy.multiply(numpy.random.randn(dim),Levy(dim))
                        X2 = numpy.clip(X2, lb, ub)
                        if objf(X2)< fitness:
                            X[i,:] = X2.copy()
                if r<0.5 and abs(Escaping_Energy)<0.5:   # Hard besiege Eq. (11) in paper
                     Jump_strength=2*(1-random.random())
                     X1=Rabbit_Location-Escaping_Energy*abs(Jump_strength*Rabbit_Location-X.mean(0))
                     X1 = numpy.clip(X1, lb, ub)
                     
                     if objf(X1)< fitness: # improved move?
                        X[i,:] = X1.copy()
                     else: # Perform levy-based short rapid dives around the rabbit
                         X2=Rabbit_Location-Escaping_Energy*abs(Jump_strength*Rabbit_Location-X.mean(0))+numpy.multiply(numpy.random.randn(dim),Levy(dim))
                         X2 = numpy.clip(X2, lb, ub)
                         if objf(X2)< fitness:
                            X[i,:] = X2.copy()
                
        convergence_curve[t]=Rabbit_Energy
        if (t%1==0):
               print(['At iteration '+ str(t)+ ' the best fitness is '+ str(Rabbit_Energy)])
        t=t+1
    
    timerEnd=time.time()  
    s.endTime=time.strftime("%Y-%m-%d-%H-%M-%S")
    s.executionTime=timerEnd-timerStart
    s.convergence=convergence_curve
    s.optimizer="HHO"   
    s.objfname=objf.__name__
    s.best =Rabbit_Energy 
    s.bestIndividual = Rabbit_Location
    
    
    
    return s

def Levy(dim):
    beta=1.5
    sigma=(math.gamma(1+beta)*math.sin(math.pi*beta/2)/(math.gamma((1+beta)/2)*beta*2**((beta-1)/2)))**(1/beta) 
    u= 0.01*numpy.random.randn(dim)*sigma
    v = numpy.random.randn(dim)
    zz = numpy.power(numpy.absolute(v),(1/beta))
    step = numpy.divide(u,zz)
    return step
    
def main(args=None):
    rclpy.init(args=args)
    hho_pid_tuner = HHOPIDTuner()
    rclpy.spin(hho_pid_tuner)
    hho_pid_tuner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


    
