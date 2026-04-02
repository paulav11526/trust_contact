import numpy as np
import random
import matplotlib.pyplot as plt 
from sklearn.linear_model import LinearRegression


def contact_type_data(contact_type):
    speed = 0
    if contact_type >= 1:
        if random.random() > 0.75:
            speed = 0.8
        else: speed = 0.5

    else:
        if random.random() > 0.75:
            speed = 0.2
        else: speed = 0.5        
    return speed
            
 
def trust_parameter_data(trust_p):
    speed = np.square(trust_p)
    return speed


##  generating dataset

C1 = np.zeros([1000, 2]) # trust parameter
C2 = np.zeros([1000, 2]) # contact type
maxS = 2 #m/s Franka
alpha = 0.5
beta = 0.25

# generating independent random variables
for trust in range(len(C1)):
    C1[trust, 0] = random.random()
    C1[trust, 1] = trust_parameter_data(C1[trust, 0])

for i in range(len(C2)):  
    C2[i,0] = random.choice([0,1])
    C2[i,1] = contact_type_data(C2[i,0])

# joining data with: SPEED = (alpha*C1 + beta*C2)maxSPEED

speed_true = np.zeros([1000])
for i in range(len(speed_true)):
    speed_true[i] = (alpha * C1[i, 1] + beta * C2[i,1]) * maxS

# stacking C1 and C2
X = np.column_stack((C1[:,1], C2[:,1]))

## Linear Regression -------------------------------------------------------------------------
model = LinearRegression()
model.fit(X, speed_true)

print(model.coef_)
print(model.intercept_) 


## ----------------------------------------------------------
C1_new = random.random()
C2_new = random.choice([0,1])
X_new = [[C1_new, C2_new]]
predicted_speed = model.predict(X_new)

print(predicted_speed)

## ----------------------------------------------
"""
Next Steps: 
- maybe make this a ros node?
- publish the speed?
- have the arm move at the predicted_speed
"""