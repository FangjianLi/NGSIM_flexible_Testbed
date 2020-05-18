# NGSIM_flexible_Testbed
A testbed for the autonomous driving controller/algorithms based on NGSIM datase

# What can this testbed do
1. test your high-level decison-making algorithm (e.g. lane-change decision) with real traffic data
2. test your low-level dynamic controller (e.g. vehicle following controller) with real traffic data
3. visualize your results with the animations

# What are already in the codes
1. NGSIM data processing part
3. Neighbor vehicles motion state tracking part (it can provide (x,y) informaton of the front car, left-front car, left rear car, right-front car, and right-rear car)
4. pre-defined vehicle dynamic model (bicycle model)
5. pre-defined low-level controllers: vehicle speed controller, vehicle following controller, lane-keeping controller, and lane-changing controller 
6. pre-defined psuedo-lane-change decision making part
7. animation part

# How to play with this testbed
1. choose which car you are interested in (the testbed will duplicate a car operated under your algorithms)
2. design/modify the high-level or low-level controller whichever you want
3. show check the results

