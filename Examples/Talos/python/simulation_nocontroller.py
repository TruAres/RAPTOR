import numpy as np
import pinocchio as pin
from pathlib import Path
import scipy.io


urdf_filename =  "../../../Robots/talos/talos_reduced_armfixed_floatingbase.urdf"
model = pin.buildModelFromUrdf(urdf_filename)
data = model.createData()

nq = model.nq
nv = model.nv
nu = nv - 6


step_length = 0.4
trajectories = np.loadtxt('../data/solution-talos-forward-' + str(step_length) + '.txt')

ts_raptor = np.linspace(0, 0.8, len(trajectories)) 
xs_raptor = np.zeros((len(ts_raptor), nq + nv)) 
us_raptor = np.zeros((len(ts_raptor), nu))

for i, states in enumerate(trajectories):
    q = states[:nv]
    v = states[nv:(2*nv)]
    u = states[(2*nv):]
    
    xs_raptor[i, :nq] = q
    xs_raptor[i, nq:] = v
    us_raptor[i, :] = u

pos_sim = xs_raptor[:, :nq]
vel_sim = xs_raptor[:, nq:]
us_sim = us_raptor

e_sim = np.zeros_like(pos_sim)
edot_sim = np.zeros_like(vel_sim)


RF_id = model.getFrameId("right_sole_link")
pin.forwardKinematics(model, data, xs_raptor[-1][:nq])
pin.updateFramePlacements(model, data)
RF_placement = data.oMf[RF_id]
step_length_opt = RF_placement.translation[0]
step_length_sim = step_length_opt 

print(step_length_opt, step_length_sim)

np.savetxt('../data/trajectory-talos-simulation.txt', pos_sim.T)

# scipy.io.savemat('../data/talos-simulation-' + str(step_length) + '.mat', 
#                  {'ts_sim': ts_raptor, 
#                   'pos_sim': pos_sim, 
#                   'vel_sim': vel_sim, 
#                   'us_sim': us_sim, 
#                   'e_sim': e_sim, 
#                   'edot_sim': edot_sim,
#                   'step_length_opt': step_length_opt,
#                   'step_length_sim': step_length_sim})
