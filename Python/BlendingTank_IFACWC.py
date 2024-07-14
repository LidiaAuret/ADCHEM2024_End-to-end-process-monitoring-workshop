'''End-to-end process monitorng demonstrated on blending tank example'''

# Import necessary libraries
import numpy as np
import matplotlib.pyplot as plt
from simulation_configs import generateConfigurations
from simulation_modules import DisturbanceModule, ProcessModule, FaultModule, MeasurementModule, RegulatoryControlModule, SupervisoryControlModule, MaintenanceModule, MonitoringModule, EconomicModule
from simulation_helpers import generateOverviewList, collectOverviewData, plotOverviewData, plotEconomicData
import pandas as pd
import time as real_time

# Repeatable results
np.random.seed(100)

# Generate configurations
t, d, u, p, x, f, y, r, m, econ = generateConfigurations(maintenance_strategy=3)

# Generate disturbances
d_range = DisturbanceModule(d,t)

# Simulation
# Print case
print(f'Maintenance strategy: Case {r["Case"]}')
# Start timer for simulation duration
start_time = real_time.time()
# List of variables to save
results_list = generateOverviewList()
# Preallocate numpy array to store results
results = np.full((t['N'], len(results_list)),np.nan)
# Start simulation
print('Simulation progress:')
while t['i'] < t['N']: # necessary while loop, since shuts can skip forward in time
    # Current disturbance
    d_now = {col: d_range[col][t['i']] for col in ['F0', 'C0']}
    # Supervisory control module
    r = SupervisoryControlModule(r, m, y, t)
    # Maintenance module
    if r['regime']=='Shut':
        r, f, t = MaintenanceModule(r, f, t) # t['i'] updated during maintenance
    # Regulatory control module
    u = RegulatoryControlModule(u, y, r)
    # Process module
    x = ProcessModule(p, u, d_now, x, f)
    # Fault module
    f = FaultModule(p, x, f, t)
    # Measurement module
    y = MeasurementModule(y, x, d_now, f)
    # Monitoring module
    m = MonitoringModule(m, y, r, t)
    # Economic module
    econ = EconomicModule(econ, x, r, t)
    # Update results
    results = collectOverviewData(x, y, r, m, f, t, results)
    # Display progress in percent time
    if t['i'] % (2*24*3600) == 0:
        print(f'{t["tvector"][t["i"]]/t["tmax"]*100:.2f}%')
    # Update time index
    t['i'] += 1

# End timer for simulation duration
end_time = real_time.time()
# Print simulation duration
print(f'Done! Simulation duration: {end_time-start_time:.1f} seconds.')

# Convert results (numpy array) to DataFrame
df = pd.DataFrame(results, columns=results_list)
df.set_index('Time (days)', inplace=True)

# Plotting
fig, ax = plotOverviewData(df)
ax[0].set_title(f'Overview of results, maintenance strategy: Case {r['Case']}')

fig, ax = plotEconomicData(t, econ)
ax.set_title(f'Economic performance, maintenance strategy: Case {r['Case']}')







