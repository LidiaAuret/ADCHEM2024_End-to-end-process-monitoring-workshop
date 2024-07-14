'''Methods and helper functions for modules for end-to-end process monitoring'''

# Import libraries
import numpy as np

# Disturbance module
def DisturbanceModule(d,t):
    # Inputs:
    # d: dictionary with disturbance parameters 'mu', 'sig', 'phi' for each disturbance
    # t: dictionary with time vector 'tvector'
    # Outputs:
    # d_range: dictionary with disturbance range for each disturbance

    # Pre-allocate disturbance range dictionary
    d_range = {key:d['mu'][key]*np.ones(t['N']) for key in d['mu']}
    for i in range(1, t['N']):
        for key in d_range:
            d_range[key][i] = d['phi'][key]*d_range[key][i-1] \
                    + d['sig'][key]*np.sqrt(1-d['phi'][key]**2)*np.random.randn() \
                    + (1-d['phi'][key])*d['mu'][key]
    return d_range

# Process module
# - Calculation
def ProcessModule(p, u, d_now, x, f, t):
    # ODEs
    def processODE(p, u, d_now, x_CV, x_V, x_xWv, f):
        # Edge cases
        x_xWv = max(0, min(x_xWv,1))
        
        # Intermediate variables
        x_C = x_CV/x_V
        x_L = max(p['levelThreshold'], x_V/p['A'])
        x_F0 = u['x0v']*d_now['F0']
        x_FW = x_xWv*p['cv']
        x_F = u['xFv']*p['kv']*np.sqrt(x_L)

        # Differential equations
        dVdt = x_F0 + x_FW - x_F
        dCVdt = d_now['C0']*x_F0 - x_C*x_F
        dxWvdt = (1/p['tauv'])*(u['xWv'] - x_xWv)
        
        # Check for fault: fail open water valve,
        if f['valveFW']['state']=='stuck' and u['control']:
            dxWvdt = 10 # fail open
        
        return dVdt, dCVdt, dxWvdt
    
    # Derivatives: Hard-coded Runge-Kutta 4th order
    k1V, k1CV, k1xWv = processODE(p, u, d_now, x['CV'], x['V'], x['xWv'], f)
    x1V = x['V'] + k1V*t['deltat']/2
    x1CV = x['CV'] + k1CV*t['deltat']/2
    x1xWv = max(0,min(x['xWv'] + k1xWv*t['deltat']/2,1))

    k2V, k2CV, k2xWv = processODE(p, u, d_now, x1CV, x1V, x1xWv, f)
    x2V = x['V'] + k2V*t['deltat']/2
    x2CV = x['CV'] + k2CV*t['deltat']/2
    x2xWv = max(0,min(x['xWv'] + k2xWv*t['deltat']/2,1))

    k3V, k3CV, k3xWv = processODE(p, u, d_now, x2CV, x2V, x2xWv, f)
    x3V = x['V'] + k3V*t['deltat']
    x3CV = x['CV'] + k3CV*t['deltat']
    x3xWv = max(0,min(x['xWv'] + k3xWv*t['deltat'],1))

    k4V, k4CV, k4xWv = processODE(p, u, d_now, x3CV, x3V, x3xWv, f)
    
    # Integration
    x['V'] = x['V'] + t['deltat']*(k1V + 2*k2V + 2*k3V + k4V)/6
    x['CV'] = x['CV'] + t['deltat']*(k1CV + 2*k2CV + 2*k3CV + k4CV)/6
    x['xWv'] = x['xWv'] + t['deltat']*(k1xWv + 2*k2xWv + 2*k3xWv + k4xWv)/6

    # Edge cases
    x['V'] = max(p['levelThreshold']*p['A'], x['V'])
    x['xWv'] = max(0, min(x['xWv'],1))

    # Intermediate variables
    x['C'] = x['CV']/x['V']
    x['L'] = x['V']/p['A']
    x['F0'] = u['x0v']*d_now['F0']
    x['FW'] = x['xWv']*p['cv']
    x['F'] = u['xFv']*p['kv']*np.sqrt(x['L']) 
    
    return x

# Fault module
# - Helper function: hazard rate
def hazardrate(t_runtime, dt, alpha, L):
    # failure CDF
    # F = lambda t_runtime: 16*(1-alpha)*(t_runtime/L-0.5)**5 + alpha*(t_runtime/L-0.5) + 0.5
    F = lambda t_runtime: np.exp(alpha*(t_runtime-L))
    h = ( F(t_runtime+dt) - F(t_runtime) )/( (1 - F(t_runtime)) + 1e-8 )
    return h

# - Calculation
def FaultModule(p, x, f, t):
    # Run through all fault IDs
    for fault_id in f.keys():
        # Only consider faults with a fault type that is not none
        if not f[fault_id]['fault_type']=='none':
            # Increase component running time
            f[fault_id]['runtime'] += t['deltat']
            # Calculate current hazard rate
            h = hazardrate(f[fault_id]['runtime'], t['deltat'], f[fault_id]['alpha'], f[fault_id]['L'])
            # Trigger faults according to the probability for failure
            if np.random.uniform()<h and f[fault_id]['state']=='none':
                f[fault_id]['state'] = f[fault_id]['fault_type']
            # Adjust drift for currently drifting sensors
            if f[fault_id]['state']=='drift':
                f[fault_id]['drift'] += f[fault_id]['drift_rate']*t['deltat']
    
    return f

# Measurement module
# - Calculation
def MeasurementModule(y, x, d, f):
    # Add random normal noise to true values for each of the measurements
    for key in y.keys():
        y[key]['value'] = y[key]['function'](x, d) + np.random.normal(0, y[key]['noiseVar'])
        # Check for fault conditions
        if f[key]['state']=="stuck":
            y[key]['value'] = y[key]['value']
        elif f[key]['state']=="bias":
            y[key]['value'] = y[key]['value'] + f[key]['bias']
        elif f[key]['state']=="drift":
            y[key]['value'] = y[key]['value'] + f[key]['drift']
    return y

# Regulatory control module
# - Calculation
def RegulatoryControlModule(u, y, r):
    # No regulatory control for F0 and F valves
    u['x0v'] = r['components']['valveF0']['position']
    u['xFv'] = r['components']['valveF']['position']

    # PI controller for FW valve
    # Check if controller mode is in automatic
    if r['components']['valveFW']['position'] == -1:
        # Calculate error
        error = r['setpoints']['C'] - y['C']['value']
        # Accumulate integral error
        u['intError'] = u['intError'] + error*u['dt']
        # Calculate controller output: output bias of zero?
        u['xWv'] = -u['PI']['K']*(error + u['intError']/u['PI']['tauI']) # TOASK: why no bias?
        # Note controller mode
        u['control'] = True
    else:
        # Manual control
        u['xWv'] = 0.5*r['components']['valveFW']['position'] # TOASK: why 0.5?
        u['intError'] = 0
        u['control'] = False

    return u

# Supervisory control module
# - Calculation
def SupervisoryControlModule(r, m, y, t):
    time_since_startup = t['tvector'][t['i']] - r['Startup']['time']
    # Check if system is in a shut
    if r['regime']=='Shut':
        # Set valve positions for shut regime
        r['components']['valveFW']['position'] = 0
        r['components']['valveF0']['position'] = 0
        r['components']['valveF']['position'] = 0

        # Set setpoints for shut regime
        r['setpoints']['C'] = np.nan

        # Special actions
        # none

        # Switch to next regime
        r['regime'] = 'Startup'
        # Save startup time (current time for current time index)
        r['Startup']['time'] = t['tvector'][t['i']]
        # Update planned shutdown time
        if not r['ShutType']=='Unplanned':
            r['NextPlannedShut'] = t['tvector'][t['i']] + r['PlannedMaintenancePeriod']
    
    elif r['regime']=='Startup':
        # Set valve positions for startup regime
        r['components']['valveFW']['position'] = 1
        r['components']['valveF0']['position'] = 0
        r['components']['valveF']['position'] = 0

        # Set setpoints for startup regime
        r['setpoints']['C'] = np.nan

        # Special actions
        # none

        # Switch to next regime
        if y['L']['value']>r['Startup']['levelThreshold']:
            r['regime'] = 'Running'
    
    elif r['regime']=='Running':
        # Set valve positions for running regime
        r['components']['valveFW']['position'] = -1 # automatic control
        r['components']['valveF0']['position'] = 1
        r['components']['valveF']['position'] = 1

        # Set setpoints for running regime
        r['setpoints']['C'] = r['Running']['Csp']

        # Special actions
        # none

        # Switch to next regime
        # Level interlock for too high level: plant trip, flag all components as faulty
        if y['L']['value']>r['Running']['levelInterlock']:
            r['regime'] = 'Shutdown'
            r['ShutType'] = 'Unplanned'
            # Mark all components as faulty
            for key in r['components']['fields']:
                r['components'][key]['faultFlag'] = True
        # Planned maintenance
        elif t['tvector'][t['i']] > r['NextPlannedShut']: 
            r['regime'] = 'Shutdown'
            r['ShutType'] = r['MaintenanceCycle'][r['PlannedShuts'] % len(r['MaintenanceCycle'])]
            r['PlannedShuts'] += 1
        # Check for component alarms (after a warm-up period of an hour)
        # Case 1: monitoring outputs ignored
        # Case 2: monitoring outputs considered during next planned maintenance
        # Case 3: monitoring outputs can trigger unplanned maintenance
        elif time_since_startup > 3600 and not r['Case']==1: 
            for key in r['components']['fields']:
                if m['components'][key]['alarm'][t['i']-1]==1: # If any component sounds and alarm
                    # Mark specific component as faulty
                    r['components'][key]['faultFlag'] = True
                    if r['Case']==3:    
                        # Switch to shutdown regime, unplanned maintenance
                        r['regime'] = 'Shutdown'    
                        r['ShutType'] = 'Unplanned' 

                        if time_since_startup < 7200:
                            for idx in r['components']['fields']:
                                r['components'][idx]['faultFlag'] = True 

    elif r['regime']=='Shutdown':
        # Set valve positions for shutdown regime
        r['components']['valveFW']['position'] = 0
        r['components']['valveF0']['position'] = 0
        r['components']['valveF']['position'] = 1

        # Set setpoints for shutdown regime
        r['setpoints']['C'] = np.nan

        # Special actions
        # none

        # Switch to next regime
        if y['L']['value']<r['Shutdown']['levelThreshold']:
            r['regime'] = 'Shut'

    else:
        raise ValueError('Unknown regime: {}'.format(r['regime']))
    
    # Update regime numeric value
    r['regimeNumeric'] = r['regimeNumericMap'][r['regime']]

    return r

# Maintenance module
# - Calculation
def MaintenanceModule(r, f, t):
    # Set duration of shutdown
    r["ShutDownTime"] = r["MinimumShutDownTime"]

    # Cycle over all components to identify components requiring replacement
    # and to determine total shutdown time
    for key in r["components"]["fields"]:
        # Check flagged components OR check for component-type planned maintenance OR check for maintaining all
        if r["components"][key]["faultFlag"] or r["components"][key]["type"]==r["ShutType"] or r["ShutType"]=="All": 
            # Add the time taken to check the component to the shutdown time
            r["ShutDownTime"] += r["components"][key]["check_duration"]

            # Replace faulty components
            if f[key]["state"]!="none":
                f[key]["state"] = "none" # reset fault state
                # Reset drift, if applicable
                if f[key]["fault_type"]=="drift":
                    f[key]["drift"] = 0
                f[key]["runtime"] = 0 # reset runtime
                r["components"][key]["faultFlag"] = False # remove fault flag
                # Add the time taken to replace the component to the shutdown time
                r["ShutDownTime"] += r["components"][key]["replace_duration"]
    
    # Move time forward for the duration of the shutdown
    # (proportional to the number of inspections)
    current_time = t['tvector'][t["i"]]
    max_time = t["tmax"]
    current_time += r["ShutDownTime"] # move time forward
    current_time = min(current_time, max_time-t['deltat']) # do not exceed simulation time
    t['tvector'][t['i']+1] = current_time

    return r, f, t

# Monitoring module
# - Calculation
def MonitoringModule(m, y, r, t):
    # Training of monitoring model happens after initial transient, 
    # and after specified training time
    # Training data collection occurs before training end time
    if m['training'] and t['tvector'][t['i']]<m['trainingEndTime']:
        m = collectTrainingData(m, y, t)

    # Training occurs once during the entire simulation
    elif m['training'] and t['tvector'][t['i']]>=m['trainingEndTime']:
        # Remove nans from training data
        # Find rows with any NaNs
        rows_with_nans = np.any(np.isnan(m['Xtrain']), axis=1)
        # Find rows without any NaNs
        rows_without_nans = ~rows_with_nans
        # Select rows that do not contain any NaNs
        m['Xtrain'] = m['Xtrain'][rows_without_nans]

        # Center and scale the data
        m['model']['mX'] = np.mean(m['Xtrain'], axis=0)
        m['model']['sX'] = np.std(m['Xtrain'], axis=0)
        m['Xtrain'] = (m['Xtrain'] - m['model']['mX'])/m['model']['sX']

        # Calculate covariance matrix
        C = np.cov(m['Xtrain'], rowvar=False)

        # Calculate eigenvalues and eigenvectors
        eigval, eigvec = np.linalg.eig(C)

        # Sort eigenvalues and eigenvectors in descending order
        idx = eigval.argsort()[::-1]
        eigval = eigval[idx]
        eigvec = eigvec[:,idx]

        # Retain principal components
        m['model']['PC'] = eigvec[:,:m['hyperparameters']['nComponents']]

        # Retain variances
        m['model']['L'] = eigval[:m['hyperparameters']['nComponents']]
        m['model']['invL'] = np.diag(1/m['model']['L'])

        # Calculate T, Hotelling T2 and SPE for training data
        T = np.dot(m['Xtrain'], m['model']['PC'])
        T2 = np.sum(T @ m['model']['invL'] * T, axis=1) 
        SPE = np.sum((m['Xtrain'] - np.dot(T, m['model']['PC'].T))**2, axis=1)

        # Add calculated statistics to pre-allocated matrices
        # Convert Xtrain non-nan indices to time indices, populate pre-allocated matrices
        indices_to_insert = np.where(~rows_with_nans)[0] + int(m['trainingStartTime']/t['deltat'])
        m['statistic']['T'][indices_to_insert,:] = T
        m['statistic']['T2'][indices_to_insert] = T2.reshape(-1,1)
        m['statistic']['SPE'][indices_to_insert] = SPE.reshape(-1,1)
        
        # Set current warnings and alarms
        for field in m['components']['fields']:
            m['components'][field]['warning'][0:t['i']] = 0
            m['components'][field]['alarm'][0:t['i']] = 0
        
        # Complete training phase
        m['training'] = False

    # Monitoring phase: during running regime
    elif not m['training'] and r['regime']=='Running':
        # Set alarms/warnings to zero for all components, initially
        for field in m['components']['fields']:
            m['components'][field]['warning'][t['i']] = 0
            m['components'][field]['alarm'][t['i']] = 0

        # Apply monitoring model
        if m['monitoringActive']:
            # Collect data (only one sample for the available measurements)
            X = np.array([y[key]['value'] for key in y.keys()]).reshape(1,-1)

            # Center and scale data
            X = (X - m['model']['mX'])/m['model']['sX']

            # Calculate T, Hotelling T2 and SPE
            T = np.dot(X, m['model']['PC'])
            T2 = np.sum(T @ m['model']['invL'] * T, axis=1)
            SPE = np.sum((X - np.dot(T, m['model']['PC'].T))**2, axis=1)

            # Add calculated statistics to pre-allocated matrices
            m['statistic']['T'][t['i'],:] = T
            m['statistic']['T2'][t['i']] = T2
            m['statistic']['SPE'][t['i']] = SPE

            # Fault diagnosis:
            # Check if abnormality is present
            if T2>m['hyperparameters']['T2threshold'] or SPE>m['hyperparameters']['SPEthreshold']:
                # Check composition sensor
                if abs(y['C']['value'] - m['model']['mX'][1]) < 4*m['model']['sX'][1]:
                    # If there is an abnormal sample, 
                    # but the sensor reading is close to NOC:
                    # Assume sensor is faulty
                    m['components']['C']['warning'][t['i']] = 1
                else: 
                    # Otherwise, assume valve is faulty
                    m['components']['valveFW']['warning'][t['i']] = 1

            # Check for fraction of warnings within a window to raise an alarm
            alarm_w = 36 # monitoring window size
            alarm_f = 0.8 # fraction of warnings in window to raise an alarm

            # Check for composition warnings
            if np.sum(m['components']['C']['warning'][t['i']-alarm_w:t['i']])>alarm_f*alarm_w:
                m['components']['C']['alarm'][t['i']] = 1

            # Check for valve warnings
            if np.sum(m['components']['valveFW']['warning'][t['i']-alarm_w:t['i']])>alarm_f*alarm_w:
                m['components']['valveFW']['alarm'][t['i']] = 1

        # Switch monitoring to active again if composition is close to NOC value
        elif y['C']['value'] > (m['model']['mX'][0] - 2*m['model']['sX'][0]): 
            m['monitoringActive'] = True
            
    elif r['regime']=='Startup':
        # Provide time for the system to stabilize (C to get to SP) before monitoring
        m['monitoringActive'] = False 

    return m

# - Helper function: collect training data
def collectTrainingData(m, y, t):
    # Collect training data (after transient, before training time end <-- last condition check outside of function)
    if t['tvector'][t['i']]>=m['trainingStartTime']:
        # Collect training data from measurements, in numpy array format
        new_data = np.array([y[key]['value'] for key in y.keys()]).reshape(1,-1)
        m['Xtrain'][t['i'],:] = new_data

    return m

# Economic module
# - Calculation
def EconomicModule(econ, x, r, t):
    if r['regime']=='Shut':
        # Costs during maintenance 
        if r['ShutType']=='Unplanned':
            # Unplanned maintenance is more expensive
            econ['KPI']['values'][t['i']] = -250*r['ShutDownTime']/3600
        else:
            econ['KPI']['values'][t['i']] = -100*r['ShutDownTime']/3600 
    else:
        # Profit during operation
        econ['KPI']['values'][t['i']] = econ['KPI']['function'](r, x)
        
    return econ