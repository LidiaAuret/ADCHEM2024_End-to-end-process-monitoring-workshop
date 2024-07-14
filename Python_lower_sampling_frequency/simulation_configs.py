'''Methods and helper functions for configurations for end-to-end process monitoring'''

# Import libraries
import numpy as np

# Generate all configurations
def generateConfigurations(case_study='Blending tank',maintenance_strategy=1):
    if case_study=='Blending tank':
        # Simulation time 
        ndays = 28
         # number of days to simulate
        deltat = 100 # in seconds
        tmax = ndays*24*3600 # simulation time in seconds
        tvector = np.arange(0, tmax+deltat, deltat) # time vector
        N = len(tvector)-1
        t = {
            'ndays': ndays,
            'deltat': deltat,
            'tmax': tmax,
            'N': N,
            'tvector': tvector,
            'i':0,
        }
        # Disturbance parameters
        d = {
            'mu': {
                'F0': 0.005, # mean flow rate, m3/s
                'C0': 1.0 # mean concentration, kg/m3
            },
            'sig': {
                'F0': 0.001, # standard deviation flow rate, m3/s
                'C0': 0.1 # standard deviation concentration, kg/m3
            },
            'phi': {
                'F0': 0.9, # correlation flow rate
                'C0': 0.99 # correlation concentration
            }
        }
        # Inputs u and controller parameters
        u = {
            'x0v': 0.5, # initial valve position
            'xFv': 0.5, # final valve position
            'xWv': 0.5, # water valve position
            'PI': {
                'K': 0.1, # controller gain, m3/kg
                'tauI':10, # controller time constant
               },
           'intError':0,
           'control':True,
           'dt':deltat
        }
        # Process parameters
        p = {
            'A':4, # cross-sectional area of the vessel, m2
            'cv':0.025, # control valve (feed water) coefficient, m3/s
            'kv':0.02, # drainage valve coefficient, m2.5/s 
            'tauv':60, # control valve (feed water) time constant, s
            'dt':deltat, # time step, s
            'levelThreshold': 0.0005 # minimum allowable level, m
        }
        # Process initial conditions
        x = {
            'V':0.001*p['A'],   # fluid volume of the vessel, m3
            'CV':0,  # solute amount in the vessel, kg
            'xWv':0,  # actual valve fraction opening feed water
        }
        x.update({
            'C':x['CV']/x['V'],
            'L':x['V']/p['A'],
            'F0':u['x0v']*d['mu']['F0'],
            'FW':x['xWv']*p['cv'],
            'F':u['xFv']*p['kv']*np.sqrt(x['V']/p['A']),
        })
        # Fault parameters
        f = { # Sensor faults
            'C': {
                'alpha':1e-5, # minimum probability for failure. Used 0.2 for bathtub curve
                'L':15*24*3600, # maximum lifetime
                'fault_type':'drift',
                'drift': 0,
                'drift_rate': 0.05/(24*3600), 
                'state':'none',
                'runtime': 0
            },
            'C0': {
                'fault_type':'none',
                'state':'none',
            },
            'F0': {
                'fault_type':'none',
                'state':'none',
            },
            'FW': {
                'fault_type':'none',
                'state':'none',
            },
            'F': {
                'fault_type':'none',
                'state':'none',
            },
            'L': {
                'fault_type':'none',
                'state':'none',
            },
            # Valve faults
            'valveFW': {
                'alpha':1e-5, # minimum probability for failure. Used 0.3 for bathtub curve
                'L':13.4*24*3600,
                'fault_type':'stuck',
                'stuck': 0,
                'state':'none',
                'runtime': 0
            },
            'valveF0': {
                'fault_type':'none',
                'state':'none',
            },
            'valveF': {
                'fault_type':'none',
                'state':'none',
            },

        }
        # Measurement parameters
        y = {
            'C': { # concentration in the tank
                'function': lambda x, d: x['C'],
                'noiseVar': 0.01,
                'value': 0
                },
            'C0': { # inlet concentration
                'function': lambda x, d: d['C0'],
                'noiseVar': 0.01,
                'value': 0
                },
            'F0': { # inlet flow rate
                'function': lambda x, d: x['F0'],
                'noiseVar': 0.002,
                'value': 0
                },
            'FW': { # water flow rate
                'function': lambda x, d: x['FW'],
                'noiseVar': 0.002,
                'value': 0
                },
            'F': { # outlet flow rate
                'function': lambda x, d: x['F'],
                'noiseVar': 0.002,
                'value': 0
                },
            'L': { # liquid level
                'function': lambda x, d: x['L'],
                'noiseVar': 0.002,
                'value': 0
                },
            }
        # Supervisory control parameters
        r = {
            'components': {
                'fields': ['valveF0', 'valveFW', 'valveF', # valves
                           'C', 'C0', 'F0', 'FW', 'F', 'L' ], # sensors
            }
        }
        # Create components
        for component in r['components']['fields']:
            r['components'][component] = {}
        # Add valve positions
        r['components'].update({
            'valveF0': {
                    'position': u['x0v'], # initial position
                },
            'valveFW': {
                    'position': -1, # automatic control indication
                },
            'valveF': {
                    'position': u['xFv'], # initial position
                },
            })
        # Add setpoints
        r.update({
            'setpoints': {
                'C': 0.5, # setpoint for concentration, kg/m3
            }})
        # Different cases for maintenance intervention
        # 1 - no monitoring intervention, completely ignore alarms, don't even flag components if alarm sounds
        # 2 - no unplanned maintenance: only replace flagged components at next planned maintenance
        # 3 - unplanned maintenance: shut down plant and replace flagged components immediately
        # 4 - switch control regime and wait for planned maintenance (TODO: implement)
        r.update({
            'Case': maintenance_strategy,
        })

        # Further supervisory control parameters
        # Update component type, fault flag, component check duration, and component replace duration
        for component in r['components']['fields']:
            if component in ['valveF0', 'valveFW', 'valveF']:
                r['components'][component]['type'] = 'valve'
                r['components'][component]['faultFlag'] = False
                r['components'][component]['check_duration'] = 2*3600 # 2 hours
                r['components'][component]['replace_duration'] = 4*3600 # 4 hours
            else:
                r['components'][component]['type'] = 'sensor'
                r['components'][component]['faultFlag'] = False
                r['components'][component]['check_duration'] = 2*3600 # 2 hours
                r['components'][component]['replace_duration'] = 4*3600 # 4 hours
        # Maintenance parameters
        if r['Case'] == 1:
            maintenance_cycle = ['All']
        else:
            maintenance_cycle = ['valve', 'sensor']
        r.update({
            'MinimumShutDownTime': 0*24*3600, # 0 days
            'PlannedMaintenancePeriod': 1*7*24*3600, # time before planned maintenance, every four weeks
            'NextPlannedShut': 1*7*24*3600, # time before next planned maintenance
            'MaintenanceCycle': maintenance_cycle, # maintenance cycle
            'PlannedShuts': 0, # number of planned maintenance shutdowns that have occured
        })
        # Regime parameters
        r.update({
            'Shutdown': {
                'levelThreshold': 2*p['levelThreshold'], # level at which to switch from shutdown to shut
            },
            'Startup': {
                'levelThreshold': 1, # level at which to switch from startup to running
                'time': [0], # list of startup times
            },
            'Running': {
                'Csp': 0.3, # setpoint for concentration, kg/m3
                'levelInterlock': 3, # level that trips process
            },
            'regime': 'Startup',
            'ShutType': 'Initial Startup' # for initialisation
        })
        # Keep track of regimes with numeric values
        # Regime: Startup, Running, Shutdown, Shut
        r.update({
            'regimeNumericMap': {
                'Startup': 2,
                'Running': 3,
                'Shutdown': 4,
                'Shut': 1,
            }
        })
        # Indicate current numeric value of regime
        r.update({
            'regimeNumeric': r['regimeNumericMap'][r['regime']]
        })
        # Monitoring parameters
        m = {
            'monitoringActive': False, # monitoring activity flag
            'yFields': {'C', 'C0', 'F0', 'FW', 'F', 'L'}, # measurements used during monitoring
            'components': {
                'fields': r['components']['fields'] # components with associated alarms or warnings
            },
            'hyperparameters': {
                'nComponents': 2, # number of principal components to retain
                'T2threshold': 30, # threshold for T2 statistic
                'SPEthreshold': 20, # threshold for SPE statistic
            }
        }
        # Current alarms are warnings on any component, pre-allocated for speed
        for field in m['components']['fields']:
            m['components'].update({
                field: {
                'warning': np.full(t['N'],np.nan),
                'alarm': np.full(t['N'],np.nan),
            }})
        # Monitoring model training time
        m.update({
            'training': True, 
            'trainingStartTime': 2*3600, # start time for training (to exlude initial transient)
            'trainingEndTime': 5*24*3600, # 5 days
            'Xtrain': np.full((5*24*3600,len(m['yFields'])),np.nan), # training data
        })
        # Model parameters
        m.update({
            'model': {
                'mX': [], # means of training data
                'sX': [], # standard deviations of training data
                'PC': [], # principal components (retained)
                'L': [], # variance explained by principal components
                'invL': [], # inverse of L for quick calculation
            }
        })
        # Pre-allocate monitoring statistics
        m.update({
            'statistic': {
                'T': np.full((t['N'], m['hyperparameters']['nComponents']),np.nan), # Scores
                'T2': np.full((t['N'], 1),np.nan),
                'SPE': np.full((t['N'], 1),np.nan),
            }
        })

        # Economic parameters
        # - KPI function
        def KPIfunction(r, x):
            # Check for NaN value in set point
            if np.isnan(r['setpoints']['C']):
                KPI = np.nan
            else:
                # Key performance indicator
                KPI = np.exp(-40*(x['C'] - r['setpoints']['C'])**2)
            return KPI
        # - Economic parameters
        econ = {
            'KPI': {
                'values': np.full((t['N'], 1),np.nan),
                'function': KPIfunction,
            }
        }
    
    return t, d, u, p, x, f, y, r, m, econ

