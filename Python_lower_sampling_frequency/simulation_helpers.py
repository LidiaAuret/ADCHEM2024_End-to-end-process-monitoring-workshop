'''Helper functions for end-to-end process monitoring'''
# Import libraries
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# Data list: Overview data
def generateOverviewList():
    results_list = ['Time (days)',
                'C', 'Cm', 'Csp',
                'Sensor C fault', 'Sensor C alarm',
                'L', 'xFW',
                'Valve FW fault', 'Valve FW alarm',
                'ShutType', 'Regime', 'Profit'
                ]
    return results_list

# Data collection: Overview data
def collectOverviewData(x, y, r, m, f, econ, t, results):
    # Overview data:
    # Time
    # C, Cm, Csp
    # sensorC alarm and fault
    # L, xFW
    # valveFW alarm and fault
    # shut type and maintenance active
    # profit
    
    # Calculations
    sensorC_fault_state = 0 if f['C']['state']=='none' else 1
    valveFW_fault_state = 0 if f['valveFW']['state']=='none' else 1
    shutTypeMap = {'Planned':0, 'Unplanned':1, 'All':2, 'valve':3, 'sensor':4, 'Initial Startup':5}
    
    # Results
    results[t['i'],:] = [
        t['tvector'][t['i']]/(3600*24),
        x['C'],
        y['C']['value'],
        r['setpoints']['C'],
        sensorC_fault_state,
        m['components']['C']['alarm'][t['i']],
        x['L'],
        x['xWv'],
        valveFW_fault_state,
        m['components']['valveFW']['alarm'][t['i']],
        shutTypeMap[r['ShutType']],
        r['regimeNumeric'],
        econ['KPI']['values'][t['i']][0]
    ]

    return results

# Plotting: Overview data
def plotOverviewData(df):
    # Downsampling specification
    n = 10
    
    # Set up figure and axes
    fig, ax = plt.subplots(3, 1, figsize=(10, 10), sharex=True, gridspec_kw={'height_ratios': [1.5, 1, 1], 'hspace': 0})
    
    # Concentration plot
    df['Cm'].iloc[::n].plot(ls='none', color='blue', marker='o', markersize=4, alpha=0.1, ax=ax[0], label='Cm')
    df['C'].iloc[::n].plot(ls='none', color='red', marker='^', markersize=4, alpha=0.1, ax=ax[0], label='C')
    df['Csp'].iloc[::n].plot(ls='--', color='k', ax=ax[0], label='Csp')
    # Limit yrange
    ax[0].set_ylim([0, 0.9])
    # Dividing line before fault and alarm states
    ax[0].axhline(0, color='k', lw=0.5)
    lower_ylimit = ax[0].get_ylim()[0]
    y_range = ax[0].get_ylim()[1] - ax[0].get_ylim()[0]
    #fault_y = lower_ylimit + 0.1 * y_range
    #alarm_y = lower_ylimit + 0.15 * y_range
    fault_y = 0.8
    alarm_y = 0.9
    # Fault
    df.loc[df['Sensor C fault'] == 0, 'Sensor C fault'] = np.nan
    ax[0].plot(df['Sensor C fault']*fault_y, marker='.',color='darkblue', ls='none', alpha=0.5, label = 'Cm fault')
    # Alarm
    df.loc[df['Sensor C alarm'] == 0, 'Sensor C alarm'] = np.nan
    ax[0].plot(df['Sensor C alarm']*alarm_y, marker='o',color='blue', ls='none', alpha=0.5, label = 'Cm alarm')
    ax[0].set_ylabel("Concentration")
    
    # Valve position plot
    df['xFW'].iloc[::n].plot(color='orangered', ax=ax[1], label='xFW')
    # Dividing line before fault and alarm states
    ax[1].axhline(0, color='k', lw=0.5).lower_ylimit = ax[0].get_ylim()[0]
    # Limit yrange
    ax[1].set_ylim([0, 1.2])
    #fault_y = lower_ylimit + 0.1 * y_range
    #alarm_y = lower_ylimit + 0.2 * y_range
    fault_y = 1.1
    alarm_y = 1.2
    # Fault
    df.loc[df['Valve FW fault'] == 0, 'Valve FW fault'] = np.nan
    ax[1].plot(df['Valve FW fault']*fault_y, color='black', marker='.',ls='none', alpha=0.5, label = 'xFW fault')
    # Alarm
    df.loc[df['Valve FW alarm'] == 0, 'Valve FW alarm'] = np.nan
    ax[1].plot(df['Valve FW alarm']*alarm_y, color='gray', ls='none', marker='o',alpha=0.5,label = 'xFW alarm')
    ax[1].yaxis.tick_right()
    ax[1].yaxis.set_label_position("right")
    ax[1].set_ylabel("Valve position")

    # Level plot
    df['L'].iloc[::n].plot(color='seagreen', ax=ax[2], label='L')
    ax[2].set_ylabel("Level")

    # Maintenance periods and types of maintenance
    # Find non-running period start and end points
    regime_not_3 = df['Regime'] != 3
    starts, ends = find_start_end_points(df, regime_not_3)
    # Adding maintenance type and duration
    shutTypeMap = ['P', 'U', 'A', 'V', 'S', 'I']
    for start, end in zip(starts, ends):
        shutAbbrev = shutTypeMap[int(df.loc[start, 'ShutType'])]
        y_pos = ax[0].get_ylim()[1] - 0.05 * (ax[0].get_ylim()[1] - ax[0].get_ylim()[0])
        ax[0].text(start, y_pos, shutAbbrev, verticalalignment='top', fontsize=14)
        for ax_c in ax:
            # Maintenance shading
            ax_c.axvspan(start, end, color='grey', alpha=0.3)

    # Generate legend
    handles = []
    labels = [] 
    for axc in ax:
        handlesc, labelsc = axc.get_legend_handles_labels()
        handles.extend(handlesc)
        labels.extend(labelsc)
    ax[2].legend(handles,labels, loc='lower right', ncol=3)

    return fig, ax

# Economic data
def plotEconomicData(df):
    # Plat cumulative profit
    cumulative_profit = np.nancumsum(df['Profit'])
    fig, ax = plt.subplots(1, 1, figsize=(10, 5))
    ax.plot(df.index, cumulative_profit, label=f'Cumulative profit\n(final value: {cumulative_profit[-1]:0.0f})')
    ax.set_xlabel('Time (days)')
    ax.set_ylabel('Cumulative profit')
    ax.legend()

    return fig, ax

# Helper functions
def find_start_end_points(df, condition, fill_gaps=False):
    """
    Find start and end points based on a given condition.
    
    Parameters:
    - df: DataFrame containing the data.
    - condition: A boolean Series indicating the condition.
    
    Returns:
    - starts: Index of start points where the condition changes from False to True.
    - ends: Index of end points where the condition changes from True to False.
    """
    if fill_gaps:
        condition = condition.fillna(False)
    changes = condition.astype(int).diff()
    starts = changes[changes > 0].index[:-1]
    ends = changes[changes < 0].index[:-1]

    # Handle case where the series starts or ends with the condition being True
    if condition.iloc[0]:
        starts = pd.Index([df.index[0]]).append(starts)
    if condition.iloc[-1]:
        ends = ends.append(pd.Index([df.index[-1]]))

    # Ensure starts and ends have the same length by adding a dummy end if necessary
    if len(starts) > len(ends):
        ends = ends.append(pd.Index([df.index[-1]]))

    return starts, ends

