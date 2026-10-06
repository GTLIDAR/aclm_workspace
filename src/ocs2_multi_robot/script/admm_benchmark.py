#!/usr/bin/env python3
"""
Script to plot ADMM solver performance metrics from solver output.
Parses primal residual data and plots it against ADMM iterations.
"""

import re
import matplotlib.pyplot as plt
import numpy as np
from typing import List, Tuple, Optional


def parse_admm_output(text: str) -> Tuple[List[int], List[float]]:
    """
    Parse ADMM solver output and extract iteration numbers and primal residuals.
    
    Args:
        text: Raw text output from ADMM solver
        
    Returns:
        Tuple of (iterations, primal_residuals) as lists
    """
    iterations = []
    primal_residuals = []
    
    # Pattern to match the iteration line:
    # "        0 |     2.139589e+00 | 3.111286e+00 | 2.052487e+02 | [1.1994e+05, 9.4890e+04]"
    pattern = r'^\s+(\d+)\s+\|\s+([\d\.e\+\-]+)\s+\|'
    
    for line in text.split('\n'):
        match = re.match(pattern, line)
        if match:
            iteration = int(match.group(1))
            primal_residual = float(match.group(2))
            iterations.append(iteration)
            primal_residuals.append(primal_residual)
    
    return iterations, primal_residuals


def plot_primal_residual(iterations: List[int], 
                         primal_residuals: List[float],
                         label: Optional[str] = None,
                         ax: Optional[plt.Axes] = None,
                         **kwargs) -> plt.Axes:
    """
    Plot primal residual vs ADMM iterations.
    
    Args:
        iterations: List of iteration numbers
        primal_residuals: List of primal residual values
        label: Label for the plot line
        ax: Optional axes to plot on (for subplots)
        **kwargs: Additional arguments to pass to plt.plot (e.g., marker, color, linestyle)
        
    Returns:
        Axes object
    """
    if ax is None:
        fig, ax = plt.subplots(figsize=(10, 6))
    
    # Set default marker if not provided in kwargs
    plot_kwargs = {'marker': 'o', 'linewidth': 2, 'markersize': 6}
    plot_kwargs.update(kwargs)  # Override with any provided kwargs
    
    ax.semilogy(iterations, primal_residuals, label=label, **plot_kwargs)
    ax.set_xlabel('ADMM Iteration', fontsize=12)
    ax.set_ylabel('Primal Residual', fontsize=12)
    ax.set_title('ADMM Primal Residual Convergence', fontsize=14)
    ax.grid(True, alpha=0.3)
    if label is not None:
        ax.legend()
    
    return ax


# Example usage with the provided data
if __name__ == "__main__":
    # Standard version data
    standard_output = """
========================================
ADMM Solver Status Summary
========================================
Current rho per robot: [1.0000e+00, 1.0000e+00]
----------------------------------------
Iteration | Primal Residual | Dual Residual | Cargo Cost | Robot Costs
----------|------------------|--------------|------------|-------------|
        0 |     2.139589e+00 | 3.111286e+00 | 2.052487e+02 | [1.1994e+05, 9.4890e+04]
        1 |     3.055632e+00 | 4.249705e+00 | 1.740554e+04 | [6.9995e+05, 7.0290e+05]
        2 |     1.496432e+00 | 3.823530e+00 | 5.633857e+03 | [3.4951e+06, 3.4950e+06]
        3 |     8.790696e-01 | 2.311659e+00 | 5.746122e+01 | [4.0590e+06, 4.0590e+06]
        4 |     5.966307e-01 | 1.449210e+00 | 8.579200e+00 | [3.5536e+06, 3.5536e+06]
        5 |     4.007161e-01 | 9.460079e-01 | 4.387754e+01 | [3.2193e+06, 3.2192e+06]
        6 |     5.303915e-01 | 7.264976e-01 | 7.997409e+01 | [3.0381e+06, 3.0381e+06]
        7 |     4.487209e-01 | 5.021150e-01 | 5.668842e+00 | [2.5315e+06, 2.5315e+06]
        8 |     3.697803e-01 | 4.556165e-01 | 9.771070e+00 | [2.6122e+06, 2.6122e+06]
        9 |     3.506250e-01 | 3.254431e-01 | 6.115579e+00 | [2.5330e+06, 2.5330e+06]
       10 |     4.744066e-01 | 5.889621e-01 | 1.214962e+02 | [2.4396e+06, 2.4396e+06]
       11 |     7.060486e-01 | 1.123098e+00 | 3.614893e+03 | [2.4375e+06, 2.4372e+06]
       12 |     7.479395e-02 | 8.755305e-01 | 5.772292e+00 | [2.4099e+06, 2.4099e+06]
       13 |     5.165964e-02 | 1.646854e-01 | 5.633852e+00 | [2.3912e+06, 2.3912e+06]
       14 |     4.162207e-02 | 1.392890e-01 | 5.647340e+00 | [2.3790e+06, 2.3790e+06]
       15 |     3.887487e-02 | 1.224467e-01 | 5.656321e+00 | [2.3690e+06, 2.3690e+06]
       16 |     3.451268e-02 | 1.065058e-01 | 5.665536e+00 | [2.3568e+06, 2.3568e+06]
       17 |     3.273642e-02 | 9.293296e-02 | 5.670355e+00 | [2.3422e+06, 2.3422e+06]
       18 |     3.092081e-02 | 8.120191e-02 | 5.672537e+00 | [2.3274e+06, 2.3274e+06]
       19 |     2.980614e-02 | 7.068061e-02 | 5.672454e+00 | [2.3142e+06, 2.3142e+06]

"""
    
    # Consensus ADMM version data (placeholder - replace with your actual data)
    consensus_output = """
========================================
ADMM Solver Status Summary
========================================
Current rho per robot: [1.0000e+00, 1.0000e+00]
----------------------------------------
Iteration | Primal Residual | Dual Residual | Cargo Cost | Robot Costs
----------|------------------|--------------|------------|-------------|
        0 |     1.898354e+01 | 3.111286e+00 | 2.052487e+02 | [1.3144e+05, 1.3474e+05]
        1 |     6.389156e+00 | 5.229023e+00 | 3.369191e+04 | [9.8443e+05, 9.8523e+05]
        2 |     2.047802e+01 | 5.445950e+00 | 2.468049e+04 | [2.6791e+06, 2.6916e+06]
        3 |     1.270304e+01 | 1.257470e+01 | 4.086022e+03 | [1.9165e+07, 1.9164e+07]
        4 |     1.814161e+01 | 1.326874e+01 | 8.289925e+02 | [5.4240e+07, 5.4239e+07]
        5 |     2.554775e+01 | 2.202544e+01 | 6.488855e+03 | [2.9530e+07, 2.9529e+07]
        6 |     3.303501e+01 | 2.551916e+01 | 3.206898e+03 | [2.7496e+08, 2.7496e+08]
        7 |     4.852149e+01 | 4.287816e+01 | 2.219582e+04 | [1.9712e+08, 1.9713e+08]
        8 |     6.020417e+01 | 5.539146e+01 | 1.865464e+04 | [8.1304e+08, 8.1306e+08]
        9 |     4.419408e+01 | 4.022170e+01 | 2.345220e+04 | [2.9385e+08, 2.9385e+08]
       10 |     4.453205e+01 | 4.281930e+01 | 1.707952e+04 | [1.3977e+08, 1.3977e+08]
       11 |     4.291943e+01 | 3.248607e+01 | 2.072112e+04 | [3.2710e+08, 3.2709e+08]
       12 |     2.894372e+01 | 2.061500e+01 | 1.286779e+04 | [1.2523e+08, 1.2519e+08]
       13 |     3.467974e+01 | 3.457642e+01 | 6.296511e+03 | [1.3461e+08, 1.3460e+08]
       14 |     5.029453e+01 | 3.497916e+01 | 1.620210e+04 | [3.1151e+08, 3.1132e+08]
       15 |     6.881504e+01 | 6.670430e+01 | 5.689167e+04 | [3.6071e+08, 3.6068e+08]
       16 |     1.049383e+02 | 8.275892e+01 | 5.003695e+04 | [9.9274e+08, 9.9271e+08]
       17 |     1.670326e+02 | 1.481493e+02 | 2.167276e+05 | [7.3486e+08, 7.3368e+08]
       18 |     2.147046e+02 | 1.697344e+02 | 4.077495e+05 | [2.8637e+09, 2.8620e+09]
       19 |     1.153257e+02 | 7.807376e+01 | 2.677173e+05 | [3.9865e+09, 3.9846e+09]

"""
    
    # Varying-penalty ADMM version data (placeholder - replace with your actual data)
    varying_penalty_output = """
========================================
ADMM Solver Status Summary
========================================
Current rho per robot: [5.0000e-01, 5.0000e-01]
----------------------------------------
Iteration | Primal Residual | Dual Residual | Cargo Cost | Robot Costs
----------|------------------|--------------|------------|-------------|
        0 |     2.139589e+00 | 3.111286e+00 | 2.052487e+02 | [1.1994e+05, 9.4890e+04]
        1 |     3.055632e+00 | 4.249705e+00 | 1.740554e+04 | [6.9995e+05, 7.0290e+05]
        2 |     1.496432e+00 | 3.823530e+00 | 5.633857e+03 | [3.4951e+06, 3.4950e+06]
        3 |     8.790695e-01 | 2.311659e+00 | 5.746123e+01 | [4.0590e+06, 4.0590e+06]
        4 |     5.966308e-01 | 1.449210e+00 | 8.579199e+00 | [3.5536e+06, 3.5536e+06]
        5 |     4.007161e-01 | 9.460080e-01 | 4.387755e+01 | [3.2193e+06, 3.2192e+06]
        6 |     5.303915e-01 | 7.264977e-01 | 7.997409e+01 | [3.0381e+06, 3.0381e+06]
        7 |     4.487209e-01 | 5.021150e-01 | 5.668841e+00 | [2.5315e+06, 2.5315e+06]
        8 |     3.697803e-01 | 4.556165e-01 | 9.770998e+00 | [2.6122e+06, 2.6122e+06]
        9 |     3.506250e-01 | 3.254431e-01 | 6.115578e+00 | [2.5330e+06, 2.5330e+06]
       10 |     4.744065e-01 | 5.889620e-01 | 1.214962e+02 | [2.4396e+06, 2.4396e+06]
       11 |     7.060486e-01 | 1.123098e+00 | 3.614893e+03 | [2.4375e+06, 2.4372e+06]
       12 |     7.479393e-02 | 8.755305e-01 | 5.772291e+00 | [2.4099e+06, 2.4099e+06]
       13 |     5.165975e-02 | 8.234270e-02 | 5.633850e+00 | [2.3912e+06, 2.3912e+06]
       14 |     4.162199e-02 | 6.964451e-02 | 5.647339e+00 | [2.3790e+06, 2.3790e+06]
       15 |     3.887457e-02 | 6.122333e-02 | 5.656320e+00 | [2.3690e+06, 2.3690e+06]
       16 |     3.451279e-02 | 5.325289e-02 | 5.665534e+00 | [2.3568e+06, 2.3568e+06]
       17 |     3.273608e-02 | 4.646648e-02 | 5.670354e+00 | [2.3422e+06, 2.3422e+06]
       18 |     3.092020e-02 | 4.060096e-02 | 5.672535e+00 | [2.3274e+06, 2.3274e+06]
       19 |     2.980610e-02 | 3.534032e-02 | 5.672451e+00 | [2.3142e+06, 2.3142e+06]


"""
    
    # Standard ADMM with 10 QP iterations (placeholder - replace with your actual data)
    standard_10qp_output = """
========================================
ADMM Solver Status Summary
========================================
Current rho per robot: [1.0000e+00, 1.0000e+00]
----------------------------------------
Iteration | Primal Residual | Dual Residual | Cargo Cost | Robot Costs
----------|------------------|--------------|------------|-------------|
        0 |     2.562556e+00 | 3.111752e+00 | 2.052380e+02 | [9.5018e+02, 9.2265e+02]
        1 |     3.712899e-01 | 4.012938e+00 | 7.252752e+00 | [2.0120e+06, 2.0120e+06]
        2 |     1.290302e-01 | 1.197680e+00 | 4.420625e+00 | [2.3210e+06, 2.3210e+06]
        3 |     1.471323e-01 | 8.358870e-01 | 4.674720e+00 | [2.3407e+06, 2.3407e+06]
        4 |     1.653124e-01 | 6.378816e-01 | 4.868613e+00 | [2.3338e+06, 2.3338e+06]
        5 |     1.552981e-01 | 5.004154e-01 | 5.065088e+00 | [2.3248e+06, 2.3248e+06]
        6 |     1.125191e-01 | 4.043784e-01 | 5.229372e+00 | [2.3201e+06, 2.3201e+06]
        7 |     7.907244e-02 | 3.397001e-01 | 5.328924e+00 | [2.3173e+06, 2.3173e+06]
        8 |     5.759100e-02 | 2.844232e-01 | 5.399679e+00 | [2.3140e+06, 2.3140e+06]
        9 |     4.332859e-02 | 2.418237e-01 | 5.452130e+00 | [2.3072e+06, 2.3072e+06]
       10 |     3.475502e-02 | 2.057781e-01 | 5.492043e+00 | [2.2990e+06, 2.2990e+06]
       11 |     2.889480e-02 | 1.772256e-01 | 5.524626e+00 | [2.2918e+06, 2.2918e+06]
       12 |     2.506523e-02 | 1.544987e-01 | 5.550586e+00 | [2.2852e+06, 2.2852e+06]
       13 |     2.251685e-02 | 1.353819e-01 | 5.568938e+00 | [2.2781e+06, 2.2781e+06]
       14 |     2.087482e-02 | 1.188788e-01 | 5.581862e+00 | [2.2709e+06, 2.2709e+06]
       15 |     1.989233e-02 | 1.045817e-01 | 5.590244e+00 | [2.2635e+06, 2.2635e+06]
       16 |     1.932573e-02 | 9.204542e-02 | 5.595320e+00 | [2.2567e+06, 2.2567e+06]
       17 |     1.905741e-02 | 8.101716e-02 | 5.597202e+00 | [2.2497e+06, 2.2497e+06]
       18 |     1.897002e-02 | 7.108473e-02 | 5.596635e+00 | [2.2429e+06, 2.2429e+06]
       19 |     1.900073e-02 | 6.183325e-02 | 5.594336e+00 | [2.2362e+06, 2.2362e+06]


"""
    
    # Parse all datasets
    iterations_std, primal_residuals_std = parse_admm_output(standard_output)
    iterations_cons, primal_residuals_cons = parse_admm_output(consensus_output)
    iterations_var, primal_residuals_var = parse_admm_output(varying_penalty_output)
    iterations_10qp, primal_residuals_10qp = parse_admm_output(standard_10qp_output)
    
    # Create the plot with all four variants
    fig, ax = plt.subplots(figsize=(12, 7))
    
    # Plot all variants with different colors, markers, and linestyles
    plot_primal_residual(iterations_std, primal_residuals_std, 
                        label='Standard ADMM', ax=ax, 
                        color='blue', marker='o', linestyle='-', linewidth=2, markersize=6)
    plot_primal_residual(iterations_cons, primal_residuals_cons, 
                        label='Consensus ADMM', ax=ax, 
                        color='red', marker='s', linestyle='--', linewidth=2, markersize=6)
    plot_primal_residual(iterations_var, primal_residuals_var, 
                        label='Varying-Penalty ADMM', ax=ax, 
                        color='green', marker='^', linestyle='-.', linewidth=2, markersize=6)
    plot_primal_residual(iterations_10qp, primal_residuals_10qp, 
                        label='Standard ADMM (10 QP)', ax=ax, 
                        color='orange', marker='d', linestyle=':', linewidth=2, markersize=6)
    
    # Save the plot
    plt.tight_layout()
    plt.savefig('admm_primal_residual.png', dpi=300, bbox_inches='tight')
    print(f"Plot saved as 'admm_primal_residual.png'")
    print(f"Standard ADMM: {len(iterations_std)} iterations, residual range: [{min(primal_residuals_std):.2e}, {max(primal_residuals_std):.2e}]")
    print(f"Consensus ADMM: {len(iterations_cons)} iterations, residual range: [{min(primal_residuals_cons):.2e}, {max(primal_residuals_cons):.2e}]")
    print(f"Varying-Penalty ADMM: {len(iterations_var)} iterations, residual range: [{min(primal_residuals_var):.2e}, {max(primal_residuals_var):.2e}]")
    print(f"Standard ADMM (10 QP): {len(iterations_10qp)} iterations, residual range: [{min(primal_residuals_10qp):.2e}, {max(primal_residuals_10qp):.2e}]")
    
    # Optionally show the plot
    # plt.show()

