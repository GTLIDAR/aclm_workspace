#!/usr/bin/env python3
"""
Script to plot MPC residual data from ConsensusADMMSolver.
Loads data from script/data/mpc_residuals.txt and plots the residuals over time.
"""

import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path
from typing import List, Tuple


def load_mpc_residuals(filepath: str) -> Tuple[List[float], List[float], List[List[float]]]:
    """
    Load MPC residual data from file.
    
    Args:
        filepath: Path to the mpc_residuals.txt file
        
    Returns:
        Tuple of (init_times, total_primal_residuals, per_constraint_residuals)
        where per_constraint_residuals is a list of lists (one per MPC solve)
    """
    init_times = []
    total_primal_residuals = []
    per_constraint_residuals = []
    
    with open(filepath, 'r') as f:
        for line in f:
            # Skip comment lines
            if line.strip().startswith('#'):
                continue
            
            # Skip empty lines
            if not line.strip():
                continue
            
            # Parse the line: initTime totalPrimalResidual perConstraintResidual_0 perConstraintResidual_1 ...
            parts = line.strip().split()
            if len(parts) < 2:
                continue
            
            init_time = float(parts[0])
            total_primal = float(parts[1])
            per_constraint = [float(x) for x in parts[2:]]
            
            init_times.append(init_time)
            total_primal_residuals.append(total_primal)
            per_constraint_residuals.append(per_constraint)
    
    return init_times, total_primal_residuals, per_constraint_residuals


def plot_mpc_residuals(init_times: List[float], 
                       total_primal_residuals: List[float],
                       per_constraint_residuals: List[List[float]],
                       output_file: str = 'mpc_residuals.png'):
    """
    Plot MPC residual data.
    
    Args:
        init_times: List of initial times for each MPC solve
        total_primal_residuals: List of total primal residuals
        per_constraint_residuals: List of per-constraint residual lists
        output_file: Output filename for the plot
    """
    fig, axes = plt.subplots(2, 1, figsize=(12, 10), sharex=True)
    
    # Convert to numpy arrays for easier indexing
    init_times = np.array(init_times)
    total_primal_residuals = np.array(total_primal_residuals)
    
    # Plot 1: Total primal residual
    ax1 = axes[0]
    ax1.semilogy(init_times, total_primal_residuals, 'b-o', linewidth=2, markersize=6, label='Total Primal Residual')
    ax1.set_ylabel('Total Primal Residual', fontsize=12)
    ax1.set_title('MPC Primal Residual Convergence Over Time', fontsize=14)
    ax1.grid(True, alpha=0.3)
    ax1.legend()
    
    # Plot 2: Per-constraint primal residuals
    ax2 = axes[1]
    
    # Determine number of constraints (robots) from the data
    if per_constraint_residuals:
        num_constraints = len(per_constraint_residuals[0])
        colors = plt.cm.tab10(np.linspace(0, 1, num_constraints))
        markers = ['o', 's', '^', 'd', 'v', '<', '>', 'p', '*', 'h']
        
        for i in range(num_constraints):
            constraint_residuals = [res[i] if i < len(res) else 0.0 for res in per_constraint_residuals]
            constraint_residuals = np.array(constraint_residuals)
            
            marker = markers[i % len(markers)]
            color = colors[i]
            ax2.semilogy(init_times, constraint_residuals, 
                        marker=marker, linestyle='-', linewidth=2, markersize=6,
                        color=color, label=f'Robot {i} Primal Residual')
    
    ax2.set_xlabel('Initial Time [s]', fontsize=12)
    ax2.set_ylabel('Per-Constraint Primal Residual', fontsize=12)
    ax2.set_title('Per-Constraint Primal Residual Convergence Over Time', fontsize=14)
    ax2.grid(True, alpha=0.3)
    ax2.legend()
    
    plt.tight_layout()
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"Plot saved as '{output_file}'")
    print(f"Loaded {len(init_times)} MPC solve records")
    print(f"Time range: [{min(init_times):.2f}, {max(init_times):.2f}] seconds")
    print(f"Total primal residual range: [{min(total_primal_residuals):.2e}, {max(total_primal_residuals):.2e}]")


if __name__ == "__main__":
    # Default file path
    script_dir = Path(__file__).parent
    data_file = script_dir / "data" / "mpc_residuals.txt"
    
    # Check if file exists
    if not data_file.exists():
        print(f"Error: Data file not found at {data_file}")
        print("Please run the MPC solver first to generate the data file.")
        exit(1)
    
    # Load and plot the data
    init_times, total_primal_residuals, per_constraint_residuals = load_mpc_residuals(str(data_file))
    
    if not init_times:
        print("Error: No data found in the file.")
        exit(1)
    
    # Create output plot in the script directory
    output_file = script_dir / "mpc_residuals.png"
    plot_mpc_residuals(init_times, total_primal_residuals, per_constraint_residuals, str(output_file))
    
    # Optionally show the plot
    # plt.show()

