import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import matplotlib.lines as mlines

# Load data
df_sgd = pd.read_csv('sgdicp_x_multi.csv')
df_icp = pd.read_csv('icp_x.csv')

# Process SGD data
df_sgd['Abs_True_DX'] = df_sgd['True_DX'].abs()
df_sgd_plot = df_sgd[df_sgd['Abs_True_DX'] != 0].copy()
df_sgd_plot = df_sgd_plot.sort_values('Abs_True_DX')

# Process ICP data
df_icp['Abs_True_DX'] = df_icp['True_DX'].abs()
df_icp_plot = df_icp[df_icp['Abs_True_DX'] != 0].copy()
df_icp_plot = df_icp_plot.sort_values('Abs_True_DX')

# --- Plot 1: Translational Error ---
plt.figure(figsize=(10, 6))

# Boxplot for SGD-ICP
sns.boxplot(x='Abs_True_DX', y='Error_Trans', data=df_sgd_plot, color='skyblue', whis=(0, 100))

# Pointplot for ICP (Mean of errors for each Abs_True_DX level)
# 修正点: scale=0.7 を削除し、markersizeとlinewidthでサイズ調整
sns.pointplot(x='Abs_True_DX', y='Error_Trans', data=df_icp_plot,
              color='red', markers='o', linestyles='-', errorbar=None,
              markersize=6, linewidth=1.5)

plt.title('Translational Error: SGD-ICP vs ICP')
plt.xlabel('Initial Deviation Magnitude |X| [m]')
plt.ylabel('Translational Error [m]')
plt.grid(True, linestyle='--', alpha=0.7)

# Custom legend
blue_patch = mlines.Line2D([], [], color='skyblue', marker='s', linestyle='None', markersize=10, label='SGD-ICP')
red_line = mlines.Line2D([], [], color='red', marker='o', markersize=6, label='ICP (Mean)')
plt.legend(handles=[blue_patch, red_line])

plt.tight_layout()
plt.savefig('comp_trans_error.png')
plt.close()

# --- Plot 2: Rotational Error ---
plt.figure(figsize=(10, 6))

# Boxplot for SGD-ICP
sns.boxplot(x='Abs_True_DX', y='Error_Theta', data=df_sgd_plot, color='skyblue', whis=(0, 100))

# Pointplot for ICP
# 修正点: 同様に scale=0.7 を削除
sns.pointplot(x='Abs_True_DX', y='Error_Theta', data=df_icp_plot,
              color='red', markers='o', linestyles='-', errorbar=None,
              markersize=6, linewidth=1.5)

plt.title('Rotational Error: SGD-ICP vs ICP')
plt.xlabel('Initial Deviation Magnitude |X| [m]')
plt.ylabel('Rotational Error [deg]')
plt.grid(True, linestyle='--', alpha=0.7)

plt.legend(handles=[blue_patch, red_line])

plt.tight_layout()
plt.savefig('comp_rot_error.png')
plt.close()