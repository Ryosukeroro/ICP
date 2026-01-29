import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import matplotlib.lines as mlines
import os

# --- データの読み込み ---
df_sgd = pd.read_csv('sgdicp_y_multi.csv')
icp_file = 'icp_y.csv'

# ICPファイルがあるか確認して読み込み
if os.path.exists(icp_file):
    df_icp = pd.read_csv(icp_file)
else:
    df_icp = None
    print("Warning: icp_y.csv not found.")

# --- データの前処理 ---

# 1. SGDデータ処理
df_sgd['Abs_True_DY'] = df_sgd['True_DY'].abs()
# 0mのデータを除外
df_sgd_plot = df_sgd[df_sgd['Abs_True_DY'] != 0].copy()

# 【重要】誤差プロット用に「収束したデータ」のみを抽出
df_sgd_converged = df_sgd_plot[df_sgd_plot['Converged'] == 1].copy()
df_sgd_converged = df_sgd_converged.sort_values('Abs_True_DY')

# 2. ICPデータ処理
if df_icp is not None:
    df_icp['Abs_True_DY'] = df_icp['True_DY'].abs()
    df_icp_plot = df_icp[df_icp['Abs_True_DY'] != 0].copy()
    df_icp_plot = df_icp_plot.sort_values('Abs_True_DY')
else:
    df_icp_plot = None

# ==========================================
# グラフ1: 収束率 (Convergence Rate)
# ==========================================
convergence_rates = df_sgd_plot.groupby('Abs_True_DY')['Converged'].mean().reset_index()
convergence_rates['Rate'] = convergence_rates['Converged'] * 100

plt.figure(figsize=(10, 5))
sns.barplot(x='Abs_True_DY', y='Rate', data=convergence_rates, 
            color='lightgreen', edgecolor='black', alpha=0.7)
plt.title('Convergence Rate vs Initial Deviation Y')
plt.xlabel('Initial Deviation Magnitude |Y| [m]')
plt.ylabel('Convergence Rate [%]')
plt.ylim(0, 105)
plt.grid(axis='y', linestyle='--', alpha=0.7)
plt.tight_layout()
plt.savefig('convergence_rate_y.png')
plt.close()
print("Saved convergence_rate_y.png")

# ==========================================
# グラフ2: 並進誤差 (Translational Error)
# ==========================================
plt.figure(figsize=(10, 6))

# SGDの箱ひげ図 (収束データのみ)
sns.boxplot(x='Abs_True_DY', y='Error_Trans', data=df_sgd_converged, 
            color='skyblue', whis=(0, 100))

# ICPの折れ線グラフ
if df_icp_plot is not None:
    # 【修正】scaleを削除し、markersizeとlinewidthを使用
    sns.pointplot(x='Abs_True_DY', y='Error_Trans', data=df_icp_plot,
                  color='red', markers='o', linestyles='-', errorbar=None,
                  markersize=6, linewidth=1.5)

plt.title('Translational Error: SGD-ICP (Converged) vs ICP')
plt.xlabel('Initial Deviation Magnitude |Y| [m]')
plt.ylabel('Translational Error [m]')
plt.grid(True, linestyle='--', alpha=0.7)

# カスタム凡例
blue_patch = mlines.Line2D([], [], color='skyblue', marker='s', linestyle='None', 
                           markersize=10, label='SGD-ICP (Converged)')
red_line = mlines.Line2D([], [], color='red', marker='o', 
                         markersize=6, label='ICP (Mean)')
plt.legend(handles=[blue_patch, red_line])

plt.tight_layout()
plt.savefig('comp_trans_error_y.png')
plt.close()
print("Saved comp_trans_error_y.png")

# ==========================================
# グラフ3: 回転誤差 (Rotational Error)
# ==========================================
plt.figure(figsize=(10, 6))

# SGDの箱ひげ図 (収束データのみ)
sns.boxplot(x='Abs_True_DY', y='Error_Theta', data=df_sgd_converged, 
            color='skyblue', whis=(0, 100))

# ICPの折れ線グラフ
if df_icp_plot is not None:
    # 【修正】scaleを削除し、markersizeとlinewidthを使用
    sns.pointplot(x='Abs_True_DY', y='Error_Theta', data=df_icp_plot,
                  color='red', markers='o', linestyles='-', errorbar=None,
                  markersize=6, linewidth=1.5)

plt.title('Rotational Error: SGD-ICP (Converged) vs ICP')
plt.xlabel('Initial Deviation Magnitude |Y| [m]')
plt.ylabel('Rotational Error [deg]')
plt.grid(True, linestyle='--', alpha=0.7)

# 凡例 (共通)
plt.legend(handles=[blue_patch, red_line])

plt.tight_layout()
plt.savefig('comp_rot_error_y.png')
plt.close()
print("Saved comp_rot_error_y.png")