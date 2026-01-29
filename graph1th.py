import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import matplotlib.lines as mlines

# --- データの読み込み ---
# ファイル名はご自身の環境に合わせて変更してください
df_sgd = pd.read_csv('sgdicp_t_multi.csv')
df_icp = pd.read_csv('icp_th.csv') # 回転版(厳しい条件)を使用

# --- データの前処理 (SGD) ---
# 回転角の絶対値変換
df_sgd['Abs_Theta'] = df_sgd['True_Theta_Deg'].abs()

# 0を除外し、50度以下に限定してソート
# (範囲はお好みで調整してください。ここでは安定している50度までにしています)
df_sgd_plot = df_sgd[(df_sgd['Abs_Theta'] != 0) & (df_sgd['Abs_Theta'] <= 50)].copy()
df_sgd_plot = df_sgd_plot.sort_values('Abs_Theta')

# 【重要】誤差評価用に「収束したデータ」のみを抽出
df_sgd_converged = df_sgd_plot[df_sgd_plot['Converged'] == 1].copy()

# --- データの前処理 (ICP) ---
df_icp['Abs_Theta'] = df_icp['True_Theta_Deg'].abs()
# ICPも同様にフィルタリング
df_icp_plot = df_icp[(df_icp['Abs_Theta'] != 0) & (df_icp['Abs_Theta'] <= 50)].copy()
df_icp_plot = df_icp_plot.sort_values('Abs_Theta')


# ==========================================
# グラフ1: 並進誤差比較 (Translational Error)
# ==========================================
plt.figure(figsize=(10, 6))

# 1. 箱ひげ図 (SGD-ICP: 収束データのみ)
sns.boxplot(x='Abs_Theta', y='Error_Trans', data=df_sgd_converged, 
            color='skyblue', whis=(0, 100))

# 2. 折れ線グラフ (ICP)
# ※ pointplotで平均値をプロットします
sns.pointplot(x='Abs_Theta', y='Error_Trans', data=df_icp_plot,
              color='red', markers='o', linestyles='-', errorbar=None,
              scale=0.8)

# ラベルとタイトル
plt.title('Translational Error vs. Initial Rotation: SGD-ICP (Converged) vs ICP')
plt.xlabel(r'Initial Rotation Magnitude |$\theta$| [deg]')
plt.ylabel('Translational Error [m]')
plt.grid(True, linestyle='--', alpha=0.7)

# カスタム凡例
blue_patch = mlines.Line2D([], [], color='skyblue', marker='s', linestyle='None', 
                           markersize=10, label='SGD-ICP (Converged)')
red_line = mlines.Line2D([], [], color='red', marker='o', 
                         markersize=6, label='ICP (Standard)')
plt.legend(handles=[blue_patch, red_line])

plt.tight_layout()
plt.savefig('comp_trans_error_theta.png')
plt.show()


# ==========================================
# グラフ2: 回転誤差比較 (Rotational Error)
# ==========================================
plt.figure(figsize=(10, 6))

# SGD (箱ひげ)
sns.boxplot(x='Abs_Theta', y='Error_Theta', data=df_sgd_converged, 
            color='skyblue', whis=(0, 100))

# ICP (折れ線)
sns.pointplot(x='Abs_Theta', y='Error_Theta', data=df_icp_plot,
              color='red', markers='o', linestyles='-', errorbar=None,
              scale=0.8)

plt.title('Rotational Error vs. Initial Rotation: SGD-ICP (Converged) vs ICP')
plt.xlabel(r'Initial Rotation Magnitude |$\theta$| [deg]')
plt.ylabel('Rotation Error [deg]')
plt.grid(True, linestyle='--', alpha=0.7)
plt.legend(handles=[blue_patch, red_line])

plt.tight_layout()
plt.savefig('comp_rot_error_theta.png')
plt.show()


# ==========================================
# グラフ3: 収束率 (Convergence Rate)
# ==========================================
# 角度ごとの収束率を計算
convergence_rates = df_sgd_plot.groupby('Abs_Theta')['Converged'].mean().reset_index()
convergence_rates['Rate'] = convergence_rates['Converged'] * 100

plt.figure(figsize=(10, 6))
sns.lineplot(x='Abs_Theta', y='Rate', data=convergence_rates, 
             marker='o', color='purple', linewidth=2, markersize=8)

plt.title('Convergence Rate of SGD-ICP vs. Initial Rotation')
plt.xlabel(r'Initial Rotation Magnitude |$\theta$| [deg]')
plt.ylabel('Convergence Rate [%]')
plt.ylim(-5, 105) # 0-100%が見やすいように
plt.grid(True, linestyle='--', alpha=0.7)

plt.tight_layout()
plt.savefig('convergence_rate_theta.png')
plt.show()