import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import matplotlib.lines as mlines

# --- データの読み込み ---
df_sgd = pd.read_csv('sgdicp_y_multi.csv')
df_icp = pd.read_csv('icp_y.csv') # ★ここにICPのY版データを読み込む

# --- データの前処理 (SGD) ---
# Y軸の絶対値変換
df_sgd['Abs_True_DY'] = df_sgd['True_DY'].abs()

# 0を除外し、ソート
df_sgd_plot = df_sgd[df_sgd['Abs_True_DY'] != 0].copy()
df_sgd_plot = df_sgd_plot.sort_values('Abs_True_DY')

# 【重要】誤差評価用に「収束したデータ」のみを抽出
df_sgd_converged = df_sgd_plot[df_sgd_plot['Converged'] == 1].copy()

#--- データの前処理 (ICP) ※データがある場合 ---
df_icp['Abs_True_DY'] = df_icp['True_DY'].abs()
df_icp_plot = df_icp[df_icp['Abs_True_DY'] != 0].copy()
df_icp_plot = df_icp_plot.sort_values('Abs_True_DY')


# ==========================================
# グラフ1: 並進誤差比較 (Translational Error)
# ==========================================
plt.figure(figsize=(10, 6))

# 1. 箱ひげ図 (SGD-ICP: 収束データのみ)
sns.boxplot(x='Abs_True_DY', y='Error_Trans', data=df_sgd_converged, 
            color='skyblue', whis=(0, 100))

#2. 折れ線グラフ (ICP)
sns.pointplot(x='Abs_True_DY', y='Error_Trans', data=df_icp_plot,
              color='red', markers='o', linestyles='-', errorbar=None,
              markersize=6, linewidth=1.5)

# ラベルとタイトル
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
plt.show()


# ==========================================
# グラフ2: 収束率 (Convergence Rate)
# ==========================================
# 距離ごとの収束率を計算
convergence_rates = df_sgd_plot.groupby('Abs_True_DY')['Converged'].mean().reset_index()
convergence_rates['Rate'] = convergence_rates['Converged'] * 100

plt.figure