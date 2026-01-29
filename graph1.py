import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import matplotlib.lines as mlines

# データの読み込み
df_sgd = pd.read_csv('sgdicp_x_multi.csv')
df_icp = pd.read_csv('icp_x.csv')

# --- データの前処理 ---
# SGD-ICPデータの処理（絶対値変換、0除外、ソート）
df_sgd['Abs_True_DX'] = df_sgd['True_DX'].abs()
df_sgd_plot = df_sgd[df_sgd['Abs_True_DX'] != 0].copy()
df_sgd_plot = df_sgd_plot.sort_values('Abs_True_DX')

# ICPデータの処理（絶対値変換、0除外、ソート）
df_icp['Abs_True_DX'] = df_icp['True_DX'].abs()
df_icp_plot = df_icp[df_icp['Abs_True_DX'] != 0].copy()
df_icp_plot = df_icp_plot.sort_values('Abs_True_DX')

# --- グラフ描画: 並進誤差 (Translational Error) ---
plt.figure(figsize=(10, 6))

# 1. 箱ひげ図 (SGD-ICP)
sns.boxplot(x='Abs_True_DX', y='Error_Trans', data=df_sgd_plot, 
            color='skyblue', whis=(0, 100))

# 2. 折れ線グラフ (ICP)
# 【修正】scale を削除し、markersize と linewidth でサイズを指定
sns.pointplot(x='Abs_True_DX', y='Error_Trans', data=df_icp_plot,
              color='red', markers='o', linestyles='-', errorbar=None,
              markersize=6, linewidth=1.5)

# ラベルとタイトル
plt.title('Translational Error: SGD-ICP vs ICP')
plt.xlabel('Initial Deviation Magnitude |X| [m]')
plt.ylabel('Translational Error [m]')
plt.grid(True, linestyle='--', alpha=0.7)

# カスタム凡例の作成
blue_patch = mlines.Line2D([], [], color='skyblue', marker='s', linestyle='None', 
                           markersize=10, label='SGD-ICP')
red_line = mlines.Line2D([], [], color='red', marker='o', 
                         markersize=6, label='ICP (Mean)')
plt.legend(handles=[blue_patch, red_line])

plt.tight_layout()
plt.savefig('comp_trans_error.png')
# plt.show()