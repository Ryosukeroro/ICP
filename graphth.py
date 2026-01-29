import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np

# データの読み込み
df_sgd = pd.read_csv('sgdicp_t_multi.csv')
df_icp = pd.read_csv('icp_th.csv')

# --- データ前処理 ---
# 絶対値に変換
df_sgd['Abs_Theta'] = df_sgd['True_Theta_Deg'].abs()
df_icp['Abs_Theta'] = df_icp['True_Theta_Deg'].abs()

# ★SGD-ICPは「収束したデータ (Converged == 1)」のみ抽出
df_sgd_converged = df_sgd[df_sgd['Converged'] == 1].copy()
df_sgd_converged['Algorithm'] = 'SGD-ICP (Proposed)'

# ICPはすべて使用
df_icp['Algorithm'] = 'Standard ICP'

# 結合
df_combined = pd.concat([df_sgd_converged, df_icp], ignore_index=True)

# 0を除外 & ★ここを追加: 50度以下に限定
df_combined = df_combined[(df_combined['Abs_Theta'] > 0.001) & (df_combined['Abs_Theta'] <= 50)]

# --- グラフ描画 ---
plt.figure(figsize=(10, 6))
sns.set_style("whitegrid")

# 平均値を線で結ぶ
sns.lineplot(
    data=df_combined, 
    x='Abs_Theta', 
    y='Time_ms', 
    hue='Algorithm', 
    style='Algorithm',
    markers=True,
    dashes=False,
    palette=['#800080', '#008000'], # 紫と緑
    linewidth=2.5,
    markersize=9,
    errorbar=None
)

# タイトルとラベル
plt.title('Computation Time vs. Initial Rotation Angle (up to 50°)', fontsize=16, pad=15, fontweight='bold')
plt.xlabel(r'Initial Rotation Magnitude |$\theta$| [deg]', fontsize=13, fontweight='bold')
plt.ylabel('Computation Time [ms]', fontsize=13, fontweight='bold')

# 軸の設定 (0〜50度まで 5度刻み)
plt.xticks(np.arange(0, 55, 5), fontsize=11)
plt.yticks(fontsize=11)

# Y軸の範囲 (必要に応じて調整)
plt.ylim(0, df_combined['Time_ms'].max() * 1.1) 

# 凡例
plt.legend(title=None, fontsize=12, loc='upper left', frameon=True, shadow=True)
plt.grid(True, which='major', linestyle='--', linewidth=0.8, alpha=0.7)

plt.tight_layout()
plt.savefig('time_comparison_theta_50deg.png', dpi=300)
plt.show()