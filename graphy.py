import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np
import matplotlib.lines as mlines

# データの読み込み
# ※ C++で出力したファイル名に合わせてください (例: icp_y.csv または icp_y_multi.csv)
df_sgd = pd.read_csv('sgdicp_y_multi.csv')
df_icp = pd.read_csv('icp_y.csv') # もしファイル名が icp_y.csv ならここを変更

# --- データ前処理 ---
# 絶対値に変換 (True_DY を使用)
df_sgd['Abs_DY'] = df_sgd['True_DY'].abs()
df_icp['Abs_DY'] = df_icp['True_DY'].abs()

# ★重要: SGD-ICPは「収束したデータ (Converged == 1)」のみを抽出
df_sgd_converged = df_sgd[df_sgd['Converged'] == 1].copy()
df_sgd_converged['Algorithm'] = 'SGD-ICP (Proposed)'

# ICPはすべて使用
df_icp['Algorithm'] = 'Standard ICP'

# 結合
df_combined = pd.concat([df_sgd_converged, df_icp], ignore_index=True)

# 0を除外 (ズレなしの場合)
df_combined = df_combined[df_combined['Abs_DY'] > 0.001]

# --- グラフ描画 ---
plt.figure(figsize=(10, 6))
sns.set_style("whitegrid")

# 平均値を線で結ぶ
sns.lineplot(
    data=df_combined, 
    x='Abs_DY',      # X軸を Abs_DY に変更
    y='Time_ms', 
    hue='Algorithm', 
    style='Algorithm',
    markers=True,
    dashes=False,
    palette=['#800080', '#008000'], # 紫と緑 (統一)
    linewidth=2.5,
    markersize=9,
    errorbar=None
)

# タイトルとラベル (Y軸仕様に変更)
plt.title('Computation Time vs. Initial Y Displacement', fontsize=16, pad=15, fontweight='bold')
plt.xlabel('Initial Displacement Magnitude |Y| [m]', fontsize=13, fontweight='bold')
plt.ylabel('Computation Time [ms]', fontsize=13, fontweight='bold')

# 軸の設定
plt.xticks(np.arange(0, 1.1, 0.1), fontsize=11)
plt.yticks(fontsize=11)
plt.ylim(0, 55) # 必要に応じて調整してください

# 凡例
plt.legend(title=None, fontsize=12, loc='upper left', frameon=True, shadow=True)
plt.grid(True, which='major', linestyle='--', linewidth=0.8, alpha=0.7)

plt.tight_layout()
# ファイル名もY用に変更
plt.savefig('time_comparison_y_converged_only.png', dpi=300)
# plt.show()