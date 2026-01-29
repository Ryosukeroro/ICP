# import pandas as pd
# import matplotlib.pyplot as plt
# import seaborn as sns
# import numpy as np

# # データの読み込み
# df_sgd = pd.read_csv('sgdicp_x.csv')
# df_icp = pd.read_csv('icp_x.csv')

# # 絶対値に変換
# df_sgd['Abs_DX'] = df_sgd['True_DX'].abs()
# df_icp['Abs_DX'] = df_icp['True_DX'].abs()

# df_sgd['Algorithm'] = 'SGD-ICP (Proposed)'
# df_icp['Algorithm'] = 'Standard ICP'

# df_combined = pd.concat([df_sgd, df_icp], ignore_index=True)

# # ★ 0を除外
# df_combined = df_combined[df_combined['Abs_DX'] > 0.001]

# # グラフ描画
# plt.figure(figsize=(10, 6))
# sns.set_style("whitegrid")

# sns.lineplot(
#     data=df_combined, 
#     x='Abs_DX', 
#     y='Time_ms', 
#     hue='Algorithm', 
#     style='Algorithm',
#     markers=True,
#     dashes=False,
#     palette=['#800080', '#008000'],
#     linewidth=2.5,
#     markersize=9,
#     errorbar=None  # ★ ci=None の代わり
# )

# plt.title('Computation Time vs. Displacement Magnitude', fontsize=16, pad=15, fontweight='bold')
# plt.xlabel('Initial Displacement Magnitude |X| [m]', fontsize=13, fontweight='bold')
# plt.ylabel('Computation Time [ms]', fontsize=13, fontweight='bold')

# plt.xticks(np.arange(0, 1.1, 0.1), fontsize=11)
# plt.yticks(fontsize=11)
# plt.ylim(0, df_combined['Time_ms'].max() * 1.1)

# plt.legend(title=None, fontsize=12, loc='upper left', frameon=True, shadow=True)
# plt.grid(True, which='major', linestyle='-', linewidth=0.8, alpha=0.8)

# plt.tight_layout()
# plt.savefig('absolute_displacement_comparison_no_zero.png', dpi=300)
# plt.show()
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np
import matplotlib.lines as mlines

# データの読み込み
df_sgd = pd.read_csv('sgdicp_x_multi.csv')
df_icp = pd.read_csv('icp_x.csv')

# --- データ前処理 ---
# 絶対値に変換
df_sgd['Abs_DX'] = df_sgd['True_DX'].abs()
df_icp['Abs_DX'] = df_icp['True_DX'].abs()

# ★重要: SGD-ICPは「収束したデータ (Converged == 1)」のみを抽出
# これにより、純粋なアルゴリズムの処理速度を評価する
df_sgd_converged = df_sgd[df_sgd['Converged'] == 1].copy()
df_sgd_converged['Algorithm'] = 'SGD-ICP (Proposed)'

# ICPはすべて使用
df_icp['Algorithm'] = 'Standard ICP'

# 結合
df_combined = pd.concat([df_sgd_converged, df_icp], ignore_index=True)

# 0を除外 (ズレなしの場合)
df_combined = df_combined[df_combined['Abs_DX'] > 0.001]

# --- グラフ描画 ---
plt.figure(figsize=(10, 6))
sns.set_style("whitegrid")

# 平均値を線で結ぶ (errorbar=Noneで信頼区間を消してスッキリさせる)
sns.lineplot(
    data=df_combined, 
    x='Abs_DX', 
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
plt.title('Computation Time vs. Initial Displacement', fontsize=16, pad=15, fontweight='bold')
plt.xlabel('Initial Displacement Magnitude |X| [m]', fontsize=13, fontweight='bold')
plt.ylabel('Computation Time [ms]', fontsize=13, fontweight='bold')

# 軸の設定
plt.xticks(np.arange(0, 1.1, 0.1), fontsize=11)
plt.yticks(fontsize=11)
plt.ylim(0, 55) # ICPが50msくらいなので少し余裕を持たせる

# 凡例
plt.legend(title=None, fontsize=12, loc='upper left', frameon=True, shadow=True)
plt.grid(True, which='major', linestyle='--', linewidth=0.8, alpha=0.7)

plt.tight_layout()
plt.savefig('time_comparison_converged_only.png', dpi=300)
# plt.show()
