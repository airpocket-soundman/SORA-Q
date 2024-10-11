import cv2
import os

# カレントディレクトリの取得
current_dir = os.getcwd()

# カレントディレクトリ内のファイルを取得
for filename in os.listdir(current_dir):
    if filename.endswith(".JPG"):
        # 画像の読み込み
        img = cv2.imread(filename)

        if img is not None:
            # 90度回転（時計回り）
            rotated_img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)

            # 回転した画像の保存
            cv2.imwrite(filename, rotated_img)
            print(f"{filename} を保存しました")

print("すべての画像の処理が完了しました。")
