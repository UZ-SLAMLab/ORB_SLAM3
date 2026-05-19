import sys
import os

def fix_tum_timestamps(input_file, output_file):
    # 檢查輸入檔案是否存在
    if not os.path.exists(input_file):
        print(f"錯誤：找不到檔案 '{input_file}'")
        return

    try:
        with open(input_file, 'r') as fin, open(output_file, 'w') as fout:
            count = 0
            for line in fin:
                # 忽略空白行或註解
                if not line.strip() or line.startswith('#'):
                    fout.write(line)
                    continue
                
                parts = line.strip().split()
                # 確保是標準的 8 欄 TUM 格式 (時間 + 位置3 + 四元數4)
                if len(parts) == 8:
                    # 將奈秒轉換為秒，保留 6 位小數
                    timestamp_s = float(parts[0]) / 1e9
                    # 重新組合字串並寫入新檔案
                    new_line = f"{timestamp_s:.6f} {' '.join(parts[1:])}\n"
                    fout.write(new_line)
                    count += 1
                else:
                    # 如果不是 8 欄，原封不動照抄
                    fout.write(line)
                    
        print(f"轉換成功！共處理了 {count} 行資料。")
        print(f"新軌跡檔已儲存為：{output_file}")

    except Exception as e:
        print(f"轉換過程中發生錯誤：{e}")

if __name__ == "__main__":
    # 檢查使用者是否有輸入兩個檔案名稱
    if len(sys.argv) != 3:
        print("用法提示：")
        print("python fix_time.py <你的原始軌跡.txt> <你想命名的新軌跡.txt>")
        sys.exit(1)
        
    old_file = sys.argv[1]
    new_file = sys.argv[2]
    
    fix_tum_timestamps(old_file, new_file)