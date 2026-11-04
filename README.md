# ROUTING AUTOMATED DELIVERY ROBOTS ON THE CAMPUS OF THE NATIONAL ECONOMICS UNIVERSITY

**Course:** Artificial Intelligence - National Economics University (NEU)  
**Group:** 3 (Trí tuệ nhân tạo 67A)  

**Members:**
- Hoàng Anh Tú
- Cao Đình Bình
- Phạm Ngọc Danh
- Nguyễn Trịnh Anh Tuấn

---

## 💻 Môi trường yêu cầu (Environment)
Chương trình được viết hoàn toàn bằng Python và chỉ sử dụng các thư viện tiêu chuẩn (Standard Library). Do đó, bạn không cần cài đặt thêm bất kỳ gói bên ngoài nào (không cần `pip install`).

- **Ngôn ngữ:** Python 3.7 trở lên (khuyến nghị Python 3.9+ để tối ưu hiệu suất).
- **Thư viện sử dụng:** `heapq`, `time`, `collections`, `functools`, `itertools` (Tất cả đều có sẵn trong Python).
- **Hệ điều hành:** Windows, macOS, hoặc Linux.

---

## 🚀 Hướng dẫn tái tạo kết quả (How to Reproduce Results)

Để chạy thực nghiệm và xem các bảng kết quả so sánh các thuật toán (BFS, GBFS, A*), vui lòng làm theo các bước sau:

1. **Tải mã nguồn:** 
   Đảm bảo bạn đã lưu tệp mã nguồn với tên là `robot_giao_hang_NEU_AI.py` vào một thư mục trên máy tính.

2. **Mở Terminal hoặc Command Prompt:**
   Mở công cụ dòng lệnh (Terminal trên macOS/Linux hoặc CMD/PowerShell trên Windows) và di chuyển (`cd`) đến thư mục chứa tệp mã nguồn.

3. **Chạy chương trình:**
   Nhập lệnh sau và nhấn Enter:
   ```bash
   python robot_giao_hang_NEU_AI.py
   ```
   *(Lưu ý: Nếu hệ thống của bạn sử dụng `python3` làm lệnh mặc định, hãy dùng `python3 robot_giao_hang_NEU_AI.py`)*

4. **Xem kết quả:**
   Chương trình sẽ tự động thực hiện 3 thí nghiệm (như đã cấu hình trong mã, lặp lại 50 lần để đo thời gian trung bình) và in ra terminal các bảng kết quả chi tiết bao gồm: số bước, số node mở rộng, dung lượng bộ nhớ mở lớn nhất (OPEN Max), và thời gian chạy trung bình.
