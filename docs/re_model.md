Mô tả cấu trúc robot
    1. Hệ truyền động (differential drive): bao gồm 2 bánh xe chủ động ở phía sau và 1 bánh xe con lăn(caster wheel) hình cầu phía trước để giữ thăng bằng và trượt điều hướng.
    2. Tầng đáy (chassis): tám mica hình chữ nhật thứ nhất. đây là khung gầm chính, nơi sẽ gắng động cơ, chứa pin và chịu lực chính.
    3. Tấng giữa (Deck 2): tấm mica hình chữ nhật thứ hai, lắp song song và cách tầng ddya một khoảng cách bằng các trụ đồng (standoffs). Tầng này dùng để chứa mạch điều khiển (stm32, orange pi, driver motor)
    4. Tầng trên cùng (Lidar deck): một tấm mica hình tròn nhỏ gọn, nằm trên cùng bằng các trụ đồng ngắn hơn.
    5. cảm biến (lidar): được gắn trung tâm trên tấm tròn để có góc nhìn quét 360 độ không bị cản trở bởi bất kì vật cản nào trên thân robot.
Hướng dẫn step by step tạo URDF/Xacro cho mô hình này
    thay vì viết một file dài, chúng ta sẽ sử dụng sức mạnh của xacro để chia nhỏ file dễ quản lý
    Bước 1: khởi tạo điểm gốc (base footprint & base link)
    mọi robot đều cần một điểm gốc để hệ thống ROS định vị nó trong không gian.
    - base_footprint: là một điểm ảo nằm sát trên mặt đất (hình chiếu của robot xuống sàn)
    - base_link: tâm điểm của tấm mica tầng đáy (chassis)
    Bước 2: xây dựng tầng đáy (chassis)
    từ base_link, chúng ta sẽ gắn tấm mica đầu tiên vào
    - khai báo thẻ <link> dạng hình hộp (box) với kích thước tấm mica
    - tạo joint cố định để nối chassis vào base_link
    Bước 3: Thêm hệ truyền động (hai bánh sau & con lăn trước)
    - hai bánh sau: tạo 2 link hình trụ (cylinder) cho bánh xe. dùng joint dạng continous(xoay liên tục) nối vào base_link. trục xoay (axis) sẽ là trục y.
    - con lăn: tạo 1 link hình cầu gắn vào phía trước của chassis dùng joint dạng fixed
    Bước 4: Chồng các tầng tiếp theo (Điểm mấu chốt của mô hình này)
    - Đây là lúc bạn áp dụng thiết kế nhiều tầng. Nguyên lý rất đơn giản: Tầng trên nối cố định vào tầng dưới thông qua trục Z (chiều cao của trụ đồng).
    Bước 5: Lắp đặt "Mắt thần" (Cảm biến LiDAR)
    - Cuối cùng, bạn tạo một khối đại diện cho LiDAR (thường là một hình trụ nhỏ) và gắn nó lên tâm của tấm lidar_deck.
    - Tên của link này thường được đặt chuẩn là laser_frame.
  