import cv2
import numpy as np

def detect_square_object(image):
    image = cv2.GaussianBlur(image,(5,5),0) #Gaussian
    cv2.imshow("image", image)
    # Chuyển đổi ảnh sang ảnh xám
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    cv2.imshow("gray", gray)
    # Áp dụng bộ lọc Canny để phát hiện biên
    ret, thresh = cv2.threshold(gray, 10, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
    cv2.imshow("thresh", thresh)
    edges = cv2.Canny(gray, 200, 255)
    cv2.imshow("edges", edges)
    # Tìm các đường biên hình vuông
    _, contours, hierarchy = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Lọc các đường biên hình vuông
    square_contours = []
    for contour in contours:
        perimet = cv2.arcLength(contour, True)
        #print("perimet", perimet)
        approx = cv2.approxPolyDP(contour, 0.02 * perimet, True)
        if 200 <= round(perimet,0) <= 235 and len(approx) == 4:
            print("perimett",perimet)
            square_contours.append(approx)
            rect = cv2.minAreaRect(contour)
            angle = rect[2]
            # angle = angle + 90
            angle = -angle
            print("angle: ",angle)
    # Vẽ đường biên hình vuông lên ảnh
    cv2.drawContours(image, square_contours, 0, (0, 255, 0), 2)
    text = f"Angle Yaw: {angle:.2f}"
    cv2.putText(image, text, (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
    
    # Hiển thị ảnh kết quả
    cv2.imshow("Square Object Detection", image)
    #return angle
  


# Đường dẫn đến file ảnh hoặc sử dụng camera
image_path = "Savedanh1_1.png"  # Đường dẫn đến file ảnh
# Đọc ảnh từ file hoặc từ camera
image = cv2.imread(image_path)
#image = cv2.flip(image,1)
image[:,:232] = (0,0,0)
image[:,306:] = (0,0,0)
image[335:, :] = (0,0,0)
cv2.imshow("origin", image)
# Phát hiện và xác định góc Yaw của vật thể hình vuông
a = detect_square_object(image)
print("a:", a)
cv2.waitKey(0)
cv2.destroyAllWindows()
