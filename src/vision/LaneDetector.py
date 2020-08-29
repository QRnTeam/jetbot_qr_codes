import cv2

delay = 2000

img = cv2.imread("demo.jpg")
cv2.imshow("Output",img)
cv2.waitKey(delay)

imgGray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
cv2.imshow("Gray",imgGray)
cv2.waitKey(delay)

imgBlur = cv2.GaussianBlur(imgGray,(5,5),0)
cv2.imshow("Blurred",imgBlur)
cv2.waitKey(delay)

imgCanny = cv2.Canny(imgBlur, 100,200)
cv2.imshow("Canny",imgCanny)
cv2.waitKey(delay)