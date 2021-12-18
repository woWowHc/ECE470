import cv2
mask = cv2.imread("images/cat.jpeg")
mask = cv2.pyrDown(mask,(60,60))
print(mask.shape)
#mask =cv2.resize(mask,(30,30))
cv2.imshow("1",mask)

# waits for user to press any key
# (this is necessary to avoid Python kernel form crashing)
cv2.waitKey(0)

# closing all open windows
cv2.destroyAllWindows()

